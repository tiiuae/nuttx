/****************************************************************************
 * arch/arm/src/stm32n6/stm32_otgdev.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32_N6_OTGDEV

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_pwr.h"
#include "stm32_otg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_TRACEERR_NOTREADY         1
#define STM32_TRACEERR_INVALIDPARMS     2
#define STM32_TRACEERR_NOTBOUND         3
#define STM32_TRACEERR_INITFAILED       4
#define STM32_TRACEERR_CLEANUPFAILED     5

#define STM32_USB_WAIT_US               10000u
#define STM32_USB_SUPPLY_WAIT_US        100000u
#define STM32_USB_HSE_WAIT_US           100000u
#define STM32_USB_PHY_DELAY_US          50u
#define STM32_USB_MODE_DELAY_US         25000u
#define STM32_USB_CLOCKS                (STM32_OTG_RCC_EN | \
                                         STM32_OTG_RCC_PHY_EN)
#define STM32_USB_RESETS                (STM32_OTG_RCC_RST | \
                                         STM32_OTG_RCC_PHY_RST | \
                                         STM32_OTG_RCC_PHYCTL_RST)
#define STM32_USB_HSE_CONFIG_MASK       (RCC_HSECFGR_HSEBYP | \
                                         RCC_HSECFGR_HSEEXT | \
                                         RCC_HSECFGR_HSEDIV2SEL)

#if STM32_HCLK_FREQUENCY <= 30000000
#  error "STM32N6 USB requires HCLK above 30 MHz"
#endif

#if BOARD_USB_HSE_FREQUENCY != 48000000
#  error "This USB clock contract requires a 48 MHz HSE crystal"
#endif

#if BOARD_USB_HSE_STABILIZATION_US < 2000 || \
    BOARD_USB_HSE_STABILIZATION_US > 100000
#  error "Define a bounded, board-qualified HSE settling allowance"
#endif

/* FIFO sizes are bytes at the configuration boundary, words in hardware.
 * Disabled TX slots still reserve the RM0486 minimum of 16 words.
 */

#ifndef CONFIG_USBDEV_EP0_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP0_TXFIFO_SIZE 256
#endif
#ifndef CONFIG_USBDEV_EP1_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP1_TXFIFO_SIZE 64
#endif
#ifndef CONFIG_USBDEV_EP2_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP2_TXFIFO_SIZE 1024
#endif
#ifndef CONFIG_USBDEV_EP3_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP3_TXFIFO_SIZE 0
#endif
#ifndef CONFIG_USBDEV_EP4_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP4_TXFIFO_SIZE 0
#endif
#ifndef CONFIG_USBDEV_EP5_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP5_TXFIFO_SIZE 0
#endif
#ifndef CONFIG_USBDEV_EP6_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP6_TXFIFO_SIZE 0
#endif
#ifndef CONFIG_USBDEV_EP7_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP7_TXFIFO_SIZE 0
#endif
#ifndef CONFIG_USBDEV_EP8_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP8_TXFIFO_SIZE 0
#endif

#define STM32_USB_RX_WORDS              512u
#define STM32_USB_TX_WORDS(n)            ((n) == 0 ? 16u : ((n) + 3u) / 4u)
#define STM32_USB_FIFO_VALID(n)          ((n) >= 0 && (n) <= 4096 && \
                                         STM32_USB_TX_WORDS(n) >= 16)
#define STM32_USB_FIFO_TOTAL            (STM32_USB_RX_WORDS + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP0_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP1_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP2_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP3_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP4_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP5_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP6_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP7_TXFIFO_SIZE) + \
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP8_TXFIFO_SIZE))

#if CONFIG_USBDEV_EP0_TXFIFO_SIZE == 0 || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP0_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP1_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP2_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP3_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP4_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP5_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP6_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP7_TXFIFO_SIZE) || \
    !STM32_USB_FIFO_VALID(CONFIG_USBDEV_EP8_TXFIFO_SIZE)
#  error "Invalid STM32N6 USB TX FIFO depth"
#endif

#if STM32_USB_FIFO_TOTAL > STM32_OTG_FIFO_WORDS
#  error "STM32N6 USB FIFO layout exceeds 4 KiB"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_usbdev_s
{
  struct usbdev_s usbdev;
  int initresult;
  uint32_t clocks_added;
  uint32_t pwr_added;
  uint32_t saved_clksel;
  uint32_t saved_hsecfg;
  uint8_t stage;
  bool initializing;
  bool initialized;
  bool resets_owned;
  bool clksel_changed;
  bool hse_restore;
  bool core_access;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_usbdev_s g_otgdev =
{
  .usbdev.speed = USB_SPEED_UNKNOWN,
  .initresult = -ENODEV
};

static const uint16_t g_txfifo_words[STM32_OTG_NENDPOINTS] =
{
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP0_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP1_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP2_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP3_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP4_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP5_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP6_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP7_TXFIFO_SIZE),
  STM32_USB_TX_WORDS(CONFIG_USBDEV_EP8_TXFIFO_SIZE)
};

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  {STM32_TRACEERR_NOTREADY, "USB device operations unavailable"},
  {STM32_TRACEERR_INVALIDPARMS, "Invalid USB class driver"},
  {STM32_TRACEERR_NOTBOUND, "USB class driver is not bound"},
  {STM32_TRACEERR_INITFAILED, "USB hardware initialization failed"},
  {STM32_TRACEERR_CLEANUPFAILED, "USB hardware cleanup failed"},
  {0, NULL}
};

const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  {0, NULL}
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_usb_check_access(void)
{
  if (getipsr() != 0)
    {
      return -EWOULDBLOCK;
    }

  if ((getcontrol() & CONTROL_NPRIV) != 0)
    {
      return -EACCES;
    }

  /* This port uses secure aliases and the secure N6 boot environment.
   * Secure privileged CPU access satisfies either SEC/PRIV setting of
   * RIFSC resources 56/57 and RCC/PWR (RM0486 6.3.2, 13.8, 14.7).
   * USB slave ports have no CID filter; DMA/RIMU is not used. Never alter
   * locked bootloader isolation policy to make this driver accessible.
   */

#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  return -EACCES;
#else
  return OK;
#endif
}

static int stm32_usb_wait(uintptr_t address, uint32_t mask,
                          uint32_t value, unsigned int timeout)
{
  unsigned int elapsed;

  for (elapsed = 0; elapsed <= timeout; elapsed++)
    {
      if ((getreg32(address) & mask) == value)
        {
          return OK;
        }

      if (elapsed < timeout)
        {
          up_udelay(1);
        }
    }

  return -ETIMEDOUT;
}

static int stm32_usb_reset(uint32_t mask, bool asserted)
{
  putreg32(mask, asserted ? STM32_RCC_AHB5RSTSR : STM32_RCC_AHB5RSTCR);
  if ((getreg32(STM32_RCC_AHB5RSTR) & mask) != (asserted ? mask : 0))
    {
      return -EACCES;
    }

  return OK;
}

static bool stm32_usb_sources_ready(void)
{
  uint32_t supply = PWR_SVMCR3_USB33VMEN | PWR_SVMCR3_USB33SV |
                    PWR_SVMCR3_USB33RDY;

  return (getreg32(STM32_PWR_SVMCR3) & supply) == supply &&
         (getreg32(STM32_RCC_CR) & RCC_CR_HSEON) != 0 &&
         (getreg32(STM32_RCC_SR) & RCC_SR_HSERDY) != 0 &&
         (getreg32(STM32_RCC_HSECFGR) &
          (STM32_USB_HSE_CONFIG_MASK | RCC_HSECFGR_HSECSSD)) ==
         RCC_HSECFGR_HSEDIV2SEL;
}

static int stm32_usb_hse(struct stm32_usbdev_s *priv)
{
  uint32_t cfgr;
  uint32_t clocks;
  irqstate_t flags;
  unsigned int pll;
  int ret;

  cfgr = getreg32(STM32_RCC_HSECFGR);
  if ((cfgr & RCC_HSECFGR_HSECSSD) != 0)
    {
      return -EIO;
    }

  clocks = getreg32(STM32_RCC_CR);
  if ((clocks & RCC_CR_HSEON) != 0)
    {
      if ((cfgr & STM32_USB_HSE_CONFIG_MASK) != RCC_HSECFGR_HSEDIV2SEL)
        {
          return -EBUSY;
        }
    }
  else
    {
      cfgr = getreg32(STM32_RCC_CFGR1);
      if ((cfgr & RCC_CFGR1_CPUSWS_MASK) == RCC_CFGR1_CPUSWS_HSE ||
          (cfgr & RCC_CFGR1_SYSSWS_MASK) == RCC_CFGR1_SYSSWS_HSE ||
          (cfgr & RCC_CFGR1_CPUSW_MASK) == RCC_CFGR1_CPUSW_HSE ||
          (cfgr & RCC_CFGR1_SYSSW_MASK) == RCC_CFGR1_SYSSW_HSE)
        {
          return -EBUSY;
        }

      for (pll = 1; pll <= 4; pll++)
        {
          if ((clocks & (RCC_CR_PLL1ON << (pll - 1))) != 0 &&
              (getreg32(STM32_RCC_PLLCFGR1(pll)) &
               RCC_PLL1CFGR1_SEL_MASK) == RCC_PLL1CFGR1_SEL_HSE)
            {
              return -EBUSY;
            }
        }

      if ((getreg32(STM32_RCC_LOCKCFGR0) & RCC_LOCKCFGR0_HSELOCK) != 0)
        {
          return -EACCES;
        }

      ret = stm32_usb_wait(STM32_RCC_SR, RCC_SR_HSERDY, 0,
                           STM32_USB_HSE_WAIT_US);
      if (ret < 0)
        {
          return ret;
        }

      flags = enter_critical_section();
      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSEON) != 0)
        {
          leave_critical_section(flags);
          return -EBUSY;
        }

      priv->saved_hsecfg = getreg32(STM32_RCC_HSECFGR) &
                          STM32_USB_HSE_CONFIG_MASK;
      priv->hse_restore = true;
      modifyreg32(STM32_RCC_HSECFGR, STM32_USB_HSE_CONFIG_MASK,
                  RCC_HSECFGR_HSEDIV2SEL);
      if ((getreg32(STM32_RCC_HSECFGR) & STM32_USB_HSE_CONFIG_MASK) !=
          RCC_HSECFGR_HSEDIV2SEL)
        {
          leave_critical_section(flags);
          return -EACCES;
        }

      putreg32(RCC_CR_HSEON, STM32_RCC_CSR);
      leave_critical_section(flags);
      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSEON) == 0)
        {
          return -EACCES;
        }
    }

  ret = stm32_usb_wait(STM32_RCC_SR, RCC_SR_HSERDY, RCC_SR_HSERDY,
                       STM32_USB_HSE_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(BOARD_USB_HSE_STABILIZATION_US);
  if (!stm32_usb_sources_ready())
    {
      return -EIO;
    }

  /* Once stable, HSE is a shared board clock, not a private USB gate.
   * Retain it across USB teardown rather than disrupting later consumers.
   */

  priv->hse_restore = false;
  return OK;
}

static int stm32_usb_cleanup(struct stm32_usbdev_s *priv)
{
  irqstate_t flags;
  int ret;

  up_disable_irq(STM32_IRQ_OTG);
  if (priv->core_access &&
      (getreg32(STM32_RCC_AHB5ENR) & STM32_USB_CLOCKS) ==
      STM32_USB_CLOCKS &&
      (getreg32(STM32_RCC_AHB5RSTR) & STM32_USB_RESETS) == 0 &&
      (getreg32(STM32_RCC_AHB4ENR) & RCC_AHB4ENR_PWREN) != 0 &&
      stm32_usb_sources_ready())
    {
      putreg32(0, STM32_OTG_GINTMSK);
      modifyreg32(STM32_OTG_GAHBCFG, OTG_GAHBCFG_GINTMSK, 0);
      modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SDIS);
      modifyreg32(STM32_OTG_GCCFG, OTG_GCCFG_VBVALOVAL, 0);
    }

  if (priv->resets_owned)
    {
      ret = stm32_usb_reset(STM32_USB_RESETS, true);
      if (ret < 0)
        {
          return ret;
        }

      priv->core_access = false;
    }

  if (priv->clksel_changed)
    {
      flags = enter_critical_section();
      modifyreg32(STM32_RCC_CCIPR6, STM32_OTG_RCC_CLKSEL_MASK,
                  priv->saved_clksel);
      leave_critical_section(flags);
      if ((getreg32(STM32_RCC_CCIPR6) & STM32_OTG_RCC_CLKSEL_MASK) !=
          priv->saved_clksel)
        {
          return -EACCES;
        }

      priv->clksel_changed = false;
    }

  if (priv->clocks_added != 0)
    {
      putreg32(priv->clocks_added, STM32_RCC_AHB5ENCR);
      if ((getreg32(STM32_RCC_AHB5ENR) & priv->clocks_added) != 0)
        {
          return -EACCES;
        }

      priv->clocks_added = 0;
    }

  if (priv->pwr_added != 0)
    {
      if ((getreg32(STM32_RCC_AHB4ENR) & RCC_AHB4ENR_PWREN) == 0)
        {
          return -ENODEV;
        }

      flags = enter_critical_section();
      if ((getreg32(STM32_RCC_AHB5ENR) & STM32_OTG_RCC_OTHER_EN) == 0)
        {
          modifyreg32(STM32_PWR_SVMCR3, priv->pwr_added, 0);
          if ((getreg32(STM32_PWR_SVMCR3) & priv->pwr_added) != 0)
            {
              leave_critical_section(flags);
              return -EACCES;
            }
        }

      priv->pwr_added = 0;
      leave_critical_section(flags);
    }

  if (priv->hse_restore)
    {
      putreg32(RCC_CR_HSEON, STM32_RCC_CCR);
      ret = stm32_usb_wait(STM32_RCC_SR, RCC_SR_HSERDY, 0,
                           STM32_USB_HSE_WAIT_US);
      if (ret < 0)
        {
          return ret;
        }

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSEON) != 0)
        {
          return -EACCES;
        }

      flags = enter_critical_section();
      modifyreg32(STM32_RCC_HSECFGR, STM32_USB_HSE_CONFIG_MASK,
                  priv->saved_hsecfg);
      leave_critical_section(flags);
      if ((getreg32(STM32_RCC_HSECFGR) & STM32_USB_HSE_CONFIG_MASK) !=
          priv->saved_hsecfg)
        {
          return -EACCES;
        }

      priv->hse_restore = false;
    }

  priv->resets_owned = false;
  return OK;
}

static int stm32_usb_coreconfigure(struct stm32_usbdev_s *priv)
{
  uint32_t start = STM32_USB_RX_WORDS;
  uint32_t value;
  unsigned int ep;
  int ret;

  priv->stage = 7;
  ret = stm32_usb_wait(STM32_OTG_GRSTCTL, OTG_GRSTCTL_AHBIDL,
                       OTG_GRSTCTL_AHBIDL, STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  priv->core_access = true;
  modifyreg32(STM32_OTG_PCGCCTL,
              OTG_PCGCCTL_STPPCLK | OTG_PCGCCTL_GATEHCLK |
              OTG_PCGCCTL_ENL1GTG | OTG_PCGCCTL_SUSP, 0);
  modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SDIS);
  modifyreg32(STM32_OTG_GCCFG, OTG_GCCFG_VBVALOVAL, 0);
  putreg32(OTG_GRSTCTL_CSRST, STM32_OTG_GRSTCTL);
  ret = stm32_usb_wait(STM32_OTG_GRSTCTL, OTG_GRSTCTL_CSRST, 0,
                       STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(3);
  ret = stm32_usb_wait(STM32_OTG_GRSTCTL, OTG_GRSTCTL_AHBIDL,
                       OTG_GRSTCTL_AHBIDL, STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  priv->stage = 8;
  modifyreg32(STM32_OTG_GUSBCFG,
              OTG_GUSBCFG_FHMOD | OTG_GUSBCFG_PHYLPC |
              OTG_GUSBCFG_TRDT_MASK,
              OTG_GUSBCFG_FDMOD | (9u << OTG_GUSBCFG_TRDT_SHIFT));
  up_udelay(STM32_USB_MODE_DELAY_US);
  ret = stm32_usb_wait(STM32_OTG_GINTSTS, OTG_GINT_CMOD, 0,
                       STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  value = OTG_GUSBCFG_FDMOD | (9u << OTG_GUSBCFG_TRDT_SHIFT);
  if ((getreg32(STM32_OTG_GUSBCFG) &
       (OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_FHMOD |
        OTG_GUSBCFG_TRDT_MASK)) != value)
    {
      return -EIO;
    }

  value = OTG_DCFG_PFIVL_80PCT;
#ifdef CONFIG_STM32_N6_OTGDEV_FS
  value |= OTG_DCFG_DSPD_FS;
#else
  value |= OTG_DCFG_DSPD_HS;
#endif
  modifyreg32(STM32_OTG_DCFG,
              OTG_DCFG_DSPD_MASK | OTG_DCFG_DAD_MASK |
              OTG_DCFG_NZLSOHSK | OTG_DCFG_PFIVL_MASK, value);
  if ((getreg32(STM32_OTG_DCFG) &
       (OTG_DCFG_DSPD_MASK | OTG_DCFG_DAD_MASK |
        OTG_DCFG_NZLSOHSK | OTG_DCFG_PFIVL_MASK)) != value)
    {
      return -EIO;
    }

  if ((getreg32(STM32_OTG_GINTSTS) &
       (OTG_GINT_GINAKEFF | OTG_GINT_GONAKEFF)) != 0)
    {
      return -EIO;
    }

  modifyreg32(STM32_OTG_DCTL, OTG_DCTL_RWUSIG | OTG_DCTL_TCTL_MASK,
              OTG_DCTL_SDIS | OTG_DCTL_SGINAK | OTG_DCTL_SGONAK);
  ret = stm32_usb_wait(STM32_OTG_DCTL,
                       OTG_DCTL_GINSTS | OTG_DCTL_GONSTS,
                       OTG_DCTL_GINSTS | OTG_DCTL_GONSTS,
                       STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  priv->stage = 9;
  putreg32(0, STM32_OTG_GAHBCFG);
  putreg32(0, STM32_OTG_GINTMSK);
  putreg32(0, STM32_OTG_DIEPMSK);
  putreg32(0, STM32_OTG_DOEPMSK);
  putreg32(0, STM32_OTG_DAINTMSK);
  putreg32(0, STM32_OTG_DIEPEMPMSK);
  putreg32(0, STM32_OTG_DTHRCTL);
  putreg32(STM32_USB_RX_WORDS, STM32_OTG_GRXFSIZ);
  if (getreg32(STM32_OTG_GRXFSIZ) != STM32_USB_RX_WORDS)
    {
      return -EIO;
    }

  for (ep = 0; ep < STM32_OTG_NENDPOINTS; ep++)
    {
      value = start | ((uint32_t)g_txfifo_words[ep] <<
                       OTG_DIEPTXF_DEPTH_SHIFT);
      putreg32(value, ep == 0 ? STM32_OTG_DIEPTXF0 :
               STM32_OTG_DIEPTXF(ep));
      if (getreg32(ep == 0 ? STM32_OTG_DIEPTXF0 :
                   STM32_OTG_DIEPTXF(ep)) != value)
        {
          return -EIO;
        }

      start += g_txfifo_words[ep];

      /* RCC/core reset already disabled every endpoint, including EP0.
       * Confirm that state instead of trying to disable control OUT EP0,
       * which software is not allowed to disable (RM0486 73.14.54).
       */

      if (((getreg32(STM32_OTG_DIEPCTL(ep)) |
            getreg32(STM32_OTG_DOEPCTL(ep))) & OTG_EPCTL_EPENA) != 0)
        {
          return -EIO;
        }

      putreg32(OTG_DIEPINT_W1C_MASK, STM32_OTG_DIEPINT(ep));
      putreg32(OTG_DOEPINT_W1C_MASK, STM32_OTG_DOEPINT(ep));
    }

  ret = stm32_usb_wait(STM32_OTG_GRSTCTL, OTG_GRSTCTL_AHBIDL,
                       OTG_GRSTCTL_AHBIDL, STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXFNUM_ALL,
           STM32_OTG_GRSTCTL);
  ret = stm32_usb_wait(STM32_OTG_GRSTCTL, OTG_GRSTCTL_TXFFLSH, 0,
                       STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(OTG_GRSTCTL_RXFFLSH, STM32_OTG_GRSTCTL);
  ret = stm32_usb_wait(STM32_OTG_GRSTCTL, OTG_GRSTCTL_RXFFLSH, 0,
                       STM32_USB_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(1);
  putreg32(OTG_GOTGINT_W1C_MASK, STM32_OTG_GOTGINT);
  putreg32(OTG_GINTSTS_W1C_MASK, STM32_OTG_GINTSTS);
  if (!stm32_usb_sources_ready() ||
      (getreg32(STM32_OTG_DCTL) & OTG_DCTL_SDIS) == 0 ||
      (getreg32(STM32_OTG_GCCFG) & OTG_GCCFG_VBVALOVAL) != 0 ||
      (getreg32(STM32_OTG_GAHBCFG) | getreg32(STM32_OTG_GINTMSK) |
       getreg32(STM32_OTG_DIEPMSK) | getreg32(STM32_OTG_DOEPMSK) |
       getreg32(STM32_OTG_DAINTMSK) | getreg32(STM32_OTG_DIEPEMPMSK)) != 0)
    {
      return -EIO;
    }

  return OK;
}

static int stm32_usb_hwinitialize(struct stm32_usbdev_s *priv)
{
  irqstate_t flags;
  uint32_t before;
  uint32_t after;
  int ret;

  priv->stage = 1;
  up_disable_irq(STM32_IRQ_OTG);
  if ((getreg32(STM32_RCC_AHB4ENR) & RCC_AHB4ENR_PWREN) == 0)
    {
      return -ENODEV;
    }

  priv->resets_owned = true;
  ret = stm32_usb_reset(STM32_USB_RESETS, true);
  if (ret < 0)
    {
      return ret;
    }

  priv->stage = 2;
  flags = enter_critical_section();
  before = getreg32(STM32_PWR_SVMCR3);
  modifyreg32(STM32_PWR_SVMCR3, 0, PWR_SVMCR3_USB33VMEN);
  after = getreg32(STM32_PWR_SVMCR3);
  priv->pwr_added |= (after & ~before) & PWR_SVMCR3_USB33VMEN;
  leave_critical_section(flags);
  if ((after & PWR_SVMCR3_USB33VMEN) == 0)
    {
      return -EACCES;
    }

  ret = stm32_usb_wait(STM32_PWR_SVMCR3, PWR_SVMCR3_USB33RDY,
                       PWR_SVMCR3_USB33RDY, STM32_USB_SUPPLY_WAIT_US);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  before = getreg32(STM32_PWR_SVMCR3);
  if ((before & PWR_SVMCR3_USB33RDY) == 0)
    {
      leave_critical_section(flags);
      return -EIO;
    }

  modifyreg32(STM32_PWR_SVMCR3, 0, PWR_SVMCR3_USB33SV);
  after = getreg32(STM32_PWR_SVMCR3);
  priv->pwr_added |= (after & ~before) & PWR_SVMCR3_USB33SV;
  leave_critical_section(flags);
  if ((after & PWR_SVMCR3_USB33SV) == 0)
    {
      return -EACCES;
    }

  priv->stage = 3;
  ret = stm32_usb_hse(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->stage = 4;
  flags = enter_critical_section();
  priv->saved_clksel = getreg32(STM32_RCC_CCIPR6) &
                       STM32_OTG_RCC_CLKSEL_MASK;
  priv->clksel_changed = true;
  modifyreg32(STM32_RCC_CCIPR6, STM32_OTG_RCC_CLKSEL_MASK,
              STM32_OTG_RCC_CLKSEL);
  leave_critical_section(flags);
  if ((getreg32(STM32_RCC_CCIPR6) & STM32_OTG_RCC_CLKSEL_MASK) !=
      STM32_OTG_RCC_CLKSEL)
    {
      return -EACCES;
    }

  before = getreg32(STM32_RCC_AHB5ENR);
  putreg32(STM32_USB_CLOCKS, STM32_RCC_AHB5ENSR);
  after = getreg32(STM32_RCC_AHB5ENR);
  priv->clocks_added = (after & ~before) & STM32_USB_CLOCKS;
  if ((after & STM32_USB_CLOCKS) != STM32_USB_CLOCKS)
    {
      return -EACCES;
    }

  priv->stage = 5;
  ret = stm32_usb_reset(STM32_OTG_RCC_PHYCTL_RST, false);
  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(STM32_USBPHYC_CR, USBPHYC_CR_FSEL_MASK,
              USBPHYC_CR_FSEL_24MHZ);
  if ((getreg32(STM32_USBPHYC_CR) & USBPHYC_CR_FSEL_MASK) !=
      USBPHYC_CR_FSEL_24MHZ)
    {
      return -EACCES;
    }

  priv->stage = 6;
  ret = stm32_usb_reset(STM32_OTG_RCC_PHY_RST, false);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(STM32_USB_PHY_DELAY_US);
  if (!stm32_usb_sources_ready())
    {
      return -EIO;
    }

  ret = stm32_usb_reset(STM32_OTG_RCC_RST, false);
  if (ret < 0)
    {
      return ret;
    }

  return stm32_usb_coreconfigure(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_usbinitialize(void)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  irqstate_t flags;
  int cleanup;
  int ret;

  usbtrace(TRACE_DEVINIT, STM32_OTG_PORT);
  ret = stm32_usb_check_access();
  if (ret < 0)
    {
      if (!priv->initialized && !priv->initializing)
        {
          priv->initresult = ret;
        }

      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INITFAILED), -ret);
      uerr("ERROR: USB%d initialization context denied: %d\n",
           STM32_OTG_PORT, ret);
      return;
    }

  flags = enter_critical_section();
  if (priv->initializing || priv->initialized)
    {
      ret = priv->initializing ? -EBUSY : OK;
      leave_critical_section(flags);
      if (ret < 0)
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INITFAILED), -ret);
          uerr("ERROR: USB initialization already in progress\n");
        }

      return;
    }

  priv->initializing = true;
  priv->initresult = -EBUSY;
  leave_critical_section(flags);

  ret = stm32_usb_cleanup(priv);
  if (ret == OK)
    {
      ret = stm32_usb_hwinitialize(priv);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INITFAILED), -ret);
      uerr("ERROR: USB%d initialization stage %u failed: %d\n",
           STM32_OTG_PORT, priv->stage, ret);
      cleanup = stm32_usb_cleanup(priv);
      if (cleanup < 0)
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), -cleanup);
          uerr("ERROR: USB%d cleanup failed: %d\n",
               STM32_OTG_PORT, cleanup);
        }
    }

  flags = enter_critical_section();
  priv->initresult = ret;
  priv->initialized = ret == OK;
  priv->initializing = false;
  leave_critical_section(flags);
}

void arm_usbuninitialize(void)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  irqstate_t flags;
  int ret;

  usbtrace(TRACE_DEVUNINIT, STM32_OTG_PORT);
  ret = stm32_usb_check_access();
  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), -ret);
      uerr("ERROR: USB%d uninitialization context denied: %d\n",
           STM32_OTG_PORT, ret);
      return;
    }

  flags = enter_critical_section();
  if (priv->initializing)
    {
      leave_critical_section(flags);
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), EBUSY);
      uerr("ERROR: USB initialization already in progress\n");
      return;
    }

  priv->initializing = true;
  priv->initialized = false;
  priv->initresult = -EBUSY;
  leave_critical_section(flags);
  ret = stm32_usb_cleanup(priv);
  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), -ret);
      uerr("ERROR: USB%d cleanup failed: %d\n", STM32_OTG_PORT, ret);
    }

  flags = enter_critical_section();
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  priv->initresult = ret < 0 ? ret : -ENODEV;
  priv->initializing = false;
  leave_critical_section(flags);
}

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, STM32_OTG_PORT);
  if (driver == NULL || driver->ops == NULL)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), EINVAL);
      uerr("ERROR: Invalid USB class driver\n");
      return -EINVAL;
    }

  ret = g_otgdev.initresult < 0 ? g_otgdev.initresult : -ENOSYS;
  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOTREADY), -ret);
  uerr("ERROR: USB%d device operations unavailable: %d\n",
       STM32_OTG_PORT, ret);
  return ret;
}

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  usbtrace(TRACE_DEVUNREGISTER, STM32_OTG_PORT);
  if (driver == NULL)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), EINVAL);
      uerr("ERROR: Invalid USB class driver\n");
      return -EINVAL;
    }

  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOTBOUND), ENODEV);
  uerr("ERROR: No USB class driver is bound\n");
  return -ENODEV;
}

#endif /* CONFIG_STM32_N6_OTGDEV */
