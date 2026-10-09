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
#include <string.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_pwr.h"
#include "stm32_otg.h"

void stm32_usbsuspend(struct usbdev_s *dev, bool resume) weak_function;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_TRACEERR_NOTREADY         1
#define STM32_TRACEERR_INVALIDPARMS     2
#define STM32_TRACEERR_NOTBOUND         3
#define STM32_TRACEERR_INITFAILED       4
#define STM32_TRACEERR_CLEANUPFAILED     5
#define STM32_TRACEERR_TRANSFER          6

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

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#  define CONFIG_USBDEV_SETUP_MAXDATASIZE 4096
#endif

#if CONFIG_USBDEV_SETUP_MAXDATASIZE < 128 || \
    CONFIG_USBDEV_SETUP_MAXDATASIZE > 65535
#  error "Invalid STM32N6 control OUT buffer size"
#endif

#define STM32_USB_STOP_TICKS            (MSEC2TICK(20) + 1)
#define STM32_USB_IRQ_MASK              (OTG_GINT_RXFLVL | \
  OTG_GINT_IEPINT | OTG_GINT_OEPINT | OTG_GINT_USBRST | OTG_GINT_ENUMDNE | \
  OTG_GINT_USBSUSP | OTG_GINT_WKUPINT | OTG_GINT_GONAKEFF)

enum stm32_usbstate_e
{
  USBSTATE_OFF,
  USBSTATE_INITIALIZING,
  USBSTATE_READY,
  USBSTATE_BINDING,
  USBSTATE_BOUND,
  USBSTATE_RESETTING,
  USBSTATE_ENUMERATED,
  USBSTATE_SUSPENDED,
  USBSTATE_DETACHING,
  USBSTATE_UNBINDING,
  USBSTATE_QUIESCED,
  USBSTATE_UNINITIALIZING,
  USBSTATE_FAULT
};

enum stm32_usbresource_e
{
  USBRES_RESETS = 1 << 0,
  USBRES_CLKSEL = 1 << 1,
  USBRES_HSE    = 1 << 2,
  USBRES_CORE   = 1 << 3,
  USBRES_IRQ    = 1 << 4
};

enum stm32_linkstate_e
{
  LINK_ABSENT,
  LINK_PRESENT,
  LINK_WAITING,
  LINK_PENDING,
  LINK_CONNECTED
};

enum stm32_epstate_e
{
  EPSTATE_FREE,
  EPSTATE_ALLOCATED,
  EPSTATE_IDLE,
  EPSTATE_TX_WAIT,
  EPSTATE_TX_ACTIVE,
  EPSTATE_RX_ACTIVE,
  EPSTATE_HALTED,
  EPSTATE_NAKING,
  EPSTATE_DISABLING,
  EPSTATE_FLUSHING
};

enum stm32_epaction_e
{
  EPACTION_CANCEL,
  EPACTION_DISABLE,
  EPACTION_HALT,
  EPACTION_SETUP,
  EPACTION_RECONFIGURE,
  EPACTION_FREE
};

enum stm32_ctrlstate_e
{
  CTRL_SETUP,
  CTRL_ABORT,
  CTRL_RECEIVE,
  CTRL_READY,
  CTRL_DISPATCH,
  CTRL_DATA_IN,
  CTRL_STATUS_IN,
  CTRL_STATUS_OUT,
  CTRL_ADDRESS,
  CTRL_HALT,
  CTRL_RESUME,
  CTRL_STALLED
};

enum stm32_workstate_e
{
  WORK_IDLE,
  WORK_PENDING,
  WORK_RUNNING
};

enum stm32_flushstate_e
{
  FLUSH_IDLE,
  FLUSH_ENDPOINT,
  FLUSH_RESET_TX,
  FLUSH_RESET_RX
};

enum stm32_reqstate_e
{
  REQ_IDLE,
  REQ_QUEUED,
  REQ_DATA,
  REQ_ZLP,
  REQ_COMPLETE
};

struct stm32_req_s
{
  struct usbdev_req_s req;
  struct stm32_req_s *next;
  struct stm32_ep_s *owner;
  enum stm32_reqstate_e state;
  size_t limit;
};

struct stm32_ep_s
{
  struct usbdev_ep_s ep;
  struct stm32_req_s *head;
  struct stm32_req_s *tail;
  struct stm32_req_s *lastcancel;
  enum stm32_epstate_e state;
  enum stm32_epaction_e action;
  clock_t stopstart;
  size_t packet;
  size_t received;
  int16_t result;
  uint8_t type;
};

struct stm32_usbdev_s
{
  struct usbdev_s usbdev;
  int initresult;
  uint32_t clocks_added;
  uint32_t pwr_added;
  uint32_t saved_clksel;
  uint32_t saved_hsecfg;
  uint8_t stage;
  enum stm32_usbstate_e state;
  enum stm32_linkstate_e link;
  enum stm32_ctrlstate_e control;
  enum stm32_workstate_e work;
  enum stm32_flushstate_e flush;
  unsigned int resources;
  struct usbdevclass_driver_s *driver;
  struct stm32_ep_s epin[STM32_OTG_NENDPOINTS];
  struct stm32_ep_s epout[STM32_OTG_NENDPOINTS];
  struct stm32_ep_s *flushing;
  struct wdog_s watchdog;
  sem_t quiesced;
  clock_t stopstart;
  struct usb_ctrlreq_s ctrl;
  struct stm32_req_s response;
  uint8_t setupdata[CONFIG_USBDEV_SETUP_MAXDATASIZE];
  uint8_t reply[2];
  size_t outlen;
  uint8_t configuration;
  uint8_t powerstatus;
};

static int stm32_usb_interrupt(int irq, void *context, void *arg);
static void stm32_usb_service(struct stm32_usbdev_s *priv);
static void stm32_usb_start(struct stm32_ep_s *ep);
static void stm32_usb_dispatch(struct stm32_usbdev_s *priv);
static void stm32_usb_receive(struct stm32_usbdev_s *priv);
static void stm32_usb_connect(struct stm32_usbdev_s *priv);
static void stm32_usb_ack(struct stm32_ep_s *ep);
static void stm32_usb_outack(struct stm32_ep_s *ep, uint32_t intr);
static void stm32_usb_software(struct stm32_usbdev_s *priv);
static void stm32_usb_abort(struct stm32_usbdev_s *priv, int result);
static void stm32_usb_resetstart(struct stm32_usbdev_s *priv,
                                 enum stm32_usbstate_e state);
static int stm32_ep_stall(struct usbdev_ep_s *ep, bool resume);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_usbdev_s g_otgdev =
{
  .usbdev.speed = USB_SPEED_UNKNOWN,
  .initresult = -ENODEV,
  .quiesced = SEM_INITIALIZER(0)
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
  {STM32_TRACEERR_TRANSFER, "USB transfer or endpoint quiescence failed"},
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
      priv->resources |= USBRES_HSE;
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

  priv->resources &= ~USBRES_HSE;
  return OK;
}

static int stm32_usb_cleanup(struct stm32_usbdev_s *priv)
{
  irqstate_t flags;
  int ret;

  up_disable_irq(STM32_IRQ_OTG);
  if ((priv->resources & USBRES_CORE) != 0 &&
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

  if ((priv->resources & USBRES_RESETS) != 0)
    {
      ret = stm32_usb_reset(STM32_USB_RESETS, true);
      if (ret < 0)
        {
          return ret;
        }

      priv->resources &= ~USBRES_CORE;
    }

  if ((priv->resources & USBRES_CLKSEL) != 0)
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

      priv->resources &= ~USBRES_CLKSEL;
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

  if ((priv->resources & USBRES_HSE) != 0)
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

      priv->resources &= ~USBRES_HSE;
    }

  priv->resources &= ~USBRES_RESETS;
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

  priv->resources |= USBRES_CORE;
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

  priv->resources |= USBRES_RESETS;
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
  priv->resources |= USBRES_CLKSEL;
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
 * FIFO-mode transfers
 ****************************************************************************/

static int stm32_usb_error(int error)
{
  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_TRANSFER), -error);
  uerr("ERROR: USB%d operation failed: %d\n", STM32_OTG_PORT, error);
  return error;
}

static unsigned int stm32_usb_epno(struct stm32_ep_s *ep)
{
  return ep->ep.eplog & USB_EPNO_MASK;
}

static bool stm32_usb_isin(struct stm32_ep_s *ep)
{
  return (ep->ep.eplog & USB_DIR_IN) != 0;
}

static uintptr_t stm32_usb_epctl(struct stm32_ep_s *ep)
{
  unsigned int number = stm32_usb_epno(ep);

  return stm32_usb_isin(ep) ? STM32_OTG_DIEPCTL(number) :
                             STM32_OTG_DOEPCTL(number);
}

static struct stm32_ep_s *stm32_usb_endpoint(struct usbdev_ep_s *ep)
{
  unsigned int n;

  for (n = 0; n < STM32_OTG_NENDPOINTS; n++)
    {
      if (ep == &g_otgdev.epin[n].ep)
        {
          return &g_otgdev.epin[n];
        }

      if (ep == &g_otgdev.epout[n].ep)
        {
          return &g_otgdev.epout[n];
        }
    }

  return NULL;
}

static struct stm32_ep_s *stm32_usb_address(uint16_t address)
{
  unsigned int n = address & USB_EPNO_MASK;

  if ((address & ~(USB_DIR_IN | USB_EPNO_MASK)) != 0 ||
      n >= STM32_OTG_NENDPOINTS)
    {
      return NULL;
    }

  return (address & USB_DIR_IN) != 0 ? &g_otgdev.epin[n] :
                                     &g_otgdev.epout[n];
}

static bool stm32_usb_stopping(struct stm32_ep_s *ep)
{
  return ep->state == EPSTATE_NAKING ||
         ep->state == EPSTATE_DISABLING ||
         ep->state == EPSTATE_FLUSHING;
}

static bool stm32_usb_busy(struct stm32_usbdev_s *priv)
{
  unsigned int n;

  for (n = 0; n < STM32_OTG_NENDPOINTS; n++)
    {
      if (stm32_usb_stopping(&priv->epin[n]) ||
          stm32_usb_stopping(&priv->epout[n]))
        {
          return true;
        }
    }

  return false;
}

static void stm32_usb_readfifo(uint8_t *buffer, size_t capacity,
                                size_t count)
{
  size_t offset;
  unsigned int byte;
  uint32_t word;

  for (offset = 0; offset < count; offset += 4)
    {
      word = getreg32(STM32_OTG_DFIFO(0));
      for (byte = 0; byte < 4 && offset + byte < count; byte++)
        {
          if (offset + byte < capacity)
            {
              buffer[offset + byte] = word >> (8 * byte);
            }
        }
    }
}

static void stm32_usb_writefifo(unsigned int number,
                                const uint8_t *buffer, size_t count)
{
  size_t offset;
  unsigned int byte;
  uint32_t word;

  for (offset = 0; offset < count; offset += 4)
    {
      word = 0;
      for (byte = 0; byte < 4 && offset + byte < count; byte++)
        {
          word |= (uint32_t)buffer[offset + byte] << (8 * byte);
        }

      putreg32(word, STM32_OTG_DFIFO(number));
    }
}

static void stm32_usb_callback(struct stm32_ep_s *ep,
                               struct stm32_req_s *req, int result)
{
  req->owner = NULL;
  req->state = REQ_IDLE;
  req->next = NULL;
  req->req.result = result;
  usbtrace(TRACE_COMPLETE(stm32_usb_epno(ep)), req->req.xfrd);
  req->req.callback(&ep->ep, &req->req);
}

static void stm32_usb_complete(struct stm32_ep_s *ep, int result)
{
  struct stm32_req_s *req = ep->head;

  if (req != NULL)
    {
      ep->head = req->next;
      if (ep->head == NULL)
        {
          ep->tail = NULL;
        }

      stm32_usb_callback(ep, req, result);
    }
}

static void stm32_usb_cancelprefix(struct stm32_ep_s *ep,
                                   struct stm32_req_s *last, int result)
{
  struct stm32_req_s *list;
  struct stm32_req_s *next;

  if (last == NULL)
    {
      return;
    }

  list = ep->head;
  ep->head = last->next;
  last->next = NULL;
  if (ep->head == NULL)
    {
      ep->tail = NULL;
    }

  while (list != NULL)
    {
      next = list->next;
      stm32_usb_callback(ep, list, result);
      list = next;
    }
}

static void stm32_usb_armout0(void)
{
  putreg32(64 | OTG_DOEPTSIZ0_PKTCNT |
           (3u << OTG_DOEPTSIZ0_STUPCNT_SHIFT), STM32_OTG_DOEPTSIZ(0));
  g_otgdev.epout[0].received = 0;
  modifyreg32(STM32_OTG_DOEPCTL(0), 0,
              OTG_EPCTL_EPENA | OTG_EPCTL_CNAK);
}

static void stm32_usb_ep0init(struct stm32_usbdev_s *priv)
{
  priv->control = CTRL_SETUP;
  priv->configuration = 0;
  priv->outlen = 0;
  priv->epin[0].state = EPSTATE_IDLE;
  priv->epout[0].state = EPSTATE_IDLE;
  priv->epin[0].ep.maxpacket = 64;
  priv->epout[0].ep.maxpacket = 64;
  putreg32(64 | OTG_EPCTL_USBAEP | OTG_EPCTL_SNAK,
           STM32_OTG_DIEPCTL(0));
  putreg32(OTG_EPCTL_USBAEP | OTG_EPCTL_SNAK,
           STM32_OTG_DOEPCTL(0));
  putreg32(OTG_DAINT_IN(0) | OTG_DAINT_OUT(0), STM32_OTG_DAINTMSK);
  putreg32(OTG_DIEPINT_XFRC | OTG_DIEPINT_EPDISD |
           OTG_DIEPINT_TOC | OTG_DIEPINT_TXFIFOUDRN,
           STM32_OTG_DIEPMSK);
  putreg32(OTG_DOEPINT_XFRC | OTG_DOEPINT_EPDISD |
           OTG_DOEPINT_STUP | OTG_DOEPINT_B2BSTUP |
           OTG_DOEPINT_OUTPKTERR | OTG_DOEPINT_BERR,
           STM32_OTG_DOEPMSK);
  modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_CGINAK | OTG_DCTL_CGONAK);
  stm32_usb_armout0();
}

static void stm32_usb_start(struct stm32_ep_s *ep)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  struct stm32_req_s *req = ep->head;
  unsigned int number = stm32_usb_epno(ep);
  size_t packet;

  if (req == NULL || (ep->state != EPSTATE_IDLE &&
                     ep->state != EPSTATE_TX_WAIT) ||
      priv->link != LINK_CONNECTED ||
      priv->state != USBSTATE_ENUMERATED)
    {
      return;
    }

  if (stm32_usb_isin(ep))
    {
      if (number == 0 &&
          (priv->control == CTRL_STATUS_IN ||
           priv->control == CTRL_ADDRESS) && stm32_usb_busy(priv))
        {
          ep->state = EPSTATE_TX_WAIT;
          return;
        }

      packet = req->limit - req->req.xfrd;
      if (packet > ep->ep.maxpacket)
        {
          packet = ep->ep.maxpacket;
        }

      ep->packet = packet;
      if (packet != 0 &&
          (getreg32(STM32_OTG_DTXFSTS(number)) &
           OTG_DTXFSTS_INEPTFSAV_MASK) < (packet + 3) / 4)
        {
          ep->state = EPSTATE_TX_WAIT;
          modifyreg32(STM32_OTG_DIEPEMPMSK, 0, OTG_DAINT_IN(number));
          return;
        }

      modifyreg32(STM32_OTG_DIEPEMPMSK, OTG_DAINT_IN(number), 0);
      putreg32(packet | (1u << OTG_EPTSIZ_PKTCNT_SHIFT),
               STM32_OTG_DIEPTSIZ(number));
      ep->state = EPSTATE_TX_ACTIVE;
      req->state = packet == 0 ? REQ_ZLP : REQ_DATA;
      modifyreg32(STM32_OTG_DIEPCTL(number), 0,
                  OTG_EPCTL_CNAK | OTG_EPCTL_EPENA);
      if (packet != 0)
        {
          stm32_usb_writefifo(number, req->req.buf + req->req.xfrd,
                              packet);
        }
    }
  else
    {
      ep->received = 0;
      ep->state = EPSTATE_RX_ACTIVE;
      req->state = REQ_DATA;
      putreg32(ep->ep.maxpacket | (1u << OTG_EPTSIZ_PKTCNT_SHIFT),
               STM32_OTG_DOEPTSIZ(number));
      modifyreg32(STM32_OTG_DOEPCTL(number), 0,
                  OTG_EPCTL_CNAK | OTG_EPCTL_EPENA);
    }
}

static void stm32_usb_configure(struct stm32_ep_s *ep)
{
  unsigned int number = stm32_usb_epno(ep);
  uint32_t ctl = ep->ep.maxpacket | OTG_EPCTL_USBAEP |
                 OTG_EPCTL_SD0PID | OTG_EPCTL_SNAK |
                 ((uint32_t)ep->type << OTG_EPCTL_EPTYP_SHIFT);

  if (stm32_usb_isin(ep))
    {
      ctl |= number << OTG_DIEPCTL_TXFNUM_SHIFT;
      modifyreg32(STM32_OTG_DAINTMSK, 0, OTG_DAINT_IN(number));
    }
  else
    {
      modifyreg32(STM32_OTG_DAINTMSK, 0, OTG_DAINT_OUT(number));
    }

  putreg32(ctl, stm32_usb_epctl(ep));
  ep->state = EPSTATE_IDLE;
}

static void stm32_usb_finishstop(struct stm32_ep_s *ep)
{
  struct stm32_req_s *last = ep->lastcancel;
  int result = ep->result;

  ep->lastcancel = NULL;
  switch (ep->action)
    {
      case EPACTION_DISABLE:
      case EPACTION_FREE:
        modifyreg32(stm32_usb_epctl(ep), OTG_EPCTL_USBAEP, 0);
        ep->state = ep->action == EPACTION_FREE ? EPSTATE_FREE :
                                                EPSTATE_ALLOCATED;
        break;

      case EPACTION_RECONFIGURE:
        stm32_usb_configure(ep);
        break;

      case EPACTION_HALT:
        modifyreg32(stm32_usb_epctl(ep), 0, OTG_EPCTL_STALL);
        ep->state = EPSTATE_HALTED;
        break;

      default:
        ep->state = EPSTATE_IDLE;
        break;
    }

  stm32_usb_cancelprefix(ep, last, result);
  if (ep->state == EPSTATE_HALTED && ep->head != NULL &&
      ep->head->state == REQ_COMPLETE)
    {
      stm32_usb_complete(ep, ep->head->req.result == -EINPROGRESS ?
                            OK : ep->head->req.result);
    }

  stm32_usb_start(ep);
}

static void stm32_usb_stop(struct stm32_ep_s *ep,
                           enum stm32_epaction_e action, int result,
                           struct stm32_req_s *last)
{
  unsigned int number = stm32_usb_epno(ep);

  ep->action = action;
  ep->result = result;
  ep->lastcancel = last;
  if (stm32_usb_stopping(ep))
    {
      return;
    }

  if (ep->state != EPSTATE_TX_ACTIVE)
    {
      ep->packet = 0;
    }

  ep->state = EPSTATE_NAKING;
  ep->stopstart = clock_systime_ticks();
  if (ep->head == NULL &&
      (getreg32(stm32_usb_epctl(ep)) & OTG_EPCTL_EPENA) == 0)
    {
      stm32_usb_finishstop(ep);
      return;
    }

  if (stm32_usb_isin(ep))
    {
      modifyreg32(STM32_OTG_DIEPEMPMSK, OTG_DAINT_IN(number), 0);
      modifyreg32(stm32_usb_epctl(ep), 0, OTG_EPCTL_SNAK);
    }
  else if ((getreg32(stm32_usb_epctl(ep)) & OTG_EPCTL_EPENA) != 0 &&
           (getreg32(STM32_OTG_DCTL) & OTG_DCTL_GONSTS) == 0 &&
           (getreg32(STM32_OTG_GINTSTS) & OTG_GINT_GONAKEFF) == 0)
    {
      modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SGONAK);
    }
}

static void stm32_usb_abort(struct stm32_usbdev_s *priv, int result)
{
  unsigned int number;
  unsigned int direction;
  struct stm32_ep_s *ep;
  struct stm32_req_s *last;

  priv->control = CTRL_SETUP;
  for (direction = 0; direction < 2; direction++)
    {
      for (number = 0; number < STM32_OTG_NENDPOINTS; number++)
        {
          ep = direction == 0 ? &priv->epout[number] :
                                &priv->epin[number];
          last = ep->tail;
          ep->state = number == 0 ? EPSTATE_IDLE :
                      ep->state == EPSTATE_FREE ? EPSTATE_FREE :
                                                 EPSTATE_ALLOCATED;
          ep->lastcancel = NULL;
          stm32_usb_cancelprefix(ep, last, result);
        }
    }
}

static void stm32_usb_fault(struct stm32_usbdev_s *priv, int result)
{
  enum stm32_usbstate_e old = priv->state;

  stm32_usb_error(result);
  priv->state = USBSTATE_FAULT;
  priv->initresult = result;
  up_disable_irq(STM32_IRQ_OTG);
  wd_cancel(&priv->watchdog);
  priv->work = WORK_IDLE;
  if ((priv->resources & USBRES_CORE) != 0)
    {
      modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SDIS);
      modifyreg32(STM32_OTG_GCCFG, OTG_GCCFG_VBVALOVAL, 0);
      putreg32(0, STM32_OTG_GINTMSK);
      putreg32(0, STM32_OTG_GAHBCFG);
    }

  if (stm32_usb_reset(STM32_USB_RESETS, true) < 0)
    {
      stm32_usb_error(-EACCES);
    }
  else
    {
      priv->resources &= ~USBRES_CORE;
    }

  priv->flush = FLUSH_IDLE;
  priv->flushing = NULL;
  if (priv->link == LINK_CONNECTED)
    {
      priv->link = LINK_WAITING;
    }

  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  stm32_usb_abort(priv, result);
  if (old == USBSTATE_UNBINDING)
    {
      nxsem_post(&priv->quiesced);
    }
  else if (priv->driver != NULL)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }
}

static void stm32_usb_timeout(wdparm_t arg)
{
  struct stm32_usbdev_s *priv = (struct stm32_usbdev_s *)arg;
  irqstate_t flags = enter_critical_section();

  priv->work = WORK_PENDING;
  stm32_usb_service(priv);
  leave_critical_section(flags);
}

static void stm32_usb_service(struct stm32_usbdev_s *priv)
{
  struct stm32_ep_s *ep;
  unsigned int direction;
  unsigned int number;
  uint32_t ctl;
  uint32_t intr;
  int ret;

  if (priv->work == WORK_RUNNING || priv->state == USBSTATE_FAULT)
    {
      return;
    }

  if (priv->work == WORK_IDLE)
    {
      priv->stopstart = clock_systime_ticks();
    }

  priv->work = WORK_RUNNING;
  stm32_usb_receive(priv);
  if (priv->flush == FLUSH_ENDPOINT &&
      (getreg32(STM32_OTG_GRSTCTL) & OTG_GRSTCTL_TXFFLSH) == 0)
    {
      ep = priv->flushing;
      priv->flushing = NULL;
      priv->flush = FLUSH_IDLE;
      stm32_usb_finishstop(ep);
    }

  for (direction = 0; direction < 2; direction++)
    {
      for (number = 0; number < STM32_OTG_NENDPOINTS; number++)
        {
          ep = direction == 0 ? &priv->epout[number] :
                                &priv->epin[number];
          if (direction == 1 && stm32_usb_stopping(ep))
            {
              intr = getreg32(STM32_OTG_DIEPINT(number));
              if ((intr & OTG_DIEPINT_XFRC) != 0)
                {
                  putreg32(OTG_DIEPINT_XFRC, STM32_OTG_DIEPINT(number));
                  stm32_usb_ack(ep);
                }
            }

          if (ep->state == EPSTATE_NAKING)
            {
              ctl = getreg32(stm32_usb_epctl(ep));
              intr = getreg32(direction == 0 ? STM32_OTG_DOEPINT(number) :
                                               STM32_OTG_DIEPINT(number));
              if ((ctl & OTG_EPCTL_EPENA) == 0)
                {
                  if (direction == 0 &&
                      (getreg32(STM32_OTG_GINTSTS) &
                       OTG_GINT_RXFLVL) != 0)
                    {
                      if ((clock_t)(clock_systime_ticks() -
                          ep->stopstart) >= STM32_USB_STOP_TICKS)
                        {
                          stm32_usb_fault(priv, -ETIMEDOUT);
                          return;
                        }

                      continue;
                    }

                  ep->state = direction == 0 ? EPSTATE_IDLE :
                                               EPSTATE_FLUSHING;
                  if (direction == 0)
                    {
                      stm32_usb_outack(ep, intr);
                      putreg32(OTG_DOEPINT_XFRC,
                               STM32_OTG_DOEPINT(number));
                      stm32_usb_finishstop(ep);
                    }
                }
              else if ((direction == 1 &&
                        (intr & OTG_DIEPINT_INEPNE) != 0) ||
                       (direction == 0 &&
                        (getreg32(STM32_OTG_GINTSTS) &
                         OTG_GINT_GONAKEFF) != 0))
                {
                  ep->state = EPSTATE_DISABLING;
                  ep->stopstart = clock_systime_ticks();
                  modifyreg32(stm32_usb_epctl(ep), 0,
                              OTG_EPCTL_EPDIS | OTG_EPCTL_SNAK |
                              (ep->action == EPACTION_HALT ?
                               OTG_EPCTL_STALL : 0));
                }
            }

          if (ep->state == EPSTATE_DISABLING)
            {
              intr = getreg32(direction == 0 ? STM32_OTG_DOEPINT(number) :
                                               STM32_OTG_DIEPINT(number));
              if ((intr & OTG_DIEPINT_EPDISD) != 0)
                {
                  putreg32(OTG_DIEPINT_EPDISD,
                           direction == 0 ? STM32_OTG_DOEPINT(number) :
                                            STM32_OTG_DIEPINT(number));
                  ep->state = direction == 0 ? EPSTATE_IDLE :
                                               EPSTATE_FLUSHING;
                  ep->stopstart = clock_systime_ticks();
                  if (direction == 0)
                    {
                      stm32_usb_outack(ep, intr);
                      putreg32(OTG_DOEPINT_XFRC,
                               STM32_OTG_DOEPINT(number));
                      stm32_usb_finishstop(ep);
                    }
                }
            }

          if (ep->state == EPSTATE_FLUSHING &&
              priv->flush == FLUSH_IDLE &&
              (getreg32(STM32_OTG_GRSTCTL) &
               (OTG_GRSTCTL_AHBIDL | OTG_GRSTCTL_TXFFLSH |
                OTG_GRSTCTL_RXFFLSH)) == OTG_GRSTCTL_AHBIDL)
            {
              priv->flush = FLUSH_ENDPOINT;
              priv->flushing = ep;
              ep->stopstart = clock_systime_ticks();
              putreg32(OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXFNUM(number),
                       STM32_OTG_GRSTCTL);
              if ((getreg32(STM32_OTG_GRSTCTL) & OTG_GRSTCTL_TXFFLSH) == 0)
                {
                  priv->flush = FLUSH_IDLE;
                  priv->flushing = NULL;
                  stm32_usb_finishstop(ep);
                }
            }

          if (stm32_usb_stopping(ep) &&
              (clock_t)(clock_systime_ticks() - ep->stopstart) >=
              STM32_USB_STOP_TICKS)
            {
              stm32_usb_fault(priv, -ETIMEDOUT);
              return;
            }
        }
    }

  /* Global OUT NAK must remain asserted until every OUT disable completes. */

  for (number = 1; number < STM32_OTG_NENDPOINTS; number++)
    {
      if (stm32_usb_stopping(&priv->epout[number]))
        {
          break;
        }
    }

  if (number == STM32_OTG_NENDPOINTS)
    {
      modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_CGONAK);
    }

  if (!stm32_usb_busy(priv) && priv->flush == FLUSH_IDLE)
    {
      if (priv->state == USBSTATE_RESETTING ||
          priv->state == USBSTATE_DETACHING ||
          priv->state == USBSTATE_UNBINDING)
        {
          priv->flush = FLUSH_RESET_TX;
          priv->stopstart = clock_systime_ticks();
          if ((getreg32(STM32_OTG_GRSTCTL) &
               OTG_GRSTCTL_AHBIDL) != 0)
            {
              putreg32(OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXFNUM_ALL,
                       STM32_OTG_GRSTCTL);
            }
          else
            {
              stm32_usb_fault(priv, -EIO);
              return;
            }
        }
    }

  if (priv->flush == FLUSH_RESET_TX &&
      (getreg32(STM32_OTG_GRSTCTL) & OTG_GRSTCTL_TXFFLSH) == 0)
    {
      priv->flush = FLUSH_RESET_RX;
      priv->stopstart = clock_systime_ticks();
      putreg32(OTG_GRSTCTL_RXFFLSH, STM32_OTG_GRSTCTL);
    }

  if (priv->flush == FLUSH_RESET_RX &&
      (getreg32(STM32_OTG_GRSTCTL) & OTG_GRSTCTL_RXFFLSH) == 0)
    {
      priv->flush = FLUSH_IDLE;
      if (priv->state == USBSTATE_UNBINDING)
        {
          priv->state = USBSTATE_QUIESCED;
          nxsem_post(&priv->quiesced);
        }
      else
        {
          enum stm32_usbstate_e old = priv->state;

          stm32_usb_ep0init(priv);
          priv->state = priv->usbdev.speed == USB_SPEED_UNKNOWN ?
                        USBSTATE_BOUND : USBSTATE_ENUMERATED;
          if (priv->driver != NULL)
            {
              CLASS_DISCONNECT(priv->driver, &priv->usbdev);
            }

          if (old == USBSTATE_DETACHING)
            {
              priv->state = USBSTATE_BOUND;
            }

          stm32_usb_connect(priv);
        }
    }

  if (stm32_usb_busy(priv) || priv->flush != FLUSH_IDLE)
    {
      priv->work = WORK_PENDING;
      if ((priv->flush == FLUSH_RESET_TX ||
           priv->flush == FLUSH_RESET_RX) &&
          (clock_t)(clock_systime_ticks() - priv->stopstart) >=
          STM32_USB_STOP_TICKS)
        {
          stm32_usb_fault(priv, -ETIMEDOUT);
          return;
        }

      ret = wd_start(&priv->watchdog, 1, stm32_usb_timeout,
                     (wdparm_t)priv);
      if (ret < 0)
        {
          stm32_usb_fault(priv, ret);
        }
    }
  else
    {
      priv->work = WORK_IDLE;
      wd_cancel(&priv->watchdog);
      stm32_usb_dispatch(priv);
      stm32_usb_start(&priv->epin[0]);
    }
}

static int stm32_ep_configure(struct usbdev_ep_s *public,
                              const struct usb_epdesc_s *desc, bool last)
{
  struct stm32_ep_s *ep = stm32_usb_endpoint(public);
  struct stm32_usbdev_s *priv = &g_otgdev;
  unsigned int number;
  unsigned int packet;
  irqstate_t flags;

  (void)last;
  if (ep == NULL || desc == NULL || desc->addr != ep->ep.eplog ||
      (desc->attr & USB_EP_ATTR_XFERTYPE_MASK) != ep->type)
    {
      return stm32_usb_error(-EINVAL);
    }

  number = stm32_usb_epno(ep);
  packet = GETUINT16(desc->mxpacketsize);
  if (number == 0 || packet == 0 || (packet & ~0x7ffu) != 0 ||
      ep->state == EPSTATE_FREE)
    {
      return stm32_usb_error(-EINVAL);
    }

  if (priv->state != USBSTATE_ENUMERATED ||
      priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      return stm32_usb_error(-ESHUTDOWN);
    }

  if ((priv->usbdev.speed == USB_SPEED_FULL &&
       (packet > 64 || (ep->type == USB_EP_ATTR_XFER_BULK &&
        packet != 8 && packet != 16 && packet != 32 && packet != 64))) ||
      (priv->usbdev.speed == USB_SPEED_HIGH &&
       ((ep->type == USB_EP_ATTR_XFER_BULK && packet != 512) ||
        (ep->type == USB_EP_ATTR_XFER_INT && packet > 1024))))
    {
      return stm32_usb_error(-EINVAL);
    }

  if ((stm32_usb_isin(ep) &&
       g_txfifo_words[number] < (packet + 3) / 4) ||
      (!stm32_usb_isin(ep) &&
       2 * ((packet + 3) / 4 + 1) + 10 +
       2 * STM32_OTG_NENDPOINTS + 1 > STM32_USB_RX_WORDS))
    {
      return stm32_usb_error(-ENOSPC);
    }

  flags = enter_critical_section();
  if (ep->state != EPSTATE_ALLOCATED &&
      !(stm32_usb_stopping(ep) &&
        (ep->action == EPACTION_DISABLE ||
         ep->action == EPACTION_RECONFIGURE)))
    {
      leave_critical_section(flags);
      return stm32_usb_error(-EBUSY);
    }

  ep->ep.maxpacket = packet;
  if (stm32_usb_stopping(ep))
    {
      ep->action = EPACTION_RECONFIGURE;
    }
  else
    {
      stm32_usb_configure(ep);
    }

  leave_critical_section(flags);
  return OK;
}

static int stm32_ep_disable(struct usbdev_ep_s *public)
{
  struct stm32_ep_s *ep = stm32_usb_endpoint(public);
  irqstate_t flags;

  if (ep == NULL || stm32_usb_epno(ep) == 0)
    {
      return stm32_usb_error(-EINVAL);
    }

  flags = enter_critical_section();
  if (ep->state == EPSTATE_FREE || ep->state == EPSTATE_ALLOCATED)
    {
      leave_critical_section(flags);
      return OK;
    }

  if (g_otgdev.state == USBSTATE_FAULT ||
      (g_otgdev.resources & USBRES_CORE) == 0)
    {
      ep->state = EPSTATE_ALLOCATED;
      stm32_usb_cancelprefix(ep, ep->tail, -ESHUTDOWN);
    }
  else
    {
      stm32_usb_stop(ep, EPACTION_DISABLE, -ESHUTDOWN, ep->tail);
      stm32_usb_service(&g_otgdev);
    }

  leave_critical_section(flags);
  return OK;
}

static struct usbdev_req_s *stm32_ep_allocreq(struct usbdev_ep_s *public)
{
  struct stm32_req_s *req;

  if (stm32_usb_endpoint(public) == NULL)
    {
      stm32_usb_error(-EINVAL);
      return NULL;
    }

  req = kmm_zalloc(sizeof(*req));
  if (req == NULL)
    {
      stm32_usb_error(-ENOMEM);
      return NULL;
    }

  return &req->req;
}

static void stm32_ep_freereq(struct usbdev_ep_s *public,
                            struct usbdev_req_s *request)
{
  struct stm32_req_s *req = (struct stm32_req_s *)request;

  if (stm32_usb_endpoint(public) == NULL || req == NULL)
    {
      stm32_usb_error(-EINVAL);
    }
  else if (req->owner != NULL)
    {
      stm32_usb_error(-EBUSY);
    }
  else
    {
      kmm_free(req);
    }
}

static int stm32_ep_submit(struct usbdev_ep_s *public,
                           struct usbdev_req_s *request)
{
  struct stm32_ep_s *ep = stm32_usb_endpoint(public);
  struct stm32_req_s *req = (struct stm32_req_s *)request;
  struct stm32_usbdev_s *priv = &g_otgdev;
  unsigned int number;
  irqstate_t flags;

  if (ep == NULL || req == NULL || request->callback == NULL ||
      (request->len != 0 && request->buf == NULL) ||
      request->len > USBDEV_MAXREQUEUST)
    {
      return stm32_usb_error(-EINVAL);
    }

  flags = enter_critical_section();
  if (req->owner != NULL)
    {
      leave_critical_section(flags);
      return stm32_usb_error(-EBUSY);
    }

  if (priv->driver == NULL || priv->state != USBSTATE_ENUMERATED ||
      priv->link != LINK_CONNECTED ||
      ep->state == EPSTATE_FREE || ep->state == EPSTATE_ALLOCATED)
    {
      leave_critical_section(flags);
      return stm32_usb_error(-ESHUTDOWN);
    }

  if (ep->state == EPSTATE_HALTED ||
      (stm32_usb_stopping(ep) && ep->action != EPACTION_RECONFIGURE))
    {
      leave_critical_section(flags);
      return stm32_usb_error(-EBUSY);
    }

  number = stm32_usb_epno(ep);
  req->limit = request->len;
  if (number == 0)
    {
      if (!stm32_usb_isin(ep) || ep->head != NULL ||
          (priv->control != CTRL_DISPATCH &&
           priv->control != CTRL_ADDRESS))
        {
          leave_critical_section(flags);
          return stm32_usb_error(-EBUSY);
        }

      if ((priv->ctrl.type & USB_DIR_IN) != 0 &&
          GETUINT16(priv->ctrl.len) != 0)
        {
          if (req->limit > GETUINT16(priv->ctrl.len))
            {
              req->limit = GETUINT16(priv->ctrl.len);
            }

          priv->control = CTRL_DATA_IN;
        }
      else
        {
          if (request->len != 0)
            {
              leave_critical_section(flags);
              return stm32_usb_error(-EINVAL);
            }

          if (priv->control != CTRL_ADDRESS)
            {
              priv->control = CTRL_STATUS_IN;
            }
        }
    }

  request->xfrd = 0;
  request->result = -EINPROGRESS;
  req->next = NULL;
  req->owner = ep;
  req->state = REQ_QUEUED;
  if (ep->tail == NULL)
    {
      ep->head = req;
    }
  else
    {
      ep->tail->next = req;
    }

  ep->tail = req;
  usbtrace(TRACE_EPSUBMIT, public->eplog);
  stm32_usb_start(ep);
  leave_critical_section(flags);
  return OK;
}

static int stm32_ep_cancel(struct usbdev_ep_s *public,
                           struct usbdev_req_s *request)
{
  struct stm32_ep_s *ep = stm32_usb_endpoint(public);
  struct stm32_req_s *req;
  struct stm32_req_s *previous = NULL;
  irqstate_t flags;

  if (ep == NULL || request == NULL)
    {
      return stm32_usb_error(-EINVAL);
    }

  flags = enter_critical_section();
  for (req = ep->head; req != NULL && &req->req != request;
       req = req->next)
    {
      previous = req;
    }

  if (req == NULL)
    {
      leave_critical_section(flags);
      return stm32_usb_error(-ENOENT);
    }

  if (previous != NULL)
    {
      previous->next = req->next;
      if (ep->tail == req)
        {
          ep->tail = previous;
        }

      if (ep->lastcancel == req)
        {
          ep->lastcancel = previous;
        }

      stm32_usb_callback(ep, req, -ECONNRESET);
    }
  else if (stm32_usb_stopping(ep))
    {
      leave_critical_section(flags);
      return stm32_usb_error(-EBUSY);
    }
  else
    {
      stm32_usb_stop(ep, EPACTION_CANCEL, -ECONNRESET, req);
      stm32_usb_service(&g_otgdev);
    }

  leave_critical_section(flags);
  return OK;
}

static int stm32_ep_stall(struct usbdev_ep_s *public, bool resume)
{
  struct stm32_ep_s *ep = stm32_usb_endpoint(public);
  irqstate_t flags;

  if (ep == NULL)
    {
      return stm32_usb_error(-EINVAL);
    }

  flags = enter_critical_section();
  if (g_otgdev.state != USBSTATE_ENUMERATED ||
      ep->state == EPSTATE_ALLOCATED || ep->state == EPSTATE_FREE)
    {
      leave_critical_section(flags);
      return stm32_usb_error(-ESHUTDOWN);
    }

  if (stm32_usb_stopping(ep))
    {
      leave_critical_section(flags);
      return stm32_usb_error(-EBUSY);
    }

  if (resume)
    {
      modifyreg32(stm32_usb_epctl(ep), OTG_EPCTL_STALL,
                  OTG_EPCTL_SD0PID);
      ep->state = EPSTATE_IDLE;
      stm32_usb_start(ep);
    }
  else if (stm32_usb_epno(ep) == 0)
    {
      g_otgdev.control = CTRL_STALLED;
      modifyreg32(STM32_OTG_DIEPCTL(0), 0, OTG_EPCTL_STALL);
      modifyreg32(STM32_OTG_DOEPCTL(0), 0, OTG_EPCTL_STALL);
    }
  else
    {
      stm32_usb_stop(ep, EPACTION_HALT, -EPIPE, NULL);
      stm32_usb_service(&g_otgdev);
    }

  leave_critical_section(flags);
  return OK;
}

static struct usbdev_ep_s *stm32_usb_allocep(struct usbdev_s *dev,
                                           uint8_t number, bool in,
                                           uint8_t type)
{
  static const uint16_t sizes[STM32_OTG_NENDPOINTS] =
  {
    CONFIG_USBDEV_EP0_TXFIFO_SIZE, CONFIG_USBDEV_EP1_TXFIFO_SIZE,
    CONFIG_USBDEV_EP2_TXFIFO_SIZE, CONFIG_USBDEV_EP3_TXFIFO_SIZE,
    CONFIG_USBDEV_EP4_TXFIFO_SIZE, CONFIG_USBDEV_EP5_TXFIFO_SIZE,
    CONFIG_USBDEV_EP6_TXFIFO_SIZE, CONFIG_USBDEV_EP7_TXFIFO_SIZE,
    CONFIG_USBDEV_EP8_TXFIFO_SIZE
  };

  struct stm32_ep_s *ep;
  unsigned int n;
  irqstate_t flags;

  number &= ~USB_DIR_IN;
  if (dev != &g_otgdev.usbdev || g_otgdev.driver == NULL ||
      number >= STM32_OTG_NENDPOINTS ||
      (type != USB_EP_ATTR_XFER_BULK && type != USB_EP_ATTR_XFER_INT))
    {
      stm32_usb_error(-EINVAL);
      return NULL;
    }

  flags = enter_critical_section();
  for (n = number == 0 ? 1 : number; n < STM32_OTG_NENDPOINTS; n++)
    {
      ep = in ? &g_otgdev.epin[n] : &g_otgdev.epout[n];
      if (ep->state == EPSTATE_FREE && (!in || sizes[n] != 0))
        {
          ep->state = EPSTATE_ALLOCATED;
          ep->type = type;
          leave_critical_section(flags);
          return &ep->ep;
        }

      if (number != 0)
        {
          break;
        }
    }

  leave_critical_section(flags);
  stm32_usb_error(-ENOSPC);
  return NULL;
}

static void stm32_usb_freeep(struct usbdev_s *dev,
                             struct usbdev_ep_s *public)
{
  struct stm32_ep_s *ep = stm32_usb_endpoint(public);
  irqstate_t flags;

  if (dev != &g_otgdev.usbdev || ep == NULL || stm32_usb_epno(ep) == 0)
    {
      stm32_usb_error(-EINVAL);
      return;
    }

  flags = enter_critical_section();
  if (ep->state == EPSTATE_ALLOCATED || ep->state == EPSTATE_FREE)
    {
      ep->state = EPSTATE_FREE;
    }
  else
    {
      stm32_usb_stop(ep, EPACTION_FREE, -ESHUTDOWN, ep->tail);
      stm32_usb_service(&g_otgdev);
    }

  leave_critical_section(flags);
}

static int stm32_usb_getframe(struct usbdev_s *dev)
{
  if (dev != &g_otgdev.usbdev ||
      (g_otgdev.resources & USBRES_CORE) == 0)
    {
      return stm32_usb_error(-ENODEV);
    }

  return (getreg32(STM32_OTG_DSTS) & OTG_DSTS_FNSOF_MASK) >>
         OTG_DSTS_FNSOF_SHIFT;
}

static int stm32_usb_wakeup(struct usbdev_s *dev)
{
  (void)dev;
  return stm32_usb_error(-EOPNOTSUPP);
}

static int stm32_usb_selfpowered(struct usbdev_s *dev, bool powered)
{
  if (dev != &g_otgdev.usbdev)
    {
      return stm32_usb_error(-EINVAL);
    }

  g_otgdev.powerstatus = powered ? 1 : 0;
  return OK;
}

static void stm32_usb_connect(struct stm32_usbdev_s *priv)
{
  if (priv->link == LINK_PENDING && priv->state == USBSTATE_BOUND)
    {
      stm32_usb_ep0init(priv);
      priv->link = LINK_CONNECTED;
      modifyreg32(STM32_OTG_GCCFG, 0, OTG_GCCFG_VBVALOVAL);
      modifyreg32(STM32_OTG_DCTL, OTG_DCTL_SDIS, 0);
    }
}

static int stm32_usb_pullup(struct usbdev_s *dev, bool enable)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  irqstate_t flags;
  int ret = OK;

  if (dev != &priv->usbdev || priv->driver == NULL ||
      priv->state == USBSTATE_FAULT)
    {
      return stm32_usb_error(-ENODEV);
    }

  flags = enter_critical_section();
  if (enable)
    {
      switch (priv->link)
        {
          case LINK_ABSENT:
          case LINK_WAITING:
            priv->link = LINK_WAITING;
            ret = -ENOTCONN;
            break;

          case LINK_PRESENT:
            priv->link = LINK_PENDING;
            stm32_usb_connect(priv);
            break;

          case LINK_PENDING:
            stm32_usb_connect(priv);
            break;

          default:
            break;
        }
    }
  else if (priv->link == LINK_CONNECTED)
    {
      priv->link = LINK_PRESENT;
      modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SDIS);
      modifyreg32(STM32_OTG_GCCFG, OTG_GCCFG_VBVALOVAL, 0);
      stm32_usb_resetstart(priv, USBSTATE_DETACHING);
    }
  else if (priv->link != LINK_PRESENT && priv->link != LINK_ABSENT)
    {
      priv->link = priv->link == LINK_PENDING ? LINK_PRESENT : LINK_ABSENT;
    }

  leave_critical_section(flags);
  return ret < 0 ? stm32_usb_error(ret) : ret;
}

static const struct usbdev_epops_s g_epops =
{
  .configure = stm32_ep_configure,
  .disable = stm32_ep_disable,
  .allocreq = stm32_ep_allocreq,
  .freereq = stm32_ep_freereq,
  .submit = stm32_ep_submit,
  .cancel = stm32_ep_cancel,
  .stall = stm32_ep_stall
};

static const struct usbdev_ops_s g_devops =
{
  .allocep = stm32_usb_allocep,
  .freeep = stm32_usb_freeep,
  .getframe = stm32_usb_getframe,
  .wakeup = stm32_usb_wakeup,
  .selfpowered = stm32_usb_selfpowered,
  .pullup = stm32_usb_pullup
};

static void stm32_usb_software(struct stm32_usbdev_s *priv)
{
  unsigned int n;

  memset(priv->epin, 0, sizeof(priv->epin));
  memset(priv->epout, 0, sizeof(priv->epout));
  memset(&priv->response, 0, sizeof(priv->response));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->epin[0].ep;
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
#ifdef CONFIG_USBDEV_DUALSPEED
  priv->usbdev.dualspeed = 1;
#endif
  priv->link = LINK_ABSENT;
  priv->control = CTRL_SETUP;
  priv->work = WORK_IDLE;
  priv->flush = FLUSH_IDLE;
  priv->configuration = 0;
  priv->outlen = 0;
  for (n = 0; n < STM32_OTG_NENDPOINTS; n++)
    {
      priv->epin[n].ep.ops = &g_epops;
      priv->epin[n].ep.eplog = n | USB_DIR_IN;
      priv->epin[n].ep.maxpacket = 64;
      priv->epout[n].ep.ops = &g_epops;
      priv->epout[n].ep.eplog = n;
      priv->epout[n].ep.maxpacket = 64;
      priv->epin[n].state = n == 0 ? EPSTATE_IDLE : EPSTATE_FREE;
      priv->epout[n].state = n == 0 ? EPSTATE_IDLE : EPSTATE_FREE;
    }
}

static void stm32_usb_ctrlstall(struct stm32_usbdev_s *priv, int result)
{
  stm32_usb_error(result);
  priv->control = CTRL_STALLED;
  modifyreg32(STM32_OTG_DIEPCTL(0), 0, OTG_EPCTL_STALL);
  modifyreg32(STM32_OTG_DOEPCTL(0), 0, OTG_EPCTL_STALL);
  stm32_usb_armout0();
  if (priv->epin[0].head != NULL)
    {
      stm32_usb_stop(&priv->epin[0], EPACTION_HALT, result,
                     priv->epin[0].tail);
      stm32_usb_service(priv);
    }
}

static void stm32_usb_response_done(struct usbdev_ep_s *ep,
                                   struct usbdev_req_s *req)
{
  (void)ep;
  (void)req;
}

static void stm32_usb_reply(struct stm32_usbdev_s *priv, size_t size)
{
  int ret;

  priv->response.req.buf = priv->reply;
  priv->response.req.len = size;
  priv->response.req.flags = 0;
  priv->response.req.callback = stm32_usb_response_done;
  ret = stm32_ep_submit(priv->usbdev.ep0, &priv->response.req);
  if (ret < 0)
    {
      stm32_usb_ctrlstall(priv, ret);
    }
}

static void stm32_usb_dispatch(struct stm32_usbdev_s *priv)
{
  struct stm32_ep_s *ep;
  uint16_t value = GETUINT16(priv->ctrl.value);
  uint16_t index = GETUINT16(priv->ctrl.index);
  uint16_t length = GETUINT16(priv->ctrl.len);
  uint8_t type = priv->ctrl.type;
  int ret;

  if (priv->state != USBSTATE_ENUMERATED ||
      priv->epin[0].state != EPSTATE_IDLE)
    {
      return;
    }

  if (priv->control == CTRL_HALT || priv->control == CTRL_RESUME)
    {
      ep = stm32_usb_address(index);
      if (ep == NULL || stm32_usb_stopping(ep))
        {
          return;
        }

      if (priv->control == CTRL_RESUME)
        {
          ret = stm32_ep_stall(&ep->ep, true);
          if (ret < 0)
            {
              stm32_usb_ctrlstall(priv, ret);
              return;
            }
        }

      priv->control = CTRL_DISPATCH;
      stm32_usb_reply(priv, 0);
      return;
    }

  if (priv->control != CTRL_READY)
    {
      return;
    }

  priv->control = CTRL_DISPATCH;
  if ((type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      switch (priv->ctrl.req)
        {
          case USB_REQ_GETSTATUS:
            if (value != 0 || length != 2 || (type & USB_DIR_IN) == 0)
              {
                break;
              }

            priv->reply[0] = 0;
            priv->reply[1] = 0;
            if (type == USB_DIR_IN && index == 0)
              {
                priv->reply[0] = priv->powerstatus;
              }
            else if (type == (USB_DIR_IN | USB_REQ_RECIPIENT_ENDPOINT))
              {
                ep = stm32_usb_address(index);
                if (ep == NULL || ep->state == EPSTATE_FREE ||
                    ep->state == EPSTATE_ALLOCATED)
                  {
                    break;
                  }

                priv->reply[0] = ep->state == EPSTATE_HALTED ? 1 : 0;
              }
            else if (type != (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) ||
                     priv->configuration == 0 || index > 255)
              {
                break;
              }

            stm32_usb_reply(priv, 2);
            return;

          case USB_REQ_SETADDRESS:
            if (type != 0 || index != 0 || length != 0 || value > 127 ||
                priv->configuration != 0)
              {
                break;
              }

            priv->control = CTRL_ADDRESS;
            stm32_usb_reply(priv, 0);
            return;

          case USB_REQ_CLEARFEATURE:
          case USB_REQ_SETFEATURE:
            if (type != USB_REQ_RECIPIENT_ENDPOINT || length != 0 ||
                value != USB_FEATURE_ENDPOINTHALT)
              {
                break;
              }

            ep = stm32_usb_address(index);
            if (ep == NULL || stm32_usb_epno(ep) == 0 ||
                ep->state == EPSTATE_ALLOCATED || ep->state == EPSTATE_FREE)
              {
                break;
              }

            if (priv->ctrl.req == USB_REQ_SETFEATURE)
              {
                ret = stm32_ep_stall(&ep->ep, false);
                if (ret < 0)
                  {
                    break;
                  }

                priv->control = CTRL_HALT;
              }
            else
              {
                priv->control = CTRL_RESUME;
              }

            stm32_usb_dispatch(priv);
            return;

          case USB_REQ_GETDESCRIPTOR:
          case USB_REQ_SETDESCRIPTOR:
            if (type != (priv->ctrl.req == USB_REQ_GETDESCRIPTOR ?
                         USB_DIR_IN : 0) || length == 0)
              {
                break;
              }

            goto dispatch;

          case USB_REQ_GETCONFIGURATION:
            if (type != USB_DIR_IN || value != 0 || index != 0 ||
                length != 1)
              {
                break;
              }

            goto dispatch;

          case USB_REQ_SETCONFIGURATION:
            if (type != 0 || value > 255 || index != 0 || length != 0 ||
                (getreg32(STM32_OTG_DCFG) & OTG_DCFG_DAD_MASK) == 0)
              {
                break;
              }

            goto dispatch;

          case USB_REQ_GETINTERFACE:
          case USB_REQ_SETINTERFACE:
            if (priv->configuration == 0 || index > 255 ||
                (type & USB_REQ_RECIPIENT_MASK) !=
                USB_REQ_RECIPIENT_INTERFACE ||
                (priv->ctrl.req == USB_REQ_GETINTERFACE &&
                 (type != (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) ||
                  value != 0 || length != 1)) ||
                (priv->ctrl.req == USB_REQ_SETINTERFACE &&
                 (type != USB_REQ_RECIPIENT_INTERFACE ||
                  value > 255 || length != 0)))
              {
                break;
              }

            goto dispatch;

          default:
            break;
        }

      stm32_usb_ctrlstall(priv, -EINVAL);
      return;
    }

dispatch:
  if ((type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD &&
      (type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_CLASS &&
      (type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_VENDOR)
    {
      stm32_usb_ctrlstall(priv, -EINVAL);
      return;
    }

  ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                    priv->setupdata, priv->outlen);
  if (ret < 0)
    {
      stm32_usb_ctrlstall(priv, ret);
    }
  else
    {
      if (type == 0 && priv->ctrl.req == USB_REQ_SETCONFIGURATION)
        {
          priv->configuration = value;
        }

      /* Classes may submit an asynchronous EP0 response after returning. */
    }
}

static void stm32_usb_receive(struct stm32_usbdev_s *priv)
{
  struct stm32_ep_s *ep;
  struct stm32_req_s *req;
  uint32_t status;
  unsigned int number;
  unsigned int count;
  unsigned int budget;
  size_t available;
  uint16_t length;

  for (budget = 0; budget < 32 &&
       (getreg32(STM32_OTG_GINTSTS) & OTG_GINT_RXFLVL) != 0; budget++)
    {
      status = getreg32(STM32_OTG_GRXSTSP);
      number = status & OTG_GRXST_EPNUM_MASK;
      count = (status & OTG_GRXST_BCNT_MASK) >> OTG_GRXST_BCNT_SHIFT;
      if (number >= STM32_OTG_NENDPOINTS)
        {
          stm32_usb_readfifo(NULL, 0, count);
          stm32_usb_error(-EINVAL);
          continue;
        }

      ep = &priv->epout[number];
      switch (status & OTG_GRXST_PKTSTS_MASK)
        {
          case OTG_GRXST_PKTSTS_SETUPRECVD:
            if (number != 0 || count != sizeof(priv->ctrl))
              {
                stm32_usb_readfifo(NULL, 0, count);
                stm32_usb_ctrlstall(priv, -EPROTO);
                break;
              }

            stm32_usb_readfifo((uint8_t *)&priv->ctrl,
                               sizeof(priv->ctrl), count);
            priv->control = CTRL_ABORT;
            priv->outlen = 0;
            ep->received = 0;
            stm32_usb_stop(&priv->epin[0], EPACTION_SETUP,
                           -EPROTO, priv->epin[0].tail);
            break;

          case OTG_GRXST_PKTSTS_SETUPDONE:
            length = GETUINT16(priv->ctrl.len);
            if (priv->control == CTRL_ABORT)
              {
                if ((priv->ctrl.type & USB_DIR_IN) == 0 && length != 0)
                  {
                    if (length > sizeof(priv->setupdata))
                      {
                        stm32_usb_ctrlstall(priv, -EOVERFLOW);
                      }
                    else
                      {
                        priv->control = CTRL_RECEIVE;
                        stm32_usb_armout0();
                      }
                  }
                else
                  {
                    priv->control = CTRL_READY;
                  }
              }

            break;

          case OTG_GRXST_PKTSTS_OUTRECVD:
            if (number == 0)
              {
                ep->received = count;
                if (priv->control == CTRL_RECEIVE)
                  {
                    available = GETUINT16(priv->ctrl.len) - priv->outlen;
                    stm32_usb_readfifo(priv->setupdata + priv->outlen,
                                       available, count);
                    priv->outlen += count < available ? count : available;
                    if (count > available || count > 64)
                      {
                        stm32_usb_ctrlstall(priv, -EOVERFLOW);
                      }
                  }
                else
                  {
                    stm32_usb_readfifo(NULL, 0, count);
                    if (priv->control == CTRL_STATUS_OUT && count != 0)
                      {
                        stm32_usb_ctrlstall(priv, -EPROTO);
                      }
                  }
              }
            else
              {
                req = ep->head;
                if (req == NULL || (ep->state != EPSTATE_RX_ACTIVE &&
                    !stm32_usb_stopping(ep)))
                  {
                    stm32_usb_readfifo(NULL, 0, count);
                    stm32_usb_error(-EPROTO);
                    break;
                  }

                available = req->req.len - req->req.xfrd;
                stm32_usb_readfifo(req->req.buf == NULL ? NULL :
                                   req->req.buf + req->req.xfrd,
                                   available, count);
                req->req.xfrd += count < available ? count : available;
                ep->received = count;
                if (count > available || count > ep->ep.maxpacket)
                  {
                    req->req.result = -EOVERFLOW;
                    stm32_usb_error(-EOVERFLOW);
                  }
              }

            break;

          case OTG_GRXST_PKTSTS_GONAK:
          case OTG_GRXST_PKTSTS_OUTDONE:
            if (count != 0)
              {
                stm32_usb_readfifo(NULL, 0, count);
                stm32_usb_error(-EPROTO);
              }

            break;

          default:
            stm32_usb_readfifo(NULL, 0, count);
            stm32_usb_error(-EPROTO);
            break;
        }
    }
}

static void stm32_usb_ack(struct stm32_ep_s *ep)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  struct stm32_req_s *req = ep->head;
  size_t packet = ep->packet;
  unsigned int number = stm32_usb_epno(ep);

  if (req == NULL || (req->state != REQ_DATA && req->state != REQ_ZLP))
    {
      return;
    }

  req->req.xfrd += packet;
  ep->packet = 0;
  if (req->req.xfrd < req->limit ||
      (packet != 0 && req->limit % ep->ep.maxpacket == 0 &&
       ((number == 0 && priv->control == CTRL_DATA_IN &&
         req->limit < GETUINT16(priv->ctrl.len)) ||
        (number != 0 &&
         (req->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0))))
    {
      req->state = REQ_QUEUED;
    }
  else
    {
      req->state = REQ_COMPLETE;
    }
}

static void stm32_usb_incomplete(struct stm32_ep_s *ep)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  struct stm32_req_s *req = ep->head;
  unsigned int number = stm32_usb_epno(ep);

  if (req == NULL || ep->state != EPSTATE_TX_ACTIVE)
    {
      if (stm32_usb_stopping(ep))
        {
          stm32_usb_ack(ep);
        }

      return;
    }

  stm32_usb_ack(ep);
  ep->state = EPSTATE_IDLE;
  if (req->state != REQ_COMPLETE)
    {
      stm32_usb_start(ep);
      return;
    }

  if (number == 0)
    {
      if (priv->control == CTRL_ADDRESS)
        {
          modifyreg32(STM32_OTG_DCFG, OTG_DCFG_DAD_MASK,
                      (uint32_t)GETUINT16(priv->ctrl.value) <<
                      OTG_DCFG_DAD_SHIFT);
          priv->control = CTRL_SETUP;
        }
      else if (priv->control == CTRL_DATA_IN)
        {
          priv->control = CTRL_STATUS_OUT;
        }
      else
        {
          priv->control = CTRL_SETUP;
        }

      stm32_usb_armout0();
    }

  stm32_usb_complete(ep, OK);
  stm32_usb_start(ep);
}

static void stm32_usb_outack(struct stm32_ep_s *ep, uint32_t intr)
{
  struct stm32_req_s *req = ep->head;

  if (req != NULL && (intr & OTG_DOEPINT_XFRC) != 0)
    {
      req->state = req->req.xfrd == req->req.len ||
                   ep->received < ep->ep.maxpacket ||
                   req->req.result != -EINPROGRESS ?
                   REQ_COMPLETE : REQ_QUEUED;
    }
}

static void stm32_usb_outcomplete(struct stm32_ep_s *ep)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  struct stm32_req_s *req = ep->head;

  if (stm32_usb_epno(ep) == 0)
    {
      if (priv->control == CTRL_RECEIVE)
        {
          if (ep->received < 64 ||
              priv->outlen == GETUINT16(priv->ctrl.len))
            {
              priv->control = CTRL_READY;
            }
          else
            {
              stm32_usb_armout0();
            }
        }
      else if (priv->control == CTRL_STATUS_OUT)
        {
          priv->control = CTRL_SETUP;
          stm32_usb_armout0();
        }

      return;
    }

  if (ep->state != EPSTATE_RX_ACTIVE || req == NULL)
    {
      return;
    }

  ep->state = EPSTATE_IDLE;
  stm32_usb_outack(ep, OTG_DOEPINT_XFRC);
  if (req->state == REQ_COMPLETE)
    {
      stm32_usb_complete(ep, req->req.result == -EINPROGRESS ?
                            OK : req->req.result);
    }

  stm32_usb_start(ep);
}

static void stm32_usb_resetstart(struct stm32_usbdev_s *priv,
                                 enum stm32_usbstate_e state)
{
  unsigned int n;
  enum stm32_usbstate_e old = priv->state;

  priv->state = state;
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  priv->control = CTRL_SETUP;
  priv->configuration = 0;
  modifyreg32(STM32_OTG_DCFG, OTG_DCFG_DAD_MASK, 0);
  for (n = 0; n < STM32_OTG_NENDPOINTS; n++)
    {
      if (n == 0 || priv->epin[n].state != EPSTATE_FREE)
        {
          stm32_usb_stop(&priv->epin[n],
                         n == 0 ? EPACTION_SETUP : EPACTION_DISABLE,
                         -ESHUTDOWN, priv->epin[n].tail);
        }

      if (n != 0 && priv->epout[n].state != EPSTATE_FREE)
        {
          stm32_usb_stop(&priv->epout[n], EPACTION_DISABLE,
                         -ESHUTDOWN, priv->epout[n].tail);
        }
    }

  modifyreg32(STM32_OTG_DOEPCTL(0), 0, OTG_EPCTL_SNAK);
  if (old != USBSTATE_RESETTING && old != USBSTATE_DETACHING &&
      old != USBSTATE_UNBINDING)
    {
      priv->work = WORK_IDLE;
    }

  stm32_usb_service(priv);
}

static int stm32_usb_interrupt(int irq, void *context, void *arg)
{
  struct stm32_usbdev_s *priv = arg;
  struct stm32_ep_s *ep;
  unsigned int n;
  uint32_t pending;
  uint32_t intr;
  uint32_t speed;
  irqstate_t flags;

  (void)irq;
  (void)context;
  flags = enter_critical_section();
  if (priv->state == USBSTATE_READY || priv->state == USBSTATE_FAULT ||
      priv->state == USBSTATE_OFF ||
      (priv->resources & USBRES_CORE) == 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  usbtrace(TRACE_INTENTRY(1), 0);
  pending = getreg32(STM32_OTG_GINTSTS) & getreg32(STM32_OTG_GINTMSK);
  putreg32(pending & OTG_GINTSTS_W1C_MASK, STM32_OTG_GINTSTS);
  if ((pending & OTG_GINT_USBRST) != 0 &&
      (priv->state == USBSTATE_BOUND ||
       priv->state == USBSTATE_ENUMERATED ||
       priv->state == USBSTATE_SUSPENDED ||
       priv->state == USBSTATE_RESETTING))
    {
      stm32_usb_resetstart(priv, USBSTATE_RESETTING);
    }

  stm32_usb_receive(priv);
  if ((pending & OTG_GINT_ENUMDNE) != 0)
    {
      speed = getreg32(STM32_OTG_DSTS) & OTG_DSTS_ENUMSPD_MASK;
      if (speed != OTG_DSTS_ENUMSPD_FS && speed != OTG_DSTS_ENUMSPD_HS)
        {
          stm32_usb_fault(priv, -EPROTO);
          leave_critical_section(flags);
          return OK;
        }

      priv->usbdev.speed = speed == OTG_DSTS_ENUMSPD_FS ?
                           USB_SPEED_FULL : USB_SPEED_HIGH;
      if (priv->state == USBSTATE_BOUND)
        {
          priv->state = USBSTATE_ENUMERATED;
        }
    }

  for (n = 0; n < STM32_OTG_NENDPOINTS; n++)
    {
      intr = getreg32(STM32_OTG_DIEPINT(n)) &
             (getreg32(STM32_OTG_DIEPMSK) | OTG_DIEPINT_TXFE);
      putreg32(intr & OTG_DIEPINT_W1C_MASK &
               ~(OTG_DIEPINT_EPDISD | OTG_DIEPINT_INEPNE),
               STM32_OTG_DIEPINT(n));
      ep = &priv->epin[n];
      if ((intr & (OTG_DIEPINT_TOC | OTG_DIEPINT_TXFIFOUDRN)) != 0 &&
          ep->head != NULL)
        {
          stm32_usb_stop(ep, EPACTION_CANCEL, -EIO, ep->head);
        }

      if ((intr & OTG_DIEPINT_XFRC) != 0)
        {
          stm32_usb_incomplete(ep);
        }

      if ((intr & OTG_DIEPINT_TXFE) != 0 &&
          (getreg32(STM32_OTG_DIEPEMPMSK) & OTG_DAINT_IN(n)) != 0)
        {
          stm32_usb_start(ep);
        }

      intr = getreg32(STM32_OTG_DOEPINT(n)) &
             getreg32(STM32_OTG_DOEPMSK);
      putreg32(intr & OTG_DOEPINT_W1C_MASK & ~OTG_DOEPINT_EPDISD,
               STM32_OTG_DOEPINT(n));
      if ((intr & (OTG_DOEPINT_OUTPKTERR | OTG_DOEPINT_BERR)) != 0 &&
          n != 0 && priv->epout[n].head != NULL)
        {
          stm32_usb_stop(&priv->epout[n], EPACTION_CANCEL, -EIO,
                         priv->epout[n].head);
        }

      if ((intr & OTG_DOEPINT_XFRC) != 0 &&
          (n != 0 || (intr & (OTG_DOEPINT_STUP |
                              OTG_DOEPINT_B2BSTUP)) == 0))
        {
          stm32_usb_outcomplete(&priv->epout[n]);
        }
    }

  if ((pending & OTG_GINT_USBSUSP) != 0 &&
      priv->state == USBSTATE_ENUMERATED)
    {
      priv->state = USBSTATE_SUSPENDED;
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
      if (stm32_usbsuspend != NULL)
        {
          stm32_usbsuspend(&priv->usbdev, false);
        }
    }

  if ((pending & OTG_GINT_WKUPINT) != 0 &&
      priv->state == USBSTATE_SUSPENDED)
    {
      priv->state = USBSTATE_ENUMERATED;
      if (stm32_usbsuspend != NULL)
        {
          stm32_usbsuspend(&priv->usbdev, true);
        }

      CLASS_RESUME(priv->driver, &priv->usbdev);
      for (n = 0; n < STM32_OTG_NENDPOINTS; n++)
        {
          stm32_usb_start(&priv->epin[n]);
          stm32_usb_start(&priv->epout[n]);
        }
    }

  stm32_usb_service(priv);
  usbtrace(TRACE_INTEXIT(1), 0);
  leave_critical_section(flags);
  return OK;
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
      if (priv->state == USBSTATE_OFF || priv->state == USBSTATE_FAULT)
        {
          priv->initresult = ret;
        }

      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INITFAILED), -ret);
      uerr("ERROR: USB%d initialization context denied: %d\n",
           STM32_OTG_PORT, ret);
      return;
    }

  flags = enter_critical_section();
  if (priv->state != USBSTATE_OFF && priv->state != USBSTATE_FAULT)
    {
      ret = priv->state == USBSTATE_INITIALIZING ||
            priv->state == USBSTATE_UNINITIALIZING ||
            priv->state == USBSTATE_BINDING ||
            priv->state == USBSTATE_UNBINDING ? -EBUSY : OK;
      leave_critical_section(flags);
      if (ret < 0)
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INITFAILED), -ret);
          uerr("ERROR: USB initialization already in progress\n");
        }

      return;
    }

  if (priv->driver != NULL)
    {
      leave_critical_section(flags);
      stm32_usb_error(-EBUSY);
      return;
    }

  priv->state = USBSTATE_INITIALIZING;
  priv->initresult = -EBUSY;
  leave_critical_section(flags);

  ret = stm32_usb_cleanup(priv);
  if (ret == OK)
    {
      ret = stm32_usb_hwinitialize(priv);
    }

  if (ret == OK)
    {
      stm32_usb_software(priv);
      ret = irq_attach(STM32_IRQ_OTG, stm32_usb_interrupt, priv);
      if (ret == OK)
        {
          priv->resources |= USBRES_IRQ;
        }
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
  priv->state = ret == OK ? USBSTATE_READY : USBSTATE_FAULT;
  leave_critical_section(flags);
}

void arm_usbuninitialize(void)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  irqstate_t flags;
  int ret;
  int detach = OK;

  usbtrace(TRACE_DEVUNINIT, STM32_OTG_PORT);
  ret = stm32_usb_check_access();
  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), -ret);
      uerr("ERROR: USB%d uninitialization context denied: %d\n",
           STM32_OTG_PORT, ret);
      return;
    }

  if (priv->driver != NULL)
    {
      ret = usbdev_unregister(priv->driver);
      if (ret < 0)
        {
          stm32_usb_error(ret);
          return;
        }
    }

  flags = enter_critical_section();
  if (priv->state == USBSTATE_INITIALIZING ||
      priv->state == USBSTATE_UNINITIALIZING ||
      priv->state == USBSTATE_BINDING)
    {
      leave_critical_section(flags);
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), EBUSY);
      uerr("ERROR: USB initialization already in progress\n");
      return;
    }

  priv->state = USBSTATE_UNINITIALIZING;
  priv->initresult = -EBUSY;
  leave_critical_section(flags);
  wd_cancel(&priv->watchdog);
  if ((priv->resources & USBRES_IRQ) != 0)
    {
      up_disable_irq(STM32_IRQ_OTG);
      detach = irq_detach(STM32_IRQ_OTG);
      if (detach < 0)
        {
          stm32_usb_error(detach);
        }
      else
        {
          priv->resources &= ~USBRES_IRQ;
        }
    }

  ret = stm32_usb_cleanup(priv);
  if (ret == OK && detach < 0)
    {
      ret = detach;
    }

  if (ret < 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_CLEANUPFAILED), -ret);
      uerr("ERROR: USB%d cleanup failed: %d\n", STM32_OTG_PORT, ret);
    }

  flags = enter_critical_section();
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  priv->initresult = ret < 0 ? ret : -ENODEV;
  priv->state = ret < 0 ? USBSTATE_FAULT : USBSTATE_OFF;
  leave_critical_section(flags);
}

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  enum stm32_linkstate_e link;
  irqstate_t flags;
  uint32_t speed;
  int ret;

  usbtrace(TRACE_DEVREGISTER, STM32_OTG_PORT);
  if (driver == NULL || driver->ops == NULL || driver->ops->bind == NULL ||
      driver->ops->unbind == NULL || driver->ops->setup == NULL ||
      driver->ops->disconnect == NULL ||
      (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH))
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), EINVAL);
      uerr("ERROR: Invalid USB class driver\n");
      return -EINVAL;
    }

  ret = stm32_usb_check_access();
  if (ret < 0)
    {
      return stm32_usb_error(ret);
    }

  flags = enter_critical_section();
  if (priv->state != USBSTATE_READY)
    {
      ret = priv->initresult < 0 ? priv->initresult : -EBUSY;
      leave_critical_section(flags);
      return stm32_usb_error(ret);
    }

  priv->state = USBSTATE_BINDING;
  priv->driver = driver;
  speed = OTG_DCFG_DSPD_FS;
#ifndef CONFIG_STM32_N6_OTGDEV_FS
#  ifdef CONFIG_USBDEV_DUALSPEED
  if (driver->speed == USB_SPEED_HIGH)
    {
      speed = OTG_DCFG_DSPD_HS;
    }
#  endif
#endif

  modifyreg32(STM32_OTG_DCFG, OTG_DCFG_DSPD_MASK, speed);
  leave_critical_section(flags);
  ret = CLASS_BIND(driver, &priv->usbdev);
  flags = enter_critical_section();
  if (ret < 0)
    {
      link = priv->link == LINK_PRESENT || priv->link == LINK_PENDING ?
             LINK_PRESENT : LINK_ABSENT;
      priv->driver = NULL;
      stm32_usb_software(priv);
      priv->link = link;
      priv->state = USBSTATE_READY;
      leave_critical_section(flags);
      return stm32_usb_error(ret);
    }

  stm32_usb_ep0init(priv);
  priv->state = USBSTATE_BOUND;
  putreg32(OTG_GINTSTS_W1C_MASK, STM32_OTG_GINTSTS);
  putreg32(STM32_USB_IRQ_MASK, STM32_OTG_GINTMSK);
  modifyreg32(STM32_OTG_GAHBCFG, 0, OTG_GAHBCFG_GINTMSK);
  stm32_usb_connect(priv);
  up_enable_irq(STM32_IRQ_OTG);
  leave_critical_section(flags);
  return OK;
}

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  enum stm32_linkstate_e link;
  irqstate_t flags;
  int ret;

  usbtrace(TRACE_DEVUNREGISTER, STM32_OTG_PORT);
  if (driver == NULL)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), EINVAL);
      uerr("ERROR: Invalid USB class driver\n");
      return -EINVAL;
    }

  ret = stm32_usb_check_access();
  if (ret < 0)
    {
      return stm32_usb_error(ret);
    }

  flags = enter_critical_section();
  if (driver != priv->driver)
    {
      leave_critical_section(flags);
      return stm32_usb_error(-ENODEV);
    }

  if (priv->state == USBSTATE_BINDING ||
      priv->state == USBSTATE_UNBINDING ||
      priv->state == USBSTATE_QUIESCED)
    {
      leave_critical_section(flags);
      return stm32_usb_error(-EBUSY);
    }

  if (priv->state != USBSTATE_FAULT)
    {
      priv->link = priv->link == LINK_CONNECTED ||
                   priv->link == LINK_PRESENT || priv->link == LINK_PENDING ?
                   LINK_PRESENT : LINK_ABSENT;
      modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SDIS);
      modifyreg32(STM32_OTG_GCCFG, OTG_GCCFG_VBVALOVAL, 0);
      stm32_usb_resetstart(priv, USBSTATE_UNBINDING);
      leave_critical_section(flags);
      ret = nxsem_wait_uninterruptible(&priv->quiesced);
      if (ret < 0)
        {
          return stm32_usb_error(ret);
        }

      flags = enter_critical_section();
    }

  up_disable_irq(STM32_IRQ_OTG);
  if ((priv->resources & USBRES_CORE) != 0)
    {
      putreg32(0, STM32_OTG_GINTMSK);
      putreg32(0, STM32_OTG_GAHBCFG);
    }

  leave_critical_section(flags);
  CLASS_DISCONNECT(driver, &priv->usbdev);
  CLASS_UNBIND(driver, &priv->usbdev);
  flags = enter_critical_section();
  priv->driver = NULL;
  if (priv->state != USBSTATE_FAULT)
    {
      link = priv->link == LINK_PRESENT || priv->link == LINK_PENDING ||
             priv->link == LINK_CONNECTED ? LINK_PRESENT : LINK_ABSENT;
      stm32_usb_software(priv);
      priv->link = link;
      priv->state = USBSTATE_READY;
    }

  leave_critical_section(flags);
  return OK;
}

int stm32_usbdev_vbus(bool present)
{
  struct stm32_usbdev_s *priv = &g_otgdev;
  irqstate_t flags;

  if ((priv->resources & USBRES_CORE) == 0 ||
      priv->state == USBSTATE_INITIALIZING ||
      priv->state == USBSTATE_UNINITIALIZING ||
      priv->state == USBSTATE_FAULT)
    {
      return stm32_usb_error(-ENODEV);
    }

  flags = enter_critical_section();
  if (present)
    {
      if (priv->link == LINK_ABSENT)
        {
          priv->link = LINK_PRESENT;
        }
      else if (priv->link == LINK_WAITING)
        {
          priv->link = LINK_PENDING;
          stm32_usb_connect(priv);
        }
    }
  else
    {
      enum stm32_linkstate_e old = priv->link;

      priv->link = old == LINK_CONNECTED || old == LINK_PENDING ||
                   old == LINK_WAITING ? LINK_WAITING : LINK_ABSENT;
      if (old == LINK_CONNECTED)
        {
          modifyreg32(STM32_OTG_DCTL, 0, OTG_DCTL_SDIS);
          modifyreg32(STM32_OTG_GCCFG, OTG_GCCFG_VBVALOVAL, 0);
          stm32_usb_resetstart(priv, USBSTATE_DETACHING);
        }
    }

  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_STM32_N6_OTGDEV */
