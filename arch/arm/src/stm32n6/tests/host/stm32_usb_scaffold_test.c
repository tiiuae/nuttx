/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_usb_scaffold_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAR
#define CODE
#define EXTERN extern
#define printf_like(a, b)
#define STM32_IRQ_FIRST 16
#define uerr test_error
#define OK 0
#define DEBUGASSERT assert
#define kmm_malloc malloc
#define kmm_zalloc(s) calloc(1, (s))
#define kmm_free free
#define MSEC2TICK(m) (m)
#define USEC2TICK(u) (((u) + 999) / 1000)
#define TICK2MSEC(t) (t)
#define SEM_INITIALIZER(n) {.count = (n)}
#define CONTROL_NPRIV 1
#define TEST_CLOCKS (STM32_OTG_RCC_EN | STM32_OTG_RCC_PHY_EN)
#define TEST_RESETS (STM32_OTG_RCC_RST | STM32_OTG_RCC_PHY_RST | \
                     STM32_OTG_RCC_PHYCTL_RST)

/* USB_HEADERS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef unsigned int irqstate_t;
typedef int (*xcpt_t)(int, void *, void *);
typedef uintptr_t wdparm_t;
typedef void (*wdentry_t)(uintptr_t);

typedef struct
{
  unsigned int count;
} sem_t;

struct wdog_s
{
  wdentry_t callback;
  uintptr_t arg;
  unsigned int due;
  bool active;
};

enum test_failure_e
{
  TEST_NONE,
  TEST_RESET,
  TEST_MONITOR_WRITE,
  TEST_SUPPLY,
  TEST_VALID_WRITE,
  TEST_HSE_STOP,
  TEST_HSE_CONFIG,
  TEST_HSE_ENABLE,
  TEST_HSE_READY,
  TEST_HSE_SETTLE,
  TEST_CLKSEL,
  TEST_CLOCK_ENABLE,
  TEST_PHYCTL_RESET,
  TEST_PHY_CONFIG,
  TEST_PHY_RESET,
  TEST_PHY_SUPPLY,
  TEST_CORE_RESET,
  TEST_IDLE,
  TEST_SOFT_RESET,
  TEST_RESET_IDLE,
  TEST_MODE,
  TEST_CORE_HSE,
  TEST_CORE_SUPPLY,
  TEST_NAK_EFFECTIVE,
  TEST_NAK,
  TEST_INTERRUPT_MASK,
  TEST_RX_SIZE,
  TEST_TX_SIZE,
  TEST_ENDPOINT,
  TEST_TX_FLUSH,
  TEST_RX_FLUSH,
  TEST_CLOCK_CLEAR,
  TEST_CLEANUP_RESET
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned int g_errors;
static unsigned int g_binds;
static uint32_t g_rcc[0x1300 / 4];
static uint32_t g_pwr[0x80 / 4];
static uint32_t g_phy[3];
static uint32_t g_core[0x1000 / 4];
static unsigned int g_time;
static unsigned int g_reads;
static unsigned int g_writes;
static unsigned int g_irqdisables;
static unsigned int g_critical;
static uint32_t g_control;
static uint32_t g_ipsr;
static unsigned int g_hse_due;
static unsigned int g_hse_ready;
static unsigned int g_phy_released;
static unsigned int g_reset_due;
static unsigned int g_reset_done;
static unsigned int g_mode_due;
static unsigned int g_nak_due;
static enum test_failure_e g_failure;
static bool g_soft_reset;
static bool g_reenter;
static bool g_runtime;
static bool g_irqenabled;
static bool g_delayed_stop;
static bool g_flush_stuck;
static xcpt_t g_handler;
static void *g_irqarg;
static struct wdog_s *g_watchdog;
static unsigned int g_unbinds;
static unsigned int g_disconnects;
static uint32_t g_rxstatus[256];
static unsigned int g_rxhead;
static unsigned int g_rxtail;
static uint32_t g_rxwords[4096];
static unsigned int g_rxread;
static unsigned int g_rxwrite;
static uint32_t g_txwords[9][4096];
static unsigned int g_txcount[9];
static unsigned int g_txflush[64];
static unsigned int g_ntxflush;
static unsigned int g_nrxflush;

void arm_usbinitialize(void);
void arm_usbuninitialize(void);
static void test_dispatch(void);
static void test_tick(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void test_error(const char *format, ...)
{
  assert(format != NULL);
  g_errors++;
}

static uint32_t *test_register(uintptr_t address)
{
  assert((address & 3) == 0);
  if (address >= STM32_RCC_BASE && address < STM32_RCC_BASE + 0x1300)
    {
      return &g_rcc[(address - STM32_RCC_BASE) / 4];
    }

  if (address >= STM32_PWR_BASE && address < STM32_PWR_BASE + 0x80)
    {
      assert((g_rcc[STM32_RCC_AHB4ENR_OFFSET / 4] &
              RCC_AHB4ENR_PWREN) != 0);
      return &g_pwr[(address - STM32_PWR_BASE) / 4];
    }

  if (address >= STM32_USBPHYC_BASE && address < STM32_USBPHYC_BASE + 12)
    {
      assert((g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] & TEST_CLOCKS) ==
             TEST_CLOCKS);
      assert((g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] &
              STM32_OTG_RCC_PHYCTL_RST) == 0);
      return &g_phy[(address - STM32_USBPHYC_BASE) / 4];
    }

  assert(address >= STM32_OTG_BASE && address < STM32_OTG_BASE + 0x1000);
  assert((g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] & TEST_CLOCKS) == TEST_CLOCKS);
  assert((g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] & TEST_RESETS) == 0);
  return &g_core[(address - STM32_OTG_BASE) / 4];
}

static uint32_t getreg32(uintptr_t address)
{
  uint32_t *reg;
  unsigned int ep;

  g_reads++;
  if (address == STM32_OTG_DFIFO(0))
    {
      assert(g_rxread < g_rxwrite);
      return g_rxwords[g_rxread++];
    }

  reg = test_register(address);
  if (g_runtime && address == STM32_OTG_GRXSTSP)
    {
      uint32_t status;

      assert(g_rxhead < g_rxtail);
      status = g_rxstatus[g_rxhead++];
      if ((status & OTG_GRXST_PKTSTS_MASK) == OTG_GRXST_PKTSTS_GONAK)
        {
          g_core[STM32_OTG_GINTSTS_OFFSET / 4] |= OTG_GINT_GONAKEFF;
        }

      return status;
    }

  if (g_runtime && address == STM32_OTG_GINTSTS)
    {
      uint32_t status = *reg & ~(OTG_GINT_RXFLVL | OTG_GINT_IEPINT |
                                OTG_GINT_OEPINT);

      if (g_rxhead < g_rxtail)
        {
          status |= OTG_GINT_RXFLVL;
        }

      for (ep = 0; ep < 9; ep++)
        {
          if ((g_core[STM32_OTG_DAINTMSK_OFFSET / 4] &
               OTG_DAINT_IN(ep)) != 0 &&
              (g_core[STM32_OTG_DIEPINT_OFFSET(ep) / 4] &
               (g_core[STM32_OTG_DIEPMSK_OFFSET / 4] |
                ((g_core[STM32_OTG_DIEPEMPMSK_OFFSET / 4] &
                  (1u << ep)) != 0 ? OTG_DIEPINT_TXFE : 0))) != 0)
            {
              status |= OTG_GINT_IEPINT;
            }

          if ((g_core[STM32_OTG_DAINTMSK_OFFSET / 4] &
               OTG_DAINT_OUT(ep)) != 0 &&
              (g_core[STM32_OTG_DOEPINT_OFFSET(ep) / 4] &
               g_core[STM32_OTG_DOEPMSK_OFFSET / 4]) != 0)
            {
              status |= OTG_GINT_OEPINT;
            }
        }

      return status;
    }

  if (g_runtime && address == STM32_OTG_DAINT)
    {
      uint32_t status = 0;

      for (ep = 0; ep < 9; ep++)
        {
          if (g_core[STM32_OTG_DIEPINT_OFFSET(ep) / 4] &
              (OTG_DIEPINT_W1C_MASK | OTG_DIEPINT_TXFE))
            {
              status |= OTG_DAINT_IN(ep);
            }

          if (g_core[STM32_OTG_DOEPINT_OFFSET(ep) / 4] &
              OTG_DOEPINT_W1C_MASK)
            {
              status |= OTG_DAINT_OUT(ep);
            }
        }

      return status;
    }

  if (address == STM32_RCC_SR)
    {
      if ((g_rcc[STM32_RCC_CR_OFFSET / 4] & RCC_CR_HSEON) != 0)
        {
          if (g_failure != TEST_HSE_READY && g_time >= g_hse_due)
            {
              if ((*reg & RCC_SR_HSERDY) == 0)
                {
                  g_hse_ready = g_time;
                }

              *reg |= RCC_SR_HSERDY;
            }
        }
      else if (g_failure != TEST_HSE_STOP)
        {
          *reg &= ~RCC_SR_HSERDY;
        }
    }
  else if (address == STM32_PWR_SVMCR3)
    {
      if ((*reg & PWR_SVMCR3_USB33VMEN) != 0 &&
          g_failure != TEST_SUPPLY &&
          !(g_failure == TEST_PHY_SUPPLY && g_phy_released != 0 &&
            g_time >= g_phy_released + 50) &&
          !(g_failure == TEST_CORE_SUPPLY && g_time >= g_mode_due))
        {
          *reg |= PWR_SVMCR3_USB33RDY;
        }
      else
        {
          *reg &= ~PWR_SVMCR3_USB33RDY;
        }
    }
  else if (address == STM32_OTG_GRSTCTL && g_time >= g_reset_due)
    {
      if ((*reg & OTG_GRSTCTL_CSRST) != 0 &&
          g_failure != TEST_SOFT_RESET)
        {
          *reg &= ~OTG_GRSTCTL_CSRST;
          g_reset_done = g_time;
          g_soft_reset = true;
        }

      if (g_failure != TEST_TX_FLUSH && !g_flush_stuck)
        {
          *reg &= ~OTG_GRSTCTL_TXFFLSH;
        }

      if (g_failure != TEST_RX_FLUSH)
        {
          *reg &= ~OTG_GRSTCTL_RXFFLSH;
        }

      if (g_failure == TEST_IDLE ||
          (g_failure == TEST_RESET_IDLE && g_soft_reset))
        {
          *reg &= ~OTG_GRSTCTL_AHBIDL;
        }
    }
  else if (address == STM32_OTG_GINTSTS && g_time >= g_mode_due &&
           g_failure != TEST_MODE)
    {
      *reg &= ~OTG_GINT_CMOD;
      if (g_failure == TEST_NAK_EFFECTIVE)
        {
          *reg |= OTG_GINT_GINAKEFF | OTG_GINT_GONAKEFF;
        }
    }
  else if (!g_runtime && address == STM32_OTG_DCTL && g_time >= g_nak_due &&
           g_failure != TEST_NAK)
    {
      *reg |= OTG_DCTL_GINSTS | OTG_DCTL_GONSTS;
    }

  return *reg;
}

static void test_core_reset(void)
{
  unsigned int ep;

  memset(g_core, 0, sizeof(g_core));
  g_core[STM32_OTG_GUSBCFG_OFFSET / 4] = 0x1400;
  g_core[STM32_OTG_GINTSTS_OFFSET / 4] = 0x04000021;
  g_core[STM32_OTG_GRSTCTL_OFFSET / 4] = OTG_GRSTCTL_AHBIDL;
  g_core[STM32_OTG_GCCFG_OFFSET / 4] = 0x42;
  g_core[STM32_OTG_DCTL_OFFSET / 4] = OTG_DCTL_SDIS;
  g_core[STM32_OTG_DCFG_OFFSET / 4] = 0x02200000;
  if (g_failure == TEST_INTERRUPT_MASK)
    {
      g_core[STM32_OTG_GINTMSK_OFFSET / 4] = OTG_GINT_USBRST;
    }

  g_core[STM32_OTG_PCGCCTL_OFFSET / 4] = 0x200b8000;
  for (ep = 0; ep < 9; ep++)
    {
      g_core[STM32_OTG_DIEPINT_OFFSET(ep) / 4] = OTG_DIEPINT_TXFE;
      g_core[STM32_OTG_DOEPINT_OFFSET(ep) / 4] = 0x80;
    }

  g_core[STM32_OTG_DOEPCTL_OFFSET(0) / 4] = OTG_EPCTL_USBAEP;
  if (g_failure == TEST_ENDPOINT)
    {
      g_core[STM32_OTG_DIEPCTL_OFFSET(8) / 4] = OTG_EPCTL_EPENA;
    }

  g_soft_reset = false;
  g_mode_due = UINT32_MAX;
  g_nak_due = UINT32_MAX;
}

static void putreg32(uint32_t value, uintptr_t address)
{
  uint32_t *reg;

  if (address >= STM32_OTG_DFIFO(0) &&
      address <= STM32_OTG_DFIFO(8))
    {
      unsigned int ep = (address - STM32_OTG_DFIFO(0)) / 0x1000;

      assert(address == STM32_OTG_DFIFO(ep));
      assert(g_txcount[ep] < 4096);
      g_txwords[ep][g_txcount[ep]++] = value;
      g_writes++;
      return;
    }

  reg = test_register(address);

  g_writes++;
  if (address == STM32_RCC_AHB5RSTSR)
    {
      assert((value & ~TEST_RESETS) == 0);
      if (g_failure != TEST_RESET &&
          !(g_failure == TEST_CLEANUP_RESET && g_soft_reset))
        {
          g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] |= value;
          test_core_reset();
        }
    }
  else if (address == STM32_RCC_AHB5RSTCR)
    {
      assert((value & ~TEST_RESETS) == 0);
      if ((value == STM32_OTG_RCC_PHYCTL_RST &&
           g_failure == TEST_PHYCTL_RESET) ||
          (value == STM32_OTG_RCC_PHY_RST && g_failure == TEST_PHY_RESET) ||
          (value == STM32_OTG_RCC_RST && g_failure == TEST_CORE_RESET))
        {
          return;
        }

      if (value == STM32_OTG_RCC_PHYCTL_RST)
        {
          assert((g_rcc[STM32_RCC_SR_OFFSET / 4] & RCC_SR_HSERDY) != 0);
          assert(g_time >= g_hse_ready + BOARD_USB_HSE_STABILIZATION_US);
        }
      else if (value == STM32_OTG_RCC_PHY_RST)
        {
          assert((g_phy[0] & USBPHYC_CR_FSEL_MASK) ==
                 USBPHYC_CR_FSEL_24MHZ);
          g_phy_released = g_time;
        }
      else if (value == STM32_OTG_RCC_RST)
        {
          assert(g_time >= g_phy_released + 50);
        }

      g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] &= ~value;
    }
  else if (address == STM32_RCC_AHB5ENSR)
    {
      assert(value == TEST_CLOCKS);
      if (g_failure != TEST_CLOCK_ENABLE)
        {
          g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] |= value;
        }
    }
  else if (address == STM32_RCC_AHB5ENCR)
    {
      assert((value & ~TEST_CLOCKS) == 0);
      if (g_failure != TEST_CLOCK_CLEAR)
        {
          g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] &= ~value;
        }
    }
  else if (address == STM32_RCC_CSR)
    {
      assert(value == RCC_CR_HSEON);
      if (g_failure != TEST_HSE_ENABLE)
        {
          g_rcc[STM32_RCC_CR_OFFSET / 4] |= value;
          g_hse_due = g_time + 2;
        }
    }
  else if (address == STM32_RCC_CCR)
    {
      assert(value == RCC_CR_HSEON);
      g_rcc[STM32_RCC_CR_OFFSET / 4] &= ~value;
    }
  else if (address == STM32_RCC_HSECFGR)
    {
      assert((g_rcc[STM32_RCC_CR_OFFSET / 4] & RCC_CR_HSEON) == 0);
      if (g_failure != TEST_HSE_CONFIG)
        {
          *reg = value;
        }
    }
  else if (address == STM32_RCC_CCIPR6)
    {
      assert((value & ~STM32_OTG_RCC_CLKSEL_MASK) ==
             (*reg & ~STM32_OTG_RCC_CLKSEL_MASK));
      if (g_failure != TEST_CLKSEL)
        {
          *reg = value;
        }
    }
  else if (address == STM32_PWR_SVMCR3)
    {
      if ((value & PWR_SVMCR3_USB33SV) != 0)
        {
          assert((*reg & PWR_SVMCR3_USB33RDY) != 0);
        }

      if (g_failure != TEST_MONITOR_WRITE &&
          !(g_failure == TEST_VALID_WRITE &&
            (value & PWR_SVMCR3_USB33SV) != 0))
        {
          *reg = (value & ~PWR_SVMCR3_USB33RDY) |
                 (*reg & PWR_SVMCR3_USB33RDY);
        }
    }
  else if (address == STM32_USBPHYC_CR)
    {
      if (g_failure != TEST_PHY_CONFIG)
        {
          assert((value & ~USBPHYC_CR_FSEL_MASK) ==
                 (*reg & ~USBPHYC_CR_FSEL_MASK));
          *reg = value;
        }
    }
  else if (address == STM32_OTG_GRSTCTL)
    {
      assert((*reg & OTG_GRSTCTL_AHBIDL) != 0);
      if (g_runtime)
        {
          assert((*reg & (OTG_GRSTCTL_TXFFLSH |
                         OTG_GRSTCTL_RXFFLSH)) == 0);
          if ((value & OTG_GRSTCTL_TXFFLSH) != 0)
            {
              assert(g_ntxflush < 64);
              g_txflush[g_ntxflush++] =
                (value & OTG_GRSTCTL_TXFNUM_MASK) >>
                OTG_GRSTCTL_TXFNUM_SHIFT;
            }

          if ((value & OTG_GRSTCTL_RXFFLSH) != 0)
            {
              g_nrxflush++;
            }
        }

      *reg = value | OTG_GRSTCTL_AHBIDL;
      g_reset_due = g_time + (g_runtime ? 0 : 2);
    }
  else if (address == STM32_OTG_GUSBCFG)
    {
      assert(g_soft_reset && g_time >= g_reset_done + 3);
      *reg = value;
      g_mode_due = g_time + 25000;
    }
  else if (address == STM32_OTG_DCTL)
    {
      assert(g_runtime || (value & OTG_DCTL_SDIS) != 0);
      *reg = value & ~(OTG_DCTL_SGINAK | OTG_DCTL_SGONAK);
      if (g_runtime)
        {
          if ((value & OTG_DCTL_SGONAK) != 0 && !g_delayed_stop &&
              (*reg & OTG_DCTL_GONSTS) == 0)
            {
              assert(g_rxtail < 256);
              g_rxstatus[g_rxtail++] = OTG_GRXST_PKTSTS_GONAK;
              *reg |= OTG_DCTL_GONSTS;
            }

          if ((value & OTG_DCTL_CGONAK) != 0)
            {
              *reg &= ~OTG_DCTL_GONSTS;
              g_core[STM32_OTG_GINTSTS_OFFSET / 4] &=
                ~OTG_GINT_GONAKEFF;
            }
        }

      if ((value & (OTG_DCTL_SGINAK | OTG_DCTL_SGONAK)) != 0)
        {
          g_nak_due = g_time + 2;
        }
    }
  else if (address == STM32_OTG_GCCFG)
    {
      assert(g_runtime || (value & OTG_GCCFG_VBVALOVAL) == 0);
      *reg = value;
    }
  else if (address == STM32_OTG_GINTSTS ||
           address == STM32_OTG_GOTGINT)
    {
      uint32_t mask = address == STM32_OTG_GINTSTS ?
                      OTG_GINTSTS_W1C_MASK : OTG_GOTGINT_W1C_MASK;

      assert((value & ~mask) == 0);
      *reg &= ~value;
    }
  else if (address >= STM32_OTG_BASE + 0x900 &&
           address <= STM32_OTG_BASE + 0xc08 &&
           (address & 0x1f) == 8)
    {
      assert((value & ~(address < STM32_OTG_BASE + 0xb00 ?
                       OTG_DIEPINT_W1C_MASK : OTG_DOEPINT_W1C_MASK)) == 0);
      *reg &= ~value;
    }
  else if (g_runtime && address >= STM32_OTG_DIEPCTL(0) &&
           address <= STM32_OTG_DOEPCTL(8) &&
           ((address - STM32_OTG_BASE) & 0x1f) == 0)
    {
      bool in = address < STM32_OTG_DOEPCTL(0);
      unsigned int ep = (address - (in ? STM32_OTG_DIEPCTL(0) :
                                   STM32_OTG_DOEPCTL(0))) / 32;
      uint32_t *interrupt = &g_core[(in ? STM32_OTG_DIEPINT_OFFSET(ep) :
                                   STM32_OTG_DOEPINT_OFFSET(ep)) / 4];

      *reg = value;
      if ((value & OTG_EPCTL_CNAK) != 0)
        {
          *reg &= ~OTG_EPCTL_NAKSTS;
          if (in)
            {
              *interrupt &= ~OTG_DIEPINT_INEPNE;
            }
        }

      if ((value & OTG_EPCTL_SNAK) != 0 && !g_delayed_stop)
        {
          *reg |= OTG_EPCTL_NAKSTS;
          if (in)
            {
              *interrupt |= OTG_DIEPINT_INEPNE;
            }
        }

      if ((value & OTG_EPCTL_EPDIS) != 0 && !g_delayed_stop)
        {
          if (!in)
            {
              assert(g_core[STM32_OTG_GINTSTS_OFFSET / 4] &
                     OTG_GINT_GONAKEFF);
            }

          *reg &= ~(OTG_EPCTL_EPENA | OTG_EPCTL_EPDIS);
          *interrupt |= in ? OTG_DIEPINT_EPDISD : OTG_DOEPINT_EPDISD;
        }

      *reg &= ~(OTG_EPCTL_CNAK | OTG_EPCTL_SNAK);
    }
  else if ((address == STM32_OTG_GRXFSIZ && g_failure == TEST_RX_SIZE) ||
           (address == STM32_OTG_DIEPTXF(8) && g_failure == TEST_TX_SIZE) ||
           (address == STM32_OTG_GINTMSK &&
            g_failure == TEST_INTERRUPT_MASK))
    {
      return;
    }
  else
    {
      assert(address >= STM32_OTG_BASE &&
             address < STM32_OTG_BASE + 0x1000);
      *reg = value;
    }
}

static void modifyreg32(uintptr_t address, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(address) & ~clear) | set, address);
}

static irqstate_t enter_critical_section(void)
{
  return g_critical++;
}

static void leave_critical_section(irqstate_t flags)
{
  assert(g_critical == flags + 1);
  g_critical--;
}

static uint32_t getcontrol(void)
{
  return g_control;
}

static uint32_t getipsr(void)
{
  return g_ipsr;
}

static void up_disable_irq(int irq)
{
  assert(irq == STM32_IRQ_OTG);
  g_irqdisables++;
  g_irqenabled = false;
}

static void up_enable_irq(int irq)
{
  assert(irq == STM32_IRQ_OTG && g_handler != NULL);
  g_irqenabled = true;
}

static int irq_attach(int irq, xcpt_t handler, void *arg)
{
  assert(irq == STM32_IRQ_OTG);
  g_handler = handler;
  g_irqarg = arg;
  return OK;
}

static int irq_detach(int irq)
{
  assert(irq == STM32_IRQ_OTG);
  g_handler = NULL;
  g_irqarg = NULL;
  return OK;
}

static clock_t clock_systime_ticks(void)
{
  return g_time / 1000;
}

static int wd_start(struct wdog_s *wdog, clock_t delay,
                    wdentry_t callback, uintptr_t arg)
{
  assert(delay > 0 && callback != NULL);
  wdog->callback = callback;
  wdog->arg = arg;
  wdog->due = (unsigned int)clock_systime_ticks() + (unsigned int)delay;
  wdog->active = true;
  g_watchdog = wdog;
  return OK;
}

static int wd_cancel(struct wdog_s *wdog)
{
  wdog->active = false;
  return OK;
}

static int nxsem_post(sem_t *sem)
{
  assert(sem != NULL);
  sem->count++;
  return OK;
}

static int nxsem_wait_uninterruptible(sem_t *sem)
{
  unsigned int turns = 0;

  assert(sem != NULL && g_critical == 0);
  while (sem->count == 0)
    {
      assert(turns++ < 1000);
      if (g_irqenabled)
        {
          test_dispatch();
        }

      test_tick();
    }

  sem->count--;
  return OK;
}

static void up_udelay(unsigned int delay)
{
  assert(g_critical == 0);
  assert(!g_runtime);
  g_time += delay;
  if ((g_failure == TEST_HSE_SETTLE &&
       delay == BOARD_USB_HSE_STABILIZATION_US) ||
      (g_failure == TEST_CORE_HSE && delay == 25000))
    {
      g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] |= RCC_HSECFGR_HSECSSD;
    }

  if (g_reenter)
    {
      g_reenter = false;
      arm_usbinitialize();
      arm_usbuninitialize();
    }
}

/* USB_DRIVER */

#ifdef CONFIG_USBDEV_TRACE
void usbtrace(uint16_t event, uint16_t value)
{
  (void)event;
  (void)value;
}
#endif

static int test_bind(struct usbdevclass_driver_s *driver,
                     struct usbdev_s *dev)
{
  (void)driver;
  (void)dev;
  g_binds++;
  return 0;
}

static void test_unbind(struct usbdevclass_driver_s *driver,
                        struct usbdev_s *dev)
{
  assert(driver != NULL && dev != NULL);
  g_unbinds++;
}

static void test_disconnect(struct usbdevclass_driver_s *driver,
                            struct usbdev_s *dev)
{
  assert(driver != NULL && dev != NULL);
  g_disconnects++;
}

static uint8_t g_control_data[4096];
static size_t g_control_length;
static unsigned int g_setups;
static unsigned int g_completions;
static struct usbdev_req_s *g_control_req;

static void test_complete(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  unsigned int *count = req->priv;

  assert(ep != NULL && count != NULL);
  (*count)++;
}

static int test_setup(struct usbdevclass_driver_s *driver,
                      struct usbdev_s *dev, const struct usb_ctrlreq_s *ctrl,
                      uint8_t *dataout, size_t outlen)
{
  assert(driver != NULL && dev != NULL && ctrl != NULL);
  g_setups++;
  if ((ctrl->type & USB_DIR_IN) != 0)
    {
      assert(g_control_req == NULL);
      g_control_req = dev->ep0->ops->allocreq(dev->ep0);
      assert(g_control_req != NULL);
      g_control_req->buf = g_control_data;
      g_control_req->len = g_control_length;
      g_control_req->callback = test_complete;
      g_control_req->priv = &g_completions;
      return EP_SUBMIT(dev->ep0, g_control_req);
    }

  assert(outlen <= sizeof(g_control_data));
  g_control_length = outlen;
  if (outlen != 0)
    {
      memcpy(g_control_data, dataout, outlen);
    }

  return OK;
}

static void test_fixture(enum test_failure_e failure)
{
  memset(&g_otgdev, 0, sizeof(g_otgdev));
  g_otgdev.initresult = -ENODEV;
  memset(g_rcc, 0, sizeof(g_rcc));
  memset(g_pwr, 0, sizeof(g_pwr));
  g_failure = failure;
  g_time = 0;
  g_reads = 0;
  g_writes = 0;
  g_irqdisables = 0;
  g_critical = 0;
  g_control = 0;
  g_ipsr = 0;
  g_errors = 0;
  g_reenter = false;
  g_runtime = false;
  g_irqenabled = false;
  g_handler = NULL;
  g_irqarg = NULL;
  g_watchdog = NULL;
  g_delayed_stop = false;
  g_flush_stuck = false;
  g_binds = 0;
  g_unbinds = 0;
  g_disconnects = 0;
  g_setups = 0;
  g_completions = 0;
  g_control_req = NULL;
  g_control_length = 0;
  g_rxhead = g_rxtail = 0;
  g_rxread = g_rxwrite = 0;
  memset(g_txcount, 0, sizeof(g_txcount));
  g_ntxflush = 0;
  g_nrxflush = 0;
  g_hse_ready = 0;
  g_phy_released = 0;
  g_rcc[STM32_RCC_AHB4ENR_OFFSET / 4] = RCC_AHB4ENR_PWREN;
  g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] = 1;
  g_rcc[STM32_RCC_CCIPR6_OFFSET / 4] = 0x321;
  g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] = 0x800;
  g_pwr[STM32_PWR_SVMCR3_OFFSET / 4] = 0x01000300;
  g_phy[0] = USBPHYC_CR_RESET;
  g_phy[1] = 0x123;
  g_phy[2] = 0x456;
  if (failure == TEST_HSE_STOP)
    {
      g_rcc[STM32_RCC_SR_OFFSET / 4] = RCC_SR_HSERDY;
    }

  test_core_reset();
}

static void test_initialized(void)
{
#ifdef TEST_USB_CUSTOM_FIFO
  const uint16_t depths[9] =
  {
    16, 17, 128, 16, 16, 16, 16, 16, 32
  };

  const unsigned int total = 785;
#else
  const uint16_t depths[9] =
  {
    64, 16, 256, 16, 16, 16, 16, 16, 16
  };

  const unsigned int total = 944;
#endif
  unsigned int ep;
  unsigned int start = 512;
  uint32_t fifo;

  assert(g_otgdev.state == USBSTATE_READY);
  assert(g_otgdev.initresult == OK);
  assert(g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS);
  assert((g_core[STM32_OTG_GCCFG_OFFSET / 4] &
          OTG_GCCFG_VBVALOVAL) == 0);
  assert(g_core[STM32_OTG_GAHBCFG_OFFSET / 4] == 0);
  assert(g_core[STM32_OTG_GINTMSK_OFFSET / 4] == 0);
  assert(g_core[STM32_OTG_DAINTMSK_OFFSET / 4] == 0);
  assert(g_core[STM32_OTG_GRXFSIZ_OFFSET / 4] == 512);
  assert((g_core[STM32_OTG_GUSBCFG_OFFSET / 4] &
          OTG_GUSBCFG_TRDT_MASK) == (9u << 10));
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] &
          OTG_DCFG_DAD_MASK) == 0);
#ifdef CONFIG_STM32_N6_OTGDEV_FS
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] & 3) == 1);
#else
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] & 3) == 0);
#endif
  assert(g_core[STM32_OTG_PCGCCTL_OFFSET / 4] == 0x200b8000);
  assert(g_core[STM32_OTG_GCCFG_OFFSET / 4] == 0x42);
  assert(g_phy[1] == 0x123 && g_phy[2] == 0x456);
  assert((g_rcc[STM32_RCC_CCIPR6_OFFSET / 4] &
          STM32_OTG_RCC_CLKSEL_MASK) == STM32_OTG_RCC_CLKSEL);
  assert((g_rcc[STM32_RCC_CCIPR6_OFFSET / 4] &
          ~STM32_OTG_RCC_CLKSEL_MASK) == 0x321);
  for (ep = 0; ep < 9; ep++)
    {
      fifo = g_core[(ep == 0 ? STM32_OTG_DIEPTXF0_OFFSET :
                     STM32_OTG_DIEPTXF_OFFSET(ep)) / 4];
      assert((fifo & 0xffff) == start);
      assert((fifo >> 16) == depths[ep]);
      start += fifo >> 16;
      assert(g_core[STM32_OTG_DIEPINT_OFFSET(ep) / 4] ==
             OTG_DIEPINT_TXFE);
      assert(g_core[STM32_OTG_DOEPINT_OFFSET(ep) / 4] == 0x80);
      assert((g_core[STM32_OTG_DIEPCTL_OFFSET(ep) / 4] &
              OTG_EPCTL_EPENA) == 0);
      assert((g_core[STM32_OTG_DOEPCTL_OFFSET(ep) / 4] &
              OTG_EPCTL_EPENA) == 0);
    }

  assert(start == STM32_USB_FIFO_TOTAL && start <= 1024);
  assert(start == total);
  assert(g_time >= BOARD_USB_HSE_STABILIZATION_US + 25000 + 50);
  assert(g_irqdisables != 0);
  assert(g_otgdev.usbdev.ops != NULL && g_otgdev.usbdev.ep0 != NULL);
  assert(g_handler != NULL && !g_irqenabled);
  assert(g_binds == 0);
}

static void test_lifecycle(struct usbdevclass_driver_s *driver)
{
  unsigned int before;
  unsigned int fault;
  int ret;

  test_fixture(TEST_NONE);
  assert(usbdev_register(driver) == -ENODEV);
  arm_usbinitialize();
  test_initialized();
  before = g_writes;
  arm_usbinitialize();
  assert(g_writes == before);
  g_runtime = true;
  assert(usbdev_register(driver) == OK);
  assert(g_binds == 1 && g_irqenabled);
  assert(g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS);
  assert(usbdev_unregister(driver) == OK);
  assert(g_unbinds == 1);
  g_runtime = false;
  arm_usbuninitialize();
  assert(g_otgdev.state == USBSTATE_OFF &&
         g_otgdev.initresult == -ENODEV);
  assert(g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] == 1);
  assert((g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] & TEST_RESETS) ==
         TEST_RESETS);
  assert(g_rcc[STM32_RCC_CCIPR6_OFFSET / 4] == 0x321);
  assert((g_pwr[STM32_PWR_SVMCR3_OFFSET / 4] &
          ~PWR_SVMCR3_USB33RDY) == 0x01000300);
  assert(g_rcc[STM32_RCC_CR_OFFSET / 4] & RCC_CR_HSEON);
  before = g_writes;
  arm_usbuninitialize();
  assert(g_writes == before);
  arm_usbinitialize();
  g_binds = 0;
  test_initialized();

  for (fault = TEST_RESET; fault <= TEST_RX_FLUSH; fault++)
    {
      test_fixture(fault);
      arm_usbinitialize();
      ret = g_otgdev.initresult;
      assert(ret < 0 && ret != -ENOSYS);
      assert(g_otgdev.state != USBSTATE_READY);
      assert(usbdev_register(driver) == ret);
      assert(g_time <= 250000);
      assert(g_errors != 0 && g_critical == 0);
      assert(g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] == 1);
      assert((g_pwr[STM32_PWR_SVMCR3_OFFSET / 4] &
              ~PWR_SVMCR3_USB33RDY) == 0x01000300);
      g_failure = TEST_NONE;
      g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] &= ~RCC_HSECFGR_HSECSSD;
      arm_usbinitialize();
      test_initialized();
    }

  test_fixture(TEST_NONE);
  g_control = CONTROL_NPRIV;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EACCES);
  assert(g_reads == 0 && g_writes == 0 && g_irqdisables == 0);
  test_fixture(TEST_NONE);
  g_ipsr = 16;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EWOULDBLOCK);
  assert(g_reads == 0 && g_writes == 0 && g_irqdisables == 0);

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_AHB4ENR_OFFSET / 4] = 0;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -ENODEV && g_writes == 0);

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_LOCKCFGR0_OFFSET / 4] = RCC_LOCKCFGR0_HSELOCK;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EACCES);
  assert(g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] == 0x800);
  assert(g_rcc[STM32_RCC_CR_OFFSET / 4] == 0);

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_CR_OFFSET / 4] = RCC_CR_HSEON;
  g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] |= RCC_HSECFGR_HSEBYP;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EBUSY);
  assert(g_rcc[STM32_RCC_CR_OFFSET / 4] == RCC_CR_HSEON);
  assert(g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] ==
         (0x800 | RCC_HSECFGR_HSEBYP));

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] |= RCC_HSECFGR_HSECSSD;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EIO);
  assert(g_rcc[STM32_RCC_CR_OFFSET / 4] == 0);

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_CFGR1_OFFSET / 4] = RCC_CFGR1_CPUSWS_HSE;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EBUSY);
  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_CFGR1_OFFSET / 4] = RCC_CFGR1_SYSSW_HSE;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EBUSY);
  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_CR_OFFSET / 4] = RCC_CR_PLL1ON << 3;
  g_rcc[STM32_RCC_PLLCFGR1_OFFSET(4) / 4] = RCC_PLL1CFGR1_SEL_HSE;
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EBUSY);

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] |= TEST_CLOCKS |
                                      STM32_OTG_RCC_OTHER_EN;
  g_rcc[STM32_RCC_CR_OFFSET / 4] = RCC_CR_HSEON;
  g_rcc[STM32_RCC_SR_OFFSET / 4] = RCC_SR_HSERDY;
  g_rcc[STM32_RCC_HSECFGR_OFFSET / 4] |= RCC_HSECFGR_HSEDIV2SEL;
  g_rcc[STM32_RCC_LOCKCFGR0_OFFSET / 4] = RCC_LOCKCFGR0_HSELOCK;
  g_pwr[STM32_PWR_SVMCR3_OFFSET / 4] |= PWR_SVMCR3_USB33VMEN |
                                     PWR_SVMCR3_USB33SV;
  arm_usbinitialize();
  test_initialized();
  assert(g_otgdev.clocks_added == 0 && g_otgdev.pwr_added == 0);
  arm_usbuninitialize();
  assert(g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] ==
         (1 | TEST_CLOCKS | STM32_OTG_RCC_OTHER_EN));
  assert(g_pwr[STM32_PWR_SVMCR3_OFFSET / 4] & PWR_SVMCR3_USB33SV);

  test_fixture(TEST_NONE);
  g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] |= STM32_OTG_RCC_OTHER_EN;
  arm_usbinitialize();
  arm_usbuninitialize();
  assert(g_rcc[STM32_RCC_AHB5ENR_OFFSET / 4] ==
         (1 | STM32_OTG_RCC_OTHER_EN));
  assert(g_pwr[STM32_PWR_SVMCR3_OFFSET / 4] & PWR_SVMCR3_USB33SV);

  test_fixture(TEST_NONE);
  g_reenter = true;
  arm_usbinitialize();
  test_initialized();
  assert(g_errors == 2);

  test_fixture(TEST_NONE);
  arm_usbinitialize();
  g_rcc[STM32_RCC_AHB4ENR_OFFSET / 4] = 0;
  arm_usbuninitialize();
  assert(g_otgdev.initresult == -ENODEV && g_otgdev.pwr_added != 0);
  assert((g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] & TEST_RESETS) ==
         TEST_RESETS);
  g_rcc[STM32_RCC_AHB4ENR_OFFSET / 4] = RCC_AHB4ENR_PWREN;
  arm_usbinitialize();
  test_initialized();

  for (fault = TEST_CLOCK_CLEAR; fault <= TEST_CLEANUP_RESET; fault++)
    {
      test_fixture(TEST_NONE);
      arm_usbinitialize();
      g_failure = fault;
      arm_usbuninitialize();
      assert(g_otgdev.state != USBSTATE_READY &&
             g_otgdev.initresult == -EACCES);
      assert((g_otgdev.resources & USBRES_RESETS) != 0);
      assert(g_errors != 0);
      g_failure = TEST_NONE;
      arm_usbinitialize();
      test_initialized();
    }
}

/****************************************************************************
 * FIFO-mode device API tests
 ****************************************************************************/

static void test_dispatch(void)
{
  unsigned int reads = g_reads;
  unsigned int time = g_time;

  assert(g_handler != NULL && g_irqenabled);
  assert(g_handler(STM32_IRQ_OTG, NULL, g_irqarg) == OK);
  assert(g_time == time);
  assert(g_reads - reads < 1000);
}

static void test_tick(void)
{
  unsigned int reads = g_reads;

  g_time += 1000;
  if (g_watchdog != NULL && g_watchdog->active &&
      (unsigned int)clock_systime_ticks() >= g_watchdog->due)
    {
      wdentry_t callback = g_watchdog->callback;
      uintptr_t arg = g_watchdog->arg;
      unsigned int time = g_time;

      g_watchdog->active = false;
      callback(arg);
      assert(g_time == time && g_reads - reads < 1000);
    }
}

static void test_pump(void)
{
  unsigned int i;

  for (i = 0; i < 12; i++)
    {
      if (!g_irqenabled)
        {
          break;
        }

      test_dispatch();
      test_tick();
    }
}

static void test_event(uint32_t event)
{
  g_core[STM32_OTG_GINTSTS_OFFSET / 4] |= event;
  test_dispatch();
}

static void test_pending_xfrc(struct usbdev_ep_s *ep)
{
  unsigned int number = ep->eplog & USB_EPNO_MASK;
  bool in = (ep->eplog & USB_DIR_IN) != 0 || number == 0;

  g_core[(in ? STM32_OTG_DIEPCTL_OFFSET(number) :
          STM32_OTG_DOEPCTL_OFFSET(number)) / 4] &= ~OTG_EPCTL_EPENA;
  g_core[(in ? STM32_OTG_DIEPINT_OFFSET(number) :
          STM32_OTG_DOEPINT_OFFSET(number)) / 4] |= OTG_DIEPINT_XFRC;
}

static void test_xfrc(struct usbdev_ep_s *ep)
{
  test_pending_xfrc(ep);
  test_dispatch();
}

static void test_receive(unsigned int ep, uint32_t kind,
                          const uint8_t *data, size_t length)
{
  size_t offset;

  assert(g_rxtail < 256 && length <= 1024);
  if (kind == OTG_GRXST_PKTSTS_SETUPRECVD && ep == 0)
    {
      g_core[STM32_OTG_DIEPCTL_OFFSET(0) / 4] &= ~OTG_EPCTL_STALL;
      g_core[STM32_OTG_DOEPCTL_OFFSET(0) / 4] &= ~OTG_EPCTL_STALL;
      g_core[STM32_OTG_DIEPCTL_OFFSET(0) / 4] |= OTG_EPCTL_DPID;
      g_core[STM32_OTG_DOEPCTL_OFFSET(0) / 4] |= OTG_EPCTL_DPID;
    }

  g_rxstatus[g_rxtail++] = ep | ((uint32_t)length << 4) | kind;
  for (offset = 0; offset < length; offset += 4)
    {
      uint32_t word = 0;
      size_t byte;

      for (byte = 0; byte < 4 && offset + byte < length; byte++)
        {
          word |= (uint32_t)data[offset + byte] << (8 * byte);
        }

      assert(g_rxwrite < 4096);
      g_rxwords[g_rxwrite++] = word;
    }

  test_dispatch();
  assert(g_rxread == g_rxwrite);
}

static struct usbdev_s *test_online(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = &g_otgdev.usbdev;
  unsigned int ep;

  test_fixture(TEST_NONE);
  arm_usbinitialize();
  test_initialized();
  g_runtime = true;
  assert(usbdev_register(driver) == OK);
  for (ep = 0; ep < 9; ep++)
    {
      g_core[STM32_OTG_DTXFSTS_OFFSET(ep) / 4] = 1024;
    }

  assert((DEV_CONNECT(dev)) == -ENOTCONN);
  assert(g_errors != 0);
  assert(g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS);
  assert(stm32_usbdev_vbus(true) == OK);
  assert((g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS) == 0);
  test_event(OTG_GINT_USBRST);
  test_pump();
#ifdef CONFIG_STM32_N6_OTGDEV_FS
  g_core[STM32_OTG_DSTS_OFFSET / 4] = OTG_DSTS_ENUMSPD_FS;
#else
  g_core[STM32_OTG_DSTS_OFFSET / 4] = OTG_DSTS_ENUMSPD_HS;
#endif
  test_event(OTG_GINT_ENUMDNE);
  assert(g_otgdev.state == USBSTATE_ENUMERATED);
  return dev;
}

static void test_offline(struct usbdevclass_driver_s *driver)
{
  assert(usbdev_unregister(driver) == OK);
  g_runtime = false;
  arm_usbuninitialize();
  assert(g_otgdev.state == USBSTATE_OFF);
}

static int test_configure(struct usbdev_ep_s *ep, unsigned int type,
                           unsigned int mps)
{
  struct usb_epdesc_s desc =
  {
    .len = USB_SIZEOF_EPDESC,
    .type = USB_DESC_TYPE_ENDPOINT,
    .addr = ep->eplog,
    .attr = type,
    .mxpacketsize = {(uint8_t)mps, (uint8_t)(mps >> 8)},
    .interval = 1
  };

  assert(EP_DISABLE(ep) == OK);
  test_pump();
  return EP_CONFIGURE(ep, &desc, true);
}

static struct usbdev_req_s *test_request(struct usbdev_ep_s *ep,
                                         uint8_t *buffer, size_t length,
                                         unsigned int *completions)
{
  struct usbdev_req_s *req = ep->ops->allocreq(ep);

  assert(req != NULL);
  req->buf = buffer;
  req->len = length;
  req->callback = test_complete;
  req->priv = completions;
  return req;
}

static void test_fifo_bytes(unsigned int ep, unsigned int start,
                             const uint8_t *data, size_t length)
{
  size_t byte;

  for (byte = 0; byte < length; byte++)
    {
      assert(((g_txwords[ep][start + byte / 4] >>
               (8 * (byte % 4))) & 255) == data[byte]);
    }

  if ((length & 3) != 0)
    {
      assert((g_txwords[ep][start + length / 4] >>
              (8 * (length & 3))) == 0);
    }
}

static void test_allocations(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *in[9] =
  {
    NULL
  };

  struct usbdev_ep_s *out[9] =
  {
    NULL
  };

  unsigned int ep;

  assert(DEV_ALLOCEP(dev, 9, true, USB_EP_ATTR_XFER_BULK) == NULL);
  assert(DEV_ALLOCEP(dev, 1, true, USB_EP_ATTR_XFER_ISOC) == NULL);
  for (ep = 1; ep < 9; ep++)
    {
      bool enabled = ep <= 2;

#ifdef TEST_USB_CUSTOM_FIFO
      enabled |= ep == 8;
#endif
      in[ep] = DEV_ALLOCEP(dev, ep, true, ep == 2 ?
                          USB_EP_ATTR_XFER_BULK : USB_EP_ATTR_XFER_INT);
      assert((in[ep] != NULL) == enabled);
      out[ep] = DEV_ALLOCEP(dev, ep, false, USB_EP_ATTR_XFER_BULK);
      assert(out[ep] != NULL && out[ep] != in[ep]);
      assert(DEV_ALLOCEP(dev, ep, false, USB_EP_ATTR_XFER_BULK) == NULL);
      if (in[ep] != NULL)
        {
          unsigned int type = ep == 2 ? USB_EP_ATTR_XFER_BULK :
                                       USB_EP_ATTR_XFER_INT;
          unsigned int mps = 64;

#ifndef CONFIG_STM32_N6_OTGDEV_FS
          if (ep == 2)
            {
              mps = 512;
            }

#endif
          assert(test_configure(in[ep], type, 0) < 0);
          assert(test_configure(in[ep], type, mps) == OK);
        }
    }

#ifdef CONFIG_STM32_N6_OTGDEV_FS
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 63) < 0);
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 8) == OK);
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 16) == OK);
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 32) == OK);
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 64) == OK);
  assert(test_configure(in[1], USB_EP_ATTR_XFER_INT, 65) < 0);
#else
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 64) < 0);
  assert(test_configure(out[1], USB_EP_ATTR_XFER_BULK, 512) == OK);
  assert(test_configure(in[1], USB_EP_ATTR_XFER_INT, 512) < 0);
  assert(test_configure(in[2], USB_EP_ATTR_XFER_BULK, 512) == OK);
  assert(test_configure(out[2], USB_EP_ATTR_XFER_INT, 1025) < 0);
#endif
  {
    struct usb_epdesc_s invalid =
    {
      .len = USB_SIZEOF_EPDESC,
      .type = USB_DESC_TYPE_ENDPOINT,
      .addr = 2,
      .attr = USB_EP_ATTR_XFER_INT,
      .mxpacketsize = {64, 0},
      .interval = 1
    };

    assert(EP_CONFIGURE(in[1], &invalid, true) < 0);
    invalid.addr = in[1]->eplog;
    invalid.attr = USB_EP_ATTR_XFER_ISOC;
    assert(EP_CONFIGURE(in[1], &invalid, true) < 0);
  }

  for (ep = 1; ep < 9; ep++)
    {
      if (in[ep] != NULL)
        {
          DEV_FREEEP(dev, in[ep]);
        }

      DEV_FREEEP(dev, out[ep]);
    }

  test_pump();
  test_offline(driver);
}

static void test_transfers(struct usbdevclass_driver_s *driver)
{
  static const size_t lengths[] =
  {
    1, 2, 3, 4, 63, 64, 65, 127, 129
  };

  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *in = DEV_ALLOCEP(dev, 1, true, USB_EP_ATTR_XFER_INT);
  struct usbdev_ep_s *out = DEV_ALLOCEP(dev, 1, false, USB_EP_ATTR_XFER_INT);
  uint8_t bytes[260];
  uint8_t received[260];
  unsigned int count;
  size_t i;

  assert(in != NULL && out != NULL);
  assert(test_configure(in, USB_EP_ATTR_XFER_INT, 64) == OK);
  assert(test_configure(out, USB_EP_ATTR_XFER_INT, 64) == OK);
  for (i = 0; i < sizeof(bytes); i++)
    {
      bytes[i] = (uint8_t)(i * 7 + 3);
    }

  for (i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++)
    {
      struct usbdev_req_s *req;
      size_t sent = 0;

      count = 0;
      g_txcount[1] = 0;
      req = test_request(in, bytes + 1, lengths[i], &count);
      assert(EP_SUBMIT(in, req) == OK);
      assert(req->xfrd == 0 && count == 0);
      while (sent < lengths[i])
        {
          size_t packet = lengths[i] - sent;
          unsigned int start = (unsigned int)((sent + 3) / 4);

          if (packet > 64)
            {
              packet = 64;
            }

          test_dispatch();
          assert(req->xfrd == sent);
          test_fifo_bytes(1, start, bytes + 1 + sent, packet);
          assert(g_txcount[1] == start + (packet + 3) / 4);
          test_xfrc(in);
          sent += packet;
          assert(req->xfrd == sent);
        }

      assert(count == 1 && req->result == OK);
      in->ops->freereq(in, req);

      count = 0;
      memset(received, 0xa5, sizeof(received));
      req = test_request(out, received + 1, lengths[i], &count);
      assert(EP_SUBMIT(out, req) == OK);
      sent = 0;
      while (sent < lengths[i])
        {
          size_t packet = lengths[i] - sent;

          if (packet > 64)
            {
              packet = 64;
            }

          test_receive(1, OTG_GRXST_PKTSTS_OUTRECVD, bytes + 1 + sent,
                       packet);
          test_xfrc(out);
          sent += packet;
        }

      assert(count == 1 && req->result == OK && req->xfrd == lengths[i]);
      assert(memcmp(received + 1, bytes + 1, lengths[i]) == 0);
      assert(received[0] == 0xa5 && received[lengths[i] + 1] == 0xa5);
      out->ops->freereq(out, req);
    }

  {
    struct usbdev_req_s *req;

    count = 0;
    req = test_request(in, bytes + 1, 64, &count);
    req->flags = USBDEV_REQFLAGS_NULLPKT;
    assert(EP_SUBMIT(in, req) == OK);
    test_xfrc(in);
    assert(req->xfrd == 64 && count == 0);
    assert((g_core[STM32_OTG_DIEPTSIZ_OFFSET(1) / 4] &
            OTG_EPTSIZ_XFRSIZ_MASK) == 0);
    test_xfrc(in);
    assert(count == 1 && req->result == OK);
    in->ops->freereq(in, req);
  }

  {
    struct usbdev_req_s *req;
    unsigned int errors = g_errors;

    count = 0;
    req = test_request(in, NULL, 1, &count);
    assert(EP_SUBMIT(in, req) < 0);
    assert(count == 0 && g_errors > errors);
    req->len = 0;
    assert(EP_SUBMIT(in, req) == OK);
    test_xfrc(in);
    assert(count == 1 && req->result == OK && req->xfrd == 0);
    in->ops->freereq(in, req);
  }

  for (i = 0; i < 3; i++)
    {
      struct usbdev_req_s *req;
      size_t packet = i == 0 ? 0 : i == 1 ? 3 : 9;

      count = 0;
      memset(received, 0xa5, sizeof(received));
      req = test_request(out, received + 1, i == 2 ? 3 : 128, &count);
      assert(EP_SUBMIT(out, req) == OK);
      test_receive(1, OTG_GRXST_PKTSTS_OUTRECVD, bytes + 1, packet);
      test_xfrc(out);
      test_pump();
      assert(count == 1);
      assert(req->result == (i == 2 ? -EOVERFLOW : OK));
      assert(req->xfrd == (i == 2 ? 3 : packet));
      assert(received[0] == 0xa5 &&
             received[(i == 2 ? 3 : 128) + 1] == 0xa5);
      out->ops->freereq(out, req);
    }

  test_offline(driver);
}

static void test_out_cancel(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *out = DEV_ALLOCEP(dev, 1, false,
                                       USB_EP_ATTR_XFER_INT);
  struct usbdev_ep_s *sibling = DEV_ALLOCEP(dev, 2, false,
                                           USB_EP_ATTR_XFER_INT);
  struct usbdev_req_s *req[3];
  uint8_t buffers[3][130];
  uint8_t payload[64];
  unsigned int counts[3] =
  {
    0
  };

  unsigned int i;
  unsigned int flushes;

  assert(test_configure(out, USB_EP_ATTR_XFER_INT, 64) == OK);
  assert(test_configure(sibling, USB_EP_ATTR_XFER_INT, 64) == OK);
  memset(buffers, 0xa5, sizeof(buffers));
  memset(payload, 0x5a, sizeof(payload));
  req[0] = test_request(out, buffers[0] + 1, 128, &counts[0]);
  req[1] = test_request(out, buffers[1] + 1, 128, &counts[1]);
  req[2] = test_request(sibling, buffers[2] + 1, 128, &counts[2]);
  assert(EP_SUBMIT(out, req[0]) == OK);
  assert(EP_SUBMIT(out, req[1]) == OK);
  assert(EP_SUBMIT(sibling, req[2]) == OK);
  assert(EP_CANCEL(out, req[1]) == OK);
  assert(counts[1] == 1 && req[1]->result == -ECONNRESET);
  flushes = g_nrxflush;
  assert(EP_CANCEL(out, req[0]) == OK);
  test_pump();
  assert(counts[0] == 1 && counts[2] == 0);
  assert(req[0]->result == -ECONNRESET);
  assert(g_nrxflush == flushes);
  assert((g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_GONSTS) == 0);
  assert((g_core[STM32_OTG_GRSTCTL_OFFSET / 4] &
          OTG_GRSTCTL_RXFFLSH) == 0);
  test_receive(2, OTG_GRXST_PKTSTS_OUTRECVD, payload, sizeof(payload));
  test_xfrc(sibling);
  test_receive(2, OTG_GRXST_PKTSTS_OUTRECVD, payload, 3);
  test_xfrc(sibling);
  assert(counts[2] == 1 && req[2]->result == OK && req[2]->xfrd == 67);
  assert(memcmp(buffers[2] + 1, payload, 64) == 0);
  for (i = 0; i < 3; i++)
    {
      assert(buffers[i][0] == 0xa5 && buffers[i][129] == 0xa5);
      (i == 2 ? sibling : out)->ops->freereq(i == 2 ? sibling : out,
                                            req[i]);
    }

  test_offline(driver);
}

static void test_stop_progress(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *ep[2];
  struct usbdev_req_s *req[2];
  unsigned int counts[2] =
  {
    0
  };

  uint8_t buffer[65] =
  {
    0
  };

  unsigned int before;
  unsigned int i;

  for (i = 0; i < 2; i++)
    {
      ep[i] = DEV_ALLOCEP(dev, i + 1, true, USB_EP_ATTR_XFER_INT);
      assert(test_configure(ep[i], USB_EP_ATTR_XFER_INT, 64) == OK);
      req[i] = test_request(ep[i], buffer + 1, 64, &counts[i]);
      assert(EP_SUBMIT(ep[i], req[i]) == OK);
    }

  before = g_ntxflush;
  g_delayed_stop = true;
  assert(EP_CANCEL(ep[0], req[0]) == OK);
  test_dispatch();
  assert(counts[0] == 0 && counts[1] == 0);
  assert(g_ntxflush == before);
  g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4] |= OTG_EPCTL_NAKSTS;
  g_core[STM32_OTG_DIEPINT_OFFSET(1) / 4] |= OTG_DIEPINT_INEPNE;
  test_dispatch();
  test_tick();
  assert(counts[0] == 0 && g_ntxflush == before);
  g_delayed_stop = false;
  g_flush_stuck = true;
  g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4] &=
    ~(OTG_EPCTL_EPENA | OTG_EPCTL_EPDIS);
  g_core[STM32_OTG_DIEPINT_OFFSET(1) / 4] |= OTG_DIEPINT_EPDISD;
  test_dispatch();
  test_tick();
  assert(g_ntxflush == before + 1 && g_txflush[before] == 1);
  assert(counts[0] == 0);
  assert(EP_CANCEL(ep[1], req[1]) == OK);
  test_dispatch();
  test_tick();
  assert(g_ntxflush == before + 1);
  g_flush_stuck = false;
  test_pump();
  assert(g_ntxflush == before + 2 && g_txflush[before + 1] == 2);
  assert(counts[0] == 1 && counts[1] == 1);
  assert(req[0]->result == -ECONNRESET && req[1]->result == -ECONNRESET);
  for (i = 0; i < 2; i++)
    {
      ep[i]->ops->freereq(ep[i], req[i]);
    }

  test_offline(driver);
}

static void test_queue_cancel(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *in = DEV_ALLOCEP(dev, 1, true, USB_EP_ATTR_XFER_INT);
  struct usbdev_req_s *req[3];
  unsigned int counts[3] =
  {
    0
  };

  uint8_t bytes[65] =
  {
    0
  };

  unsigned int i;

  assert(test_configure(in, USB_EP_ATTR_XFER_INT, 64) == OK);
  for (i = 0; i < 3; i++)
    {
      req[i] = test_request(in, bytes + 1, 64, &counts[i]);
      assert(EP_SUBMIT(in, req[i]) == OK);
    }

  assert(EP_CANCEL(in, req[1]) == OK);
  assert(counts[1] == 1 && req[1]->result == -ECONNRESET);
  assert(counts[0] == 0 && counts[2] == 0);
  assert(EP_CANCEL(in, req[0]) == OK);
  test_pump();
  assert(counts[0] == 1 && req[0]->result == -ECONNRESET);
  assert(counts[2] == 0);
  test_xfrc(in);
  assert(counts[2] == 1 && req[2]->result == OK);
  for (i = 0; i < 3; i++)
    {
      in->ops->freereq(in, req[i]);
    }

  assert(EP_STALL(in) == OK);
  test_pump();
  assert(g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4] & OTG_EPCTL_STALL);
  assert(EP_RESUME(in) == OK);
  assert((g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4] & OTG_EPCTL_STALL) == 0);
  assert(g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4] & OTG_EPCTL_SD0PID);
  test_offline(driver);
}

static void test_control_setup(uint8_t type, uint8_t request,
                                uint16_t value, uint16_t index,
                                uint16_t length)
{
  struct usb_ctrlreq_s ctrl =
  {
    .type = type,
    .req = request,
    .value = {(uint8_t)value, (uint8_t)(value >> 8)},
    .index = {(uint8_t)index, (uint8_t)(index >> 8)},
    .len = {(uint8_t)length, (uint8_t)(length >> 8)}
  };

  test_receive(0, OTG_GRXST_PKTSTS_SETUPRECVD,
               (const uint8_t *)&ctrl, sizeof(ctrl));
  test_receive(0, OTG_GRXST_PKTSTS_SETUPDONE, NULL, 0);
  g_core[STM32_OTG_DOEPINT_OFFSET(0) / 4] |= OTG_DOEPINT_STUP;
  test_dispatch();
}

static void test_control_status_out(void)
{
  test_receive(0, OTG_GRXST_PKTSTS_OUTRECVD, NULL, 0);
  g_core[STM32_OTG_DOEPINT_OFFSET(0) / 4] |= OTG_DOEPINT_XFRC;
  test_dispatch();
}

static void test_control_free(struct usbdev_s *dev)
{
  assert(g_control_req != NULL);
  dev->ep0->ops->freereq(dev->ep0, g_control_req);
  g_control_req = NULL;
}

static void test_ep0(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  uint8_t data[193];
  unsigned int i;
  unsigned int before;

  assert(dev->ep0->maxpacket == 64);
  test_control_setup(0, USB_REQ_SETADDRESS, 17, 0, 0);
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] & OTG_DCFG_DAD_MASK) == 0);
  test_xfrc(dev->ep0);
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] & OTG_DCFG_DAD_MASK) ==
         (17u << 4));
  for (i = 0; i < sizeof(data); i++)
    {
      data[i] = (uint8_t)(i + 23);
    }

  memcpy(g_control_data, data, sizeof(data));
  g_control_length = 129;
  g_txcount[0] = 0;
  before = g_setups;
  test_control_setup(USB_DIR_IN, USB_REQ_GETDESCRIPTOR,
                     USB_DESC_TYPE_DEVICE << 8, 0, 255);
  assert(g_setups == before + 1 && g_control_req != NULL);
  assert(g_control_req->xfrd == 0);
  test_fifo_bytes(0, 0, data, 64);
  test_xfrc(dev->ep0);
  assert(g_control_req->xfrd == 64 && g_completions == 0);
  test_fifo_bytes(0, 16, data + 64, 64);
  test_xfrc(dev->ep0);
  test_fifo_bytes(0, 32, data + 128, 1);
  test_xfrc(dev->ep0);
  assert(g_completions == 1 && g_control_req->result == OK);
  test_control_status_out();
  test_control_free(dev);

  g_control_length = 128;
  before = g_completions;
  test_control_setup(USB_DIR_IN | USB_REQ_TYPE_VENDOR, 0x55, 0, 0, 255);
  test_xfrc(dev->ep0);
  test_xfrc(dev->ep0);
  assert(g_completions == before);
  assert((g_core[STM32_OTG_DIEPTSIZ_OFFSET(0) / 4] &
          OTG_DIEPTSIZ0_XFRSIZ_MASK) == 0);
  test_xfrc(dev->ep0);
  assert(g_completions == before + 1);
  test_control_status_out();
  test_control_free(dev);

  for (i = 0; i < 2; i++)
    {
      unsigned int packet;

      before = g_setups;
      test_control_setup(i == 0 ? USB_REQ_TYPE_CLASS : USB_REQ_TYPE_VENDOR,
                         0x56, 0, 0, 129);
      assert(g_setups == before);
      for (packet = 0; packet < 3; packet++)
        {
          test_receive(0, OTG_GRXST_PKTSTS_OUTRECVD,
                       data + packet * 64, packet == 2 ? 1 : 64);
          g_core[STM32_OTG_DOEPINT_OFFSET(0) / 4] |= OTG_DOEPINT_XFRC;
          test_dispatch();
        }

      assert(g_setups == before + 1 && g_control_length == 129);
      assert(memcmp(g_control_data, data, 129) == 0);
      test_xfrc(dev->ep0);
    }

  before = g_setups;
  test_control_setup(USB_REQ_TYPE_VENDOR, 0x57, 0, 0, 4097);
  assert(g_setups == before);
  assert(g_core[STM32_OTG_DIEPCTL_OFFSET(0) / 4] & OTG_EPCTL_STALL);
  test_control_setup(0, USB_REQ_SETADDRESS, 128, 0, 0);
  assert(g_core[STM32_OTG_DIEPCTL_OFFSET(0) / 4] & OTG_EPCTL_STALL);
  test_control_setup(USB_DIR_IN, USB_REQ_SETADDRESS, 1, 0, 0);
  assert(g_core[STM32_OTG_DIEPCTL_OFFSET(0) / 4] & OTG_EPCTL_STALL);
  test_control_setup(0, USB_REQ_SETADDRESS, 18, 0, 0);
  assert((g_core[STM32_OTG_DIEPCTL_OFFSET(0) / 4] & OTG_EPCTL_STALL) == 0);
  test_xfrc(dev->ep0);
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] & OTG_DCFG_DAD_MASK) ==
         (18u << 4));

  g_control_length = 129;
  before = g_completions;
  test_control_setup(USB_DIR_IN | USB_REQ_TYPE_VENDOR, 0x55, 0, 0, 255);
  assert(g_control_req != NULL);
  test_control_setup(0, USB_REQ_SETADDRESS, 19, 0, 0);
  test_pump();
  assert(g_completions == before + 1 && g_control_req->result < 0);
  test_control_free(dev);
  test_xfrc(dev->ep0);
  assert((g_core[STM32_OTG_DCFG_OFFSET / 4] & OTG_DCFG_DAD_MASK) ==
         (19u << 4));
  test_offline(driver);
}

static void test_abort_events(struct usbdevclass_driver_s *driver)
{
  unsigned int event;

  for (event = 0; event < 4; event++)
    {
      struct usbdev_s *dev = test_online(driver);
      struct usbdev_ep_s *in = DEV_ALLOCEP(dev, 1, true,
                                          USB_EP_ATTR_XFER_INT);
      struct usbdev_req_s *req[2];
      unsigned int counts[2] =
      {
        0
      };

      unsigned int disconnects;
      uint8_t bytes[65] =
      {
        0
      };

      assert(test_configure(in, USB_EP_ATTR_XFER_INT, 64) == OK);
      req[0] = test_request(in, bytes + 1, 64, &counts[0]);
      req[1] = test_request(in, bytes + 1, 64, &counts[1]);
      assert(EP_SUBMIT(in, req[0]) == OK);
      assert(EP_SUBMIT(in, req[1]) == OK);
      disconnects = g_disconnects;
      if (event == 0)
        {
          test_event(OTG_GINT_USBRST);
          test_pump();
        }
      else if (event == 1)
        {
          assert(stm32_usbdev_vbus(false) == OK);
          test_pump();
          assert(stm32_usbdev_vbus(false) == OK);
          assert(g_disconnects == disconnects + 1);
        }
      else
        {
          unsigned int i;

          g_delayed_stop = event == 2;
          g_flush_stuck = event == 3;
          assert(EP_CANCEL(in, req[0]) == OK);
          for (i = 0; i < 1000 && g_otgdev.state != USBSTATE_FAULT; i++)
            {
              test_pump();
            }

          assert(g_otgdev.state == USBSTATE_FAULT);
          assert(g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS);
          assert(g_rcc[STM32_RCC_AHB5RSTR_OFFSET / 4] &
                 STM32_OTG_RCC_RST);
        }

      assert(counts[0] == 1 && counts[1] == 1);
      assert(req[0]->result < 0 && req[1]->result < 0);
      in->ops->freereq(in, req[0]);
      in->ops->freereq(in, req[1]);
      g_delayed_stop = false;
      g_flush_stuck = false;
      if (event < 2)
        {
          test_pump();
          assert(counts[0] == 1 && counts[1] == 1);
          test_offline(driver);
        }
      else
        {
          g_runtime = false;
          arm_usbuninitialize();
        }
    }
}

static void test_rebind(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *ep[2];
  struct usbdev_req_s *req[3];
  unsigned int counts[3] =
  {
    0
  };

  unsigned int time;
  unsigned int cycle;
  unsigned int i;
  uint8_t buffers[3][65] =
  {
    {
      0
    }
  };

  ep[0] = DEV_ALLOCEP(dev, 1, true, USB_EP_ATTR_XFER_INT);
  ep[1] = DEV_ALLOCEP(dev, 1, false, USB_EP_ATTR_XFER_INT);
  for (i = 0; i < 2; i++)
    {
      assert(test_configure(ep[i], USB_EP_ATTR_XFER_INT, 64) == OK);
    }

  for (i = 0; i < 3; i++)
    {
      struct usbdev_ep_s *endpoint = ep[i == 2 ? 1 : 0];

      req[i] = test_request(endpoint, buffers[i] + 1, 64, &counts[i]);
      assert(EP_SUBMIT(endpoint, req[i]) == OK);
    }

  assert(usbdev_unregister(driver) == OK);
  assert(g_otgdev.state == USBSTATE_READY && g_otgdev.driver == NULL);
  assert(g_otgdev.link == LINK_PRESENT);
  assert(!g_irqenabled);
  for (i = 0; i < 3; i++)
    {
      struct usbdev_ep_s *endpoint = ep[i == 2 ? 1 : 0];

      assert(counts[i] == 1 && req[i]->result == -ESHUTDOWN);
      endpoint->ops->freereq(endpoint, req[i]);
    }

  for (cycle = 0; cycle < 3; cycle++)
    {
      time = g_time;
      assert(usbdev_register(driver) == OK);
      assert(g_irqenabled && g_otgdev.driver == driver);
      assert(g_time == time);
      assert((DEV_CONNECT(dev)) == OK);
      assert((g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS) == 0);
      assert(usbdev_unregister(driver) == OK);
      assert(!g_irqenabled && g_otgdev.state == USBSTATE_READY);
      assert(g_otgdev.link == LINK_PRESENT);
    }

  assert(g_binds == 4 && g_unbinds == 4);
  for (i = 0; i < 3; i++)
    {
      assert(counts[i] == 1);
    }

  g_runtime = false;
  arm_usbuninitialize();
  assert(g_otgdev.state == USBSTATE_OFF);
}

static void test_vbus_race(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *ep = DEV_ALLOCEP(dev, 1, true, USB_EP_ATTR_XFER_INT);
  struct usbdev_req_s *req;
  unsigned int count = 0;
  unsigned int disconnected = g_disconnects;
  uint8_t buffer[65] =
  {
    0
  };

  assert(test_configure(ep, USB_EP_ATTR_XFER_INT, 64) == OK);
  req = test_request(ep, buffer + 1, 64, &count);
  assert(EP_SUBMIT(ep, req) == OK);
  g_delayed_stop = true;
  assert(stm32_usbdev_vbus(false) == OK);
  assert(g_otgdev.state == USBSTATE_DETACHING);
  assert(count == 0);
  assert(stm32_usbdev_vbus(true) == OK);
  assert(g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS);
  g_delayed_stop = false;
  g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4] |= OTG_EPCTL_NAKSTS;
  g_core[STM32_OTG_DIEPINT_OFFSET(1) / 4] |= OTG_DIEPINT_INEPNE;
  test_pump();
  if (count != 1 || req->result != -ESHUTDOWN)
    {
      fprintf(stderr, "VBUS race: count=%u result=%d state=%d "
              "epstate=%d link=%d flush=%d ctl=%08x intr=%08x\n",
              count, req->result, g_otgdev.state,
              g_otgdev.epin[1].state, g_otgdev.link, g_otgdev.flush,
              g_core[STM32_OTG_DIEPCTL_OFFSET(1) / 4],
              g_core[STM32_OTG_DIEPINT_OFFSET(1) / 4]);
    }

  assert(count == 1 && req->result == -ESHUTDOWN);
  assert(g_disconnects == disconnected + 1);
  assert((g_core[STM32_OTG_DCTL_OFFSET / 4] & OTG_DCTL_SDIS) == 0);
  assert(g_core[STM32_OTG_GCCFG_OFFSET / 4] & OTG_GCCFG_VBVALOVAL);
  ep->ops->freereq(ep, req);
  test_offline(driver);
}

static void test_suspend_queue(struct usbdevclass_driver_s *driver)
{
  struct usbdev_s *dev = test_online(driver);
  struct usbdev_ep_s *ep = DEV_ALLOCEP(dev, 1, true, USB_EP_ATTR_XFER_INT);
  struct usbdev_req_s *req[2];
  unsigned int counts[2] =
  {
    0
  };

  uint8_t buffer[65] =
  {
    0
  };

  unsigned int written;
  unsigned int i;

  assert(test_configure(ep, USB_EP_ATTR_XFER_INT, 64) == OK);
  for (i = 0; i < 2; i++)
    {
      req[i] = test_request(ep, buffer + 1, 64, &counts[i]);
      assert(EP_SUBMIT(ep, req[i]) == OK);
    }

  written = g_txcount[1];
  test_event(OTG_GINT_USBSUSP);
  assert(g_otgdev.state == USBSTATE_SUSPENDED);
  test_xfrc(ep);
  assert(counts[0] == 1 && req[0]->result == OK);
  assert(counts[1] == 0 && req[1]->xfrd == 0);
  assert(g_txcount[1] == written);
  test_event(OTG_GINT_WKUPINT);
  assert(g_otgdev.state == USBSTATE_ENUMERATED);
  assert(g_txcount[1] == written + 16);
  test_xfrc(ep);
  assert(counts[1] == 1 && req[1]->result == OK && req[1]->xfrd == 64);
  for (i = 0; i < 2; i++)
    {
      ep->ops->freereq(ep, req[i]);
    }

  test_offline(driver);
}

static void test_halt_queue(struct usbdevclass_driver_s *driver)
{
  unsigned int direction;
  unsigned int pending;

  for (direction = 0; direction < 2; direction++)
    {
      for (pending = 0; pending < 2; pending++)
        {
          struct usbdev_s *dev = test_online(driver);
          bool in = direction == 0;
          struct usbdev_ep_s *ep = DEV_ALLOCEP(dev, 1, in,
                                              USB_EP_ATTR_XFER_INT);
          struct usbdev_req_s *req[2];
          unsigned int counts[2] =
          {
            0
          };

          uint8_t buffers[2][130];
          uint8_t payload[64];
          size_t sent = pending != 0 ? 64 : 0;
          unsigned int i;

          memset(buffers, 0xa5, sizeof(buffers));
          memset(payload, 0x5a, sizeof(payload));
          assert(test_configure(ep, USB_EP_ATTR_XFER_INT, 64) == OK);
          req[0] = test_request(ep, buffers[0] + 1, 128, &counts[0]);
          req[1] = test_request(ep, buffers[1] + 1, 3, &counts[1]);
          assert(EP_SUBMIT(ep, req[0]) == OK);
          assert(EP_SUBMIT(ep, req[1]) == OK);
          if (pending != 0)
            {
              if (!in)
                {
                  test_receive(1, OTG_GRXST_PKTSTS_OUTRECVD,
                               payload, sizeof(payload));
                }

              test_pending_xfrc(ep);
            }

          assert(EP_STALL(ep) == OK);
          test_pump();
          assert(counts[0] == 0 && counts[1] == 0);
          assert(req[0]->xfrd == sent && req[1]->xfrd == 0);
          assert(g_core[(in ? STM32_OTG_DIEPCTL_OFFSET(1) :
                         STM32_OTG_DOEPCTL_OFFSET(1)) / 4] &
                 OTG_EPCTL_STALL);
          assert(EP_RESUME(ep) == OK);
          assert(g_core[(in ? STM32_OTG_DIEPCTL_OFFSET(1) :
                         STM32_OTG_DOEPCTL_OFFSET(1)) / 4] &
                 OTG_EPCTL_SD0PID);
          test_dispatch();
          assert(counts[0] == 0 && counts[1] == 0);
          while (sent < 128)
            {
              if (!in)
                {
                  test_receive(1, OTG_GRXST_PKTSTS_OUTRECVD,
                               payload, sizeof(payload));
                }

              test_xfrc(ep);
              sent += 64;
              assert(req[0]->xfrd == sent);
            }

          assert(counts[0] == 1 && counts[1] == 0);
          assert(req[0]->result == OK);
          if (!in)
            {
              test_receive(1, OTG_GRXST_PKTSTS_OUTRECVD, payload, 3);
            }

          test_xfrc(ep);
          assert(counts[1] == 1 && req[1]->result == OK &&
                 req[1]->xfrd == 3);
          for (i = 0; i < 2; i++)
            {
              assert(buffers[i][0] == 0xa5 &&
                     buffers[i][req[i]->len + 1] == 0xa5);
              ep->ops->freereq(ep, req[i]);
            }

          test_offline(driver);
        }
    }
}

static void test_cancel_pending_ack(struct usbdevclass_driver_s *driver)
{
  unsigned int zlp;

  for (zlp = 0; zlp < 2; zlp++)
    {
      struct usbdev_s *dev = test_online(driver);
      struct usbdev_ep_s *ep = DEV_ALLOCEP(dev, 1, true,
                                          USB_EP_ATTR_XFER_INT);
      struct usbdev_req_s *req[2];
      unsigned int counts[2] =
      {
        0
      };

      uint8_t buffer[129] =
      {
        0
      };

      unsigned int i;

      assert(test_configure(ep, USB_EP_ATTR_XFER_INT, 64) == OK);
      req[0] = test_request(ep, buffer + 1, zlp != 0 ? 64 : 128,
                            &counts[0]);
      req[1] = test_request(ep, buffer + 1, 3, &counts[1]);
      if (zlp != 0)
        {
          req[0]->flags = USBDEV_REQFLAGS_NULLPKT;
        }

      assert(EP_SUBMIT(ep, req[0]) == OK);
      assert(EP_SUBMIT(ep, req[1]) == OK);
      test_pending_xfrc(ep);
      assert(EP_CANCEL(ep, req[0]) == OK);
      test_pump();
      assert(counts[0] == 1 && req[0]->result == -ECONNRESET &&
             req[0]->xfrd == 64);
      assert(counts[1] == 0 && req[1]->xfrd == 0);
      test_xfrc(ep);
      assert(counts[1] == 1 && req[1]->result == OK && req[1]->xfrd == 3);
      for (i = 0; i < 2; i++)
        {
          ep->ops->freereq(ep, req[i]);
        }

      test_offline(driver);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  const struct usbdevclass_driverops_s ops =
  {
    .bind = test_bind,
    .unbind = test_unbind,
    .setup = test_setup,
    .disconnect = test_disconnect
  };

  struct usbdevclass_driver_s driver =
  {
    .ops = &ops,
#ifdef CONFIG_STM32_N6_OTGDEV_FS
    .speed = USB_SPEED_FULL
#else
    .speed = USB_SPEED_HIGH
#endif
  };

  struct usbdevclass_driver_s invalid =
  {
    .ops = NULL
  };

  uintptr_t base;
  uintptr_t phy;
  unsigned int ep;
  uint32_t mask;
  uint32_t clocks;
  uint32_t updated;

#ifdef CONFIG_STM32_N6_OTG1
  base = 0x58040000;
  phy = 0x5803fc00;
  assert(STM32_OTG_PORT == 1);
  assert(STM32_IRQ_OTG == 193);
  assert(STM32_OTG_RIFSC_INDEX == 56);
  assert(STM32_OTG_RCC_EN == 0x04000000);
  assert(STM32_OTG_RCC_PHY_EN == 0x08000000);
  assert(STM32_OTG_RCC_RST == 0x04000000);
  assert(STM32_OTG_RCC_PHY_RST == 0x08000000);
  assert(STM32_OTG_RCC_PHYCTL_RST == 0x00800000);
#else
  base = 0x58080000;
  phy = 0x580c0000;
  assert(STM32_OTG_PORT == 2);
  assert(STM32_IRQ_OTG == 194);
  assert(STM32_OTG_RIFSC_INDEX == 57);
  assert(STM32_OTG_RCC_EN == 0x20000000);
  assert(STM32_OTG_RCC_PHY_EN == 0x10000000);
  assert(STM32_OTG_RCC_RST == 0x20000000);
  assert(STM32_OTG_RCC_PHY_RST == 0x10000000);
  assert(STM32_OTG_RCC_PHYCTL_RST == 0x01000000);
#endif

  assert(STM32_OTG_NENDPOINTS == 9);
  assert(STM32_OTG_FIFO_BYTES == 4096);
  assert(STM32_OTG_FIFO_WORDS == 1024);
  assert(STM32_OTG_GCCFG == base + 0x38);
  assert(STM32_OTG_DCFG == base + 0x800);
  assert(STM32_OTG_DCTL == base + 0x804);
  assert(STM32_OTG_DSTS == base + 0x808);
  assert(STM32_OTG_DAINT == base + 0x818);
  assert(STM32_OTG_DAINTMSK == base + 0x81c);
  assert(STM32_OTG_DIEPEMPMSK == base + 0x834);
  assert(STM32_OTG_PCGCCTL == base + 0xe00);
  assert(STM32_OTG_GRXSTSR == base + 0x1c);
  assert(STM32_OTG_GRXSTSP == base + 0x20);

  for (ep = 0; ep < STM32_OTG_NENDPOINTS; ep++)
    {
      assert(STM32_OTG_DIEPCTL(ep) == base + 0x900 + 32 * ep);
      assert(STM32_OTG_DIEPINT(ep) == base + 0x908 + 32 * ep);
      assert(STM32_OTG_DIEPTSIZ(ep) == base + 0x910 + 32 * ep);
      assert(STM32_OTG_DIEPDMA(ep) == base + 0x914 + 32 * ep);
      assert(STM32_OTG_DTXFSTS(ep) == base + 0x918 + 32 * ep);
      assert(STM32_OTG_DOEPCTL(ep) == base + 0xb00 + 32 * ep);
      assert(STM32_OTG_DOEPINT(ep) == base + 0xb08 + 32 * ep);
      assert(STM32_OTG_DOEPTSIZ(ep) == base + 0xb10 + 32 * ep);
      assert(STM32_OTG_DOEPDMA(ep) == base + 0xb14 + 32 * ep);
      assert(STM32_OTG_DFIFO(ep) == base + 0x1000 + 0x1000 * ep);
      assert(OTG_DAINT_IN(ep) == (1u << ep));
      assert(OTG_DAINT_OUT(ep) == (1u << (16 + ep)));
      if (ep > 0)
        {
          assert(STM32_OTG_DIEPTXF(ep) == base + 0x100 + 4 * ep);
        }
    }

  assert(OTG_DAINT_ALL == 0x01ff01ff);
  assert(OTG_GOTGINT_W1C_MASK == 0x00040004);
  assert(OTG_GINTSTS_W1C_MASK == 0xd8f0fc0a);
  assert((OTG_GINTSTS_W1C_MASK &
          (OTG_GINT_RXFLVL | OTG_GINT_IEPINT | OTG_GINT_OEPINT)) == 0);
  assert(OTG_DIEPINT_W1C_MASK == 0x297f);
  assert((OTG_DIEPINT_W1C_MASK & OTG_DIEPINT_TXFE) == 0);
  assert(OTG_DOEPINT_W1C_MASK == 0xf17f);
  assert(OTG_DIEPTSIZ0_XFRSIZ_MASK == 0x7f);
  assert(OTG_DIEPTSIZ0_PKTCNT_MASK == 0x180000);
  assert(OTG_DOEPTSIZ0_PKTCNT == 0x80000);
  assert(OTG_DOEPTSIZ0_STUPCNT_MASK == 0x60000000);
  assert(OTG_EPTSIZ_XFRSIZ_MASK == 0x7ffff);
  assert(OTG_EPTSIZ_PKTCNT_MASK == 0x1ff80000);
  assert(OTG_DOEPCTL0_MPSIZ_MASK == 3);
  assert(OTG_DOEPCTL0_MPSIZ_64 == 0);
  assert(OTG_DOEPCTL0_MPSIZ_8 == 3);
  assert(OTG_DCFG_DSPD_FS == 1);
  assert(OTG_DSTS_ENUMSPD_FS == 2);
  assert(OTG_GCCFG_VBVALOVAL == 0x00800000);
  assert(OTG_GCCFG_VBUSVLD == 0x10);
  assert(STM32_USBPHYC_CR == phy);
  assert(STM32_USBPHYC_TRIM1CR == phy + 4);
  assert(STM32_USBPHYC_TRIM2CR == phy + 8);
  assert(USBPHYC_CR_FSEL_MASK == 0x70);
  assert(USBPHYC_CR_FSEL_24MHZ == 0x20);
  assert(STM32_RCC_HSECFGR_OFFSET == 0x54);
  assert(RCC_CR_HSEON == 0x10);
  assert(RCC_SR_HSERDY == 0x10);
  assert(RCC_HSECFGR_HSEDIV2SEL == 0x40);
  assert(RCC_HSECFGR_HSEBYP == 0x8000);
  assert(RCC_HSECFGR_HSEEXT == 0x10000);
  assert(STM32_RCC_CCIPR6_OFFSET == 0x158);
  assert(STM32_RCC_AHB5ENR_OFFSET == 0x260);
  assert(STM32_RCC_AHB5ENSR_OFFSET == 0xa60);
  assert(STM32_RCC_AHB5ENCR_OFFSET == 0x1260);
  assert(STM32_RCC_AHB5RSTR_OFFSET == 0x220);
  assert(STM32_RCC_AHB5RSTSR_OFFSET == 0xa20);
  assert(STM32_RCC_AHB5RSTCR_OFFSET == 0x1220);
  assert(PWR_SVMCR3_USB33VMEN == 4);
  assert(PWR_SVMCR3_USB33SV == 0x400);
  assert(PWR_SVMCR3_USB33RDY == 0x40000);

  mask = RCC_CCIPR6_OTGPHY1SEL_MASK | RCC_CCIPR6_OTGPHY1CKREFSEL |
         RCC_CCIPR6_OTGPHY2SEL_MASK | RCC_CCIPR6_OTGPHY2CKREFSEL;
  assert(mask == 0x01313000);
  clocks = 0x321;
  updated = (clocks & ~mask) | RCC_CCIPR6_OTGPHY1CKREFSEL |
            RCC_CCIPR6_OTGPHY2CKREFSEL;
  assert((updated & 0x333) == clocks);

#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  assert(usbdev_register(&driver) == -EACCES);
#else
  assert(usbdev_register(&driver) == -ENODEV);
#endif
  assert(usbdev_register(NULL) == -EINVAL);
  assert(usbdev_register(&invalid) == -EINVAL);
  assert(usbdev_unregister(NULL) == -EINVAL);
#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  assert(usbdev_unregister(&driver) == -EACCES);
#else
  assert(usbdev_unregister(&driver) == -ENODEV);
#endif
  assert(g_binds == 0);
#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  test_fixture(TEST_NONE);
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EACCES);
  assert(g_reads == 0 && g_writes == 0 && g_irqdisables == 0);
  (void)test_lifecycle;
  (void)test_allocations;
  (void)test_transfers;
  (void)test_queue_cancel;
  (void)test_out_cancel;
  (void)test_stop_progress;
  (void)test_ep0;
  (void)test_abort_events;
  (void)test_rebind;
  (void)test_vbus_race;
  (void)test_suspend_queue;
  (void)test_halt_queue;
  (void)test_cancel_pending_ack;
#else
  test_lifecycle(&driver);
  test_allocations(&driver);
  test_transfers(&driver);
  test_queue_cancel(&driver);
  test_out_cancel(&driver);
  test_stop_progress(&driver);
  test_ep0(&driver);
  test_abort_events(&driver);
  test_rebind(&driver);
  test_vbus_race(&driver);
  test_suspend_queue(&driver);
  test_halt_queue(&driver);
  test_cancel_pending_ack(&driver);
#endif
  return 0;
}
