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
#include <string.h>

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
#define CONTROL_NPRIV 1
#define TEST_CLOCKS (STM32_OTG_RCC_EN | STM32_OTG_RCC_PHY_EN)
#define TEST_RESETS (STM32_OTG_RCC_RST | STM32_OTG_RCC_PHY_RST | \
                     STM32_OTG_RCC_PHYCTL_RST)

/* USB_HEADERS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef unsigned int irqstate_t;

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

void arm_usbinitialize(void);
void arm_usbuninitialize(void);

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
  uint32_t *reg = test_register(address);

  g_reads++;
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

      if (g_failure != TEST_TX_FLUSH)
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
  else if (address == STM32_OTG_DCTL && g_time >= g_nak_due &&
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
  uint32_t *reg = test_register(address);

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
      *reg = value | OTG_GRSTCTL_AHBIDL;
      g_reset_due = g_time + 2;
    }
  else if (address == STM32_OTG_GUSBCFG)
    {
      assert(g_soft_reset && g_time >= g_reset_done + 3);
      *reg = value;
      g_mode_due = g_time + 25000;
    }
  else if (address == STM32_OTG_DCTL)
    {
      assert((value & OTG_DCTL_SDIS) != 0);
      *reg = value & ~(OTG_DCTL_SGINAK | OTG_DCTL_SGONAK);
      if ((value & (OTG_DCTL_SGINAK | OTG_DCTL_SGONAK)) != 0)
        {
          g_nak_due = g_time + 2;
        }
    }
  else if (address == STM32_OTG_GCCFG)
    {
      assert((value & OTG_GCCFG_VBVALOVAL) == 0);
      *reg = value;
    }
  else if (address == STM32_OTG_GINTSTS ||
           address == STM32_OTG_GOTGINT)
    {
      uint32_t mask = address == STM32_OTG_GINTSTS ?
                      OTG_GINTSTS_W1C_MASK : OTG_GOTGINT_W1C_MASK;

      assert(value == mask);
      *reg &= ~value;
    }
  else if (address >= STM32_OTG_BASE + 0x900 &&
           address <= STM32_OTG_BASE + 0xc08 &&
           (address & 0x1f) == 8)
    {
      assert(value == (address < STM32_OTG_BASE + 0xb00 ?
                      OTG_DIEPINT_W1C_MASK : OTG_DOEPINT_W1C_MASK));
      *reg &= ~value;
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
}

static void up_udelay(unsigned int delay)
{
  assert(g_critical == 0);
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

  assert(g_otgdev.initialized);
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
  assert(g_otgdev.usbdev.ops == NULL && g_otgdev.usbdev.ep0 == NULL);
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
  assert(usbdev_register(driver) == -ENOSYS);
  assert(g_writes == before);
  arm_usbuninitialize();
  assert(!g_otgdev.initialized && g_otgdev.initresult == -ENODEV);
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
  test_initialized();

  for (fault = TEST_RESET; fault <= TEST_RX_FLUSH; fault++)
    {
      test_fixture(fault);
      arm_usbinitialize();
      ret = g_otgdev.initresult;
      assert(ret < 0 && ret != -ENOSYS);
      assert(!g_otgdev.initialized);
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
      assert(!g_otgdev.initialized && g_otgdev.initresult == -EACCES);
      assert(g_otgdev.resets_owned);
      assert(g_errors != 0);
      g_failure = TEST_NONE;
      arm_usbinitialize();
      test_initialized();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  const struct usbdevclass_driverops_s ops =
  {
    .bind = test_bind
  };

  struct usbdevclass_driver_s driver =
  {
    .ops = &ops
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

  assert(usbdev_register(&driver) == -ENODEV);
  assert(usbdev_register(NULL) == -EINVAL);
  assert(usbdev_register(&invalid) == -EINVAL);
  assert(usbdev_unregister(NULL) == -EINVAL);
  assert(usbdev_unregister(&driver) == -ENODEV);
  assert(g_binds == 0);
#ifdef CONFIG_ARCH_TRUSTZONE_NONSECURE
  test_fixture(TEST_NONE);
  arm_usbinitialize();
  assert(g_otgdev.initresult == -EACCES);
  assert(g_reads == 0 && g_writes == 0 && g_irqdisables == 0);
  (void)test_lifecycle;
#else
  test_lifecycle(&driver);
#endif
  return 0;
}
