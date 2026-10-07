/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_tim_driver_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/stm32n6xxx_rcc.h"
#include "stm32n6/stm32n6xx_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_IRQ_FIRST 16
#define NVIC_SYSH_PRIORITY_DEFAULT 128
#define DEBUGASSERT(c) assert(c)
#define OK 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*xcpt_t)(int, void *, void *);

/* DRIVER_HEADER */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if TEST_ENABLED_MASK != 0
static const uintptr_t g_bases[19] =
{
  0, STM32_TIM1_BASE, STM32_TIM2_BASE, STM32_TIM3_BASE, STM32_TIM4_BASE,
  STM32_TIM5_BASE, STM32_TIM6_BASE, STM32_TIM7_BASE, STM32_TIM8_BASE,
  STM32_TIM9_BASE, STM32_TIM10_BASE, STM32_TIM11_BASE, STM32_TIM12_BASE,
  STM32_TIM13_BASE, STM32_TIM14_BASE, STM32_TIM15_BASE, STM32_TIM16_BASE,
  STM32_TIM17_BASE, STM32_TIM18_BASE
};

static const uint8_t g_channels[19] =
{
  0, 6, 4, 4, 4, 4, 0, 0, 6, 2, 1, 1, 2, 1, 1, 2, 1, 1, 0
};

static const uint32_t g_enable_masks[19] =
{
  0, RCC_APB2ENR_TIM1EN, RCC_APB1LENR_TIM2EN, RCC_APB1LENR_TIM3EN,
  RCC_APB1LENR_TIM4EN, RCC_APB1LENR_TIM5EN, RCC_APB1LENR_TIM6EN,
  RCC_APB1LENR_TIM7EN, RCC_APB2ENR_TIM8EN, RCC_APB2ENR_TIM9EN,
  RCC_APB1LENR_TIM10EN, RCC_APB1LENR_TIM11EN, RCC_APB1LENR_TIM12EN,
  RCC_APB1LENR_TIM13EN, RCC_APB1LENR_TIM14EN, RCC_APB2ENR_TIM15EN,
  RCC_APB2ENR_TIM16EN, RCC_APB2ENR_TIM17EN, RCC_APB2ENR_TIM18EN
};

static const int g_irqs[19] =
{
  0, STM32_IRQ_TIM1_UP, STM32_IRQ_TIM2, STM32_IRQ_TIM3, STM32_IRQ_TIM4,
  STM32_IRQ_TIM5, STM32_IRQ_TIM6, STM32_IRQ_TIM7, STM32_IRQ_TIM8_UP,
  STM32_IRQ_TIM9, STM32_IRQ_TIM10, STM32_IRQ_TIM11, STM32_IRQ_TIM12,
  STM32_IRQ_TIM13, STM32_IRQ_TIM14, STM32_IRQ_TIM15, STM32_IRQ_TIM16,
  STM32_IRQ_TIM17, STM32_IRQ_TIM18
};

static uint32_t g_regs[19][64];
static uint32_t g_apb1;
static uint32_t g_apb2;
static unsigned int g_writes;
static unsigned int g_access_width;
static unsigned int g_gpio_calls;
static uint32_t g_gpio;
static bool g_gpio_enabled;
static int g_irq;
static xcpt_t g_handler;
static void *g_arg;
static bool g_irq_enabled;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t *regaddr(uintptr_t address)
{
  unsigned int timer;

  if (address == STM32_RCC_APB1LENR)
    {
      return &g_apb1;
    }

  if (address == STM32_RCC_APB2ENR)
    {
      return &g_apb2;
    }

  for (timer = 1; timer <= 18; timer++)
    {
      if (address >= g_bases[timer] &&
          address < g_bases[timer] + sizeof(g_regs[timer]))
        {
          assert((address & 3) == 0);
          return &g_regs[timer][(address - g_bases[timer]) / 4];
        }
    }

  assert(false);
  return NULL;
}

static uint16_t getreg16(uintptr_t address)
{
  g_access_width = 16;
  return *regaddr(address);
}

static uint32_t getreg32(uintptr_t address)
{
  g_access_width = 32;
  return *regaddr(address);
}

static void putreg16(uint16_t value, uintptr_t address)
{
  uint32_t *reg = regaddr(address);

  g_access_width = 16;
  g_writes++;
  *reg = (*reg & 0xffff0000u) | value;
}

static void putreg32(uint32_t value, uintptr_t address)
{
  g_access_width = 32;
  g_writes++;
  *regaddr(address) = value;
}

static void modifyreg16(uintptr_t address, uint16_t clear, uint16_t set)
{
  putreg16((getreg16(address) & ~clear) | set, address);
}

static void modifyreg32(uintptr_t address, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(address) & ~clear) | set, address);
}

static int stm32_configgpio(uint32_t cfg)
{
  g_gpio_calls++;
  g_gpio = cfg;
  g_gpio_enabled = true;
  return OK;
}

static int stm32_unconfiggpio(uint32_t cfg)
{
  g_gpio_calls++;
  g_gpio = cfg;
  g_gpio_enabled = false;
  return OK;
}

static int irq_attach(int irq, xcpt_t handler, void *arg)
{
  g_irq = irq;
  g_handler = handler;
  g_arg = arg;
  return OK;
}

static int irq_detach(int irq)
{
  assert(irq == g_irq);
  g_handler = NULL;
  return OK;
}

static void up_enable_irq(int irq)
{
  assert(irq == g_irq);
  g_irq_enabled = true;
}

static void up_disable_irq(int irq)
{
  assert(irq == g_irq);
  g_irq_enabled = false;
}

#ifdef CONFIG_ARCH_IRQPRIO
static int up_prioritize_irq(int irq, int priority)
{
  assert(irq == g_irq);
  assert(priority == NVIC_SYSH_PRIORITY_DEFAULT);
  return OK;
}
#endif
#endif

/* DRIVER_SOURCE */

#if TEST_ENABLED_MASK != 0
static int handler(int irq, void *context, void *arg)
{
  (void)irq;
  (void)context;
  (void)arg;
  return OK;
}

static bool has_gpio(unsigned int timer, unsigned int channel)
{
#ifdef TEST_ALL_GPIO
  (void)timer;
  (void)channel;
  return true;
#elif defined(TEST_SPARSE_GPIO)
  return (timer == 1 && channel == 6) ||
         (timer == 2 && channel == 3) ||
         (timer == 8 && channel == 5);
#else
  (void)timer;
  (void)channel;
  return false;
#endif
}

static void test_timer(unsigned int timer)
{
  struct stm32_tim_dev_s *dev;
  uintptr_t base = g_bases[timer];
  uintptr_t rcc = timer == 1 || timer == 8 || timer == 9 || timer >= 15 ?
                  STM32_RCC_APB2ENR : STM32_RCC_APB1LENR;
  unsigned int width = timer == 2 || timer == 4 || timer == 5 ? 32 : 16;
  unsigned int channel;
  unsigned int writes;
  bool basic = g_channels[timer] == 0;
  bool bidirectional = timer <= 5 || timer == 8;
  bool moe = timer == 1 || timer == 8 || (timer >= 15 && timer <= 17);

  if ((TEST_ENABLED_MASK & (1u << timer)) == 0)
    {
      writes = g_writes;
      assert(stm32_tim_init(timer) == NULL);
      assert(g_writes == writes);
      return;
    }

  putreg32(0x80000000u, rcc);
  dev = stm32_tim_init(timer);
  assert(dev != NULL);
  assert(getreg32(rcc) == (0x80000000u | g_enable_masks[timer]));
  assert(stm32_tim_init(timer) == NULL);
  assert(STM32_TIM_GETWIDTH(dev) == (int)width);
  STM32_TIM_SETCOUNTER(dev, 0x12345678);
  assert(g_access_width == width);
  assert(STM32_TIM_GETCOUNTER(dev) ==
         (width == 32 ? 0x12345678u : 0x5678u));
  assert(g_access_width == width);
  STM32_TIM_SETPERIOD(dev, 0xabcdef);
  assert(g_access_width == width);
  assert(getreg32(base + STM32_GTIM_ARR_OFFSET) ==
         (width == 32 ? 0xabcdefu : 0xcdefu));
  assert(STM32_TIM_SETCLOCK(dev, 1000000) == (int)timer - 1);
  assert(getreg16(base + STM32_GTIM_PSC_OFFSET) == timer - 1);
  assert(STM32_TIM_SETCLOCK(dev, 1) == 0xffff);
  assert(STM32_TIM_SETCLOCK(dev, UINT32_MAX) == 0);
  assert(STM32_TIM_SETCLOCK(dev, 0) == OK);
  assert((getreg16(base + STM32_GTIM_CR1_OFFSET) & GTIM_CR1_CEN) == 0);
  assert(STM32_TIM_SETMODE(dev, STM32_TIM_MODE_UP) ==
         (basic ? -EINVAL : OK));
  if (moe)
    {
      assert((getreg16(base + STM32_ATIM_BDTR_OFFSET) & ATIM_BDTR_MOE) != 0);
    }

  assert(STM32_TIM_SETMODE(dev, STM32_TIM_MODE_DOWN) ==
         (bidirectional ? OK : -EINVAL));
  assert(STM32_TIM_SETMODE(dev, STM32_TIM_MODE_UPDOWN) ==
         (bidirectional ? OK : -EINVAL));

  for (channel = 1; channel <= g_channels[timer]; channel++)
    {
      unsigned int calls = g_gpio_calls;
      unsigned int offset = channel <= 4 ?
                            STM32_GTIM_CCR1_OFFSET + 4 * (channel - 1) :
                            channel == 5 ? STM32_ATIM_CCR5_OFFSET :
                                           STM32_ATIM_CCR6_OFFSET;

      assert(STM32_TIM_SETCHANNEL(dev, channel, STM32_TIM_CH_OUTPWM) == OK);
      assert(g_gpio_calls == calls + has_gpio(timer, channel));
      if (has_gpio(timer, channel))
        {
          assert(g_gpio_enabled);
          assert(g_gpio == (timer == 1 && channel == 6 ?
                            0 : timer * 16 + channel));
        }

      assert(STM32_TIM_SETCOMPARE(dev, channel, 0x12345678) == OK);
      assert(g_access_width == width);
      assert(getreg32(base + offset) ==
             (width == 32 ? 0x12345678u : 0x5678u));
      assert(STM32_TIM_GETCAPTURE(dev, channel) ==
             (channel > 4 ? -EINVAL :
              width == 32 ? 0x12345678 : 0x5678));
      assert(STM32_TIM_SETCHANNEL(dev, channel,
                                  STM32_TIM_CH_DISABLED) == OK);
      assert(g_gpio_calls == calls + 2 * has_gpio(timer, channel));
      if (has_gpio(timer, channel))
        {
          assert(!g_gpio_enabled);
        }
    }

  writes = g_writes;
  assert(STM32_TIM_SETCHANNEL(dev, 0, STM32_TIM_CH_OUTPWM) == -EINVAL);
  assert(STM32_TIM_SETCHANNEL(dev, g_channels[timer] + 1,
                              STM32_TIM_CH_OUTPWM) == -EINVAL);
  assert(STM32_TIM_SETCOMPARE(dev, 0, 1) == -EINVAL);
  assert(STM32_TIM_SETCOMPARE(dev, g_channels[timer] + 1, 1) == -EINVAL);
  assert(STM32_TIM_GETCAPTURE(dev, 0) == -EINVAL);
  assert(STM32_TIM_GETCAPTURE(dev, g_channels[timer] + 1) == -EINVAL);
  assert(g_writes == writes);
  assert(STM32_TIM_SETISR(dev, handler, dev, 0) == OK);
  assert(g_irq == g_irqs[timer]);
  assert(g_handler == handler && g_arg == dev && g_irq_enabled);
  assert(STM32_TIM_SETISR(dev, NULL, NULL, 0) == OK);
  assert(g_handler == NULL && !g_irq_enabled);
  assert(stm32_tim_deinit(dev) == OK);
  assert(getreg32(rcc) == 0x80000000u);
  assert(stm32_tim_init(timer) == dev);
  assert(stm32_tim_deinit(dev) == OK);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
#if TEST_ENABLED_MASK != 0
  unsigned int timer;

  assert(stm32_tim_init(INT_MIN) == NULL);
  assert(stm32_tim_init(-1) == NULL);
  assert(stm32_tim_init(0) == NULL);
  assert(stm32_tim_init(19) == NULL);
  assert(stm32_tim_init(INT_MAX) == NULL);
  assert(g_writes == 0);
  for (timer = 1; timer <= 18; timer++)
    {
      test_timer(timer);
    }
#endif

  return 0;
}
