/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_serial_instances_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <stm32n6/chip.h>
#include <stm32n6/stm32n6xx_irq.h>
#include "hardware/stm32n6xxx_rcc.h"
#include "hardware/stm32n6xxx_dmasigmap.h"
#include "stm32_serial_format.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OK 0
#define UNUSED(value) (void)(value)
#define USART_CR1_USED_INTS \
  (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)
#define USART_UNCONFIGURE_RX 1
#define USART_UNCONFIGURE_TX 2
#define SP_UNLOCKED 0
#define STM32_HSI_FREQUENCY 64000000
#define _err test_log
#define _warn test_log

/* CONSOLE_SELECTION */

/* ACK_TIMEOUT */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef unsigned int irqstate_t;
typedef unsigned int spinlock_t;
typedef void *DMA_HANDLE;

struct uart_buffer_s
{
  unsigned int size;
  unsigned int head;
  unsigned int tail;
  char *buffer;
};

struct uart_dev_s
{
  struct uart_buffer_s recv;
  struct uart_buffer_s xmit;
  const void *ops;
  void *priv;
  bool isconsole;
};

/* DRIVER_TYPES */;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const int g_uart_ops;
#ifdef STM32_USART1_TXDMA
static const int g_uart_dma_ops = 1;
#endif

/* HARDWARE_INSTANCES */

/* DRIVER_INSTANCES */

static uint32_t g_regs[10][12];
static uint32_t g_selectors[2] =
{
  0x11111111, 0x151
};

static uint32_t g_enabled[2];
static uint32_t g_sleep[2];
static unsigned int g_resets[10];
static unsigned int g_errors;
static unsigned int g_registered;
static char g_names[11][16];
static struct uart_dev_s *g_devices[11];
static bool g_registration_failure;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void test_log(const char *format, ...)
{
  UNUSED(format);
  g_errors++;
}

static irqstate_t spin_lock_irqsave(spinlock_t *lock)
{
  assert(*lock == 0);
  *lock = 1;
  return 0;
}

static void spin_unlock_irqrestore(spinlock_t *lock, irqstate_t flags)
{
  UNUSED(flags);
  assert(*lock == 1);
  *lock = 0;
}

static void up_udelay(unsigned int usecs)
{
  UNUSED(usecs);
  assert(false);
}

static unsigned int port_index(uint32_t address)
{
  unsigned int i;

  for (i = 0; i < 10; i++)
    {
      const struct stm32_usart_s *hw = &g_usart_config[i];

      if (hw->base != 0 && address >= hw->base &&
          address < hw->base + sizeof(g_regs[i]))
        {
          unsigned int bus = hw->enable == STM32_RCC_APB2ENSR;

          assert((g_enabled[bus] & hw->rcc_bit) != 0);
          return i;
        }
    }

  assert(false);
  return 0;
}

static uint32_t getreg32(uint32_t address)
{
  unsigned int i;

  if (address == STM32_RCC_HSICFGR)
    {
      return 2 << RCC_HSICFGR_HSIDIV_SHIFT;
    }

  if (address == STM32_RCC_CCIPR13 || address == STM32_RCC_CCIPR14)
    {
      return g_selectors[address == STM32_RCC_CCIPR14];
    }

  i = port_index(address);
  return g_regs[i][(address - g_usart_config[i].base) / 4];
}

static void putreg32(uint32_t value, uint32_t address)
{
  unsigned int i;

  if (address == STM32_RCC_CCIPR13 || address == STM32_RCC_CCIPR14)
    {
      g_selectors[address == STM32_RCC_CCIPR14] = value;
      return;
    }

  for (i = 0; i < 10; i++)
    {
      const struct stm32_usart_s *hw = &g_usart_config[i];
      unsigned int bus = hw->enable == STM32_RCC_APB2ENSR;

      if (hw->base == 0 || hw->rcc_bit != value)
        {
          continue;
        }

      if (address == hw->enable)
        {
          g_enabled[bus] |= value;
          return;
        }

      if (address == hw->disable || address == hw->lpdisable)
        {
          assert((g_regs[i][0] & USART_CR1_UE) == 0);
          if (address == hw->disable)
            {
              g_enabled[bus] &= ~value;
            }
          else
            {
              g_sleep[bus] &= ~value;
            }

          return;
        }

      if (address == hw->lpen)
        {
          g_sleep[bus] |= value;
          return;
        }

      if (address == hw->resetset || address == hw->resetclear)
        {
          if (address == hw->resetset)
            {
              g_resets[i]++;
              memset(g_regs[i], 0, sizeof(g_regs[i]));
            }

          return;
        }
    }

  i = port_index(address);
  unsigned int offset = address - g_usart_config[i].base;

  if (offset == STM32_USART_CR1_OFFSET)
    {
      g_regs[i][STM32_USART_ISR_OFFSET / 4] =
        (value & USART_CR1_UE) != 0 ?
        USART_ISR_TEACK | USART_ISR_REACK | USART_ISR_TXE :
        USART_ISR_TC | USART_ISR_TXE;
    }
  else if (offset == STM32_USART_BRR_OFFSET ||
           offset == STM32_USART_PRESC_OFFSET ||
           offset == STM32_USART_CR2_OFFSET)
    {
      assert((g_regs[i][0] & USART_CR1_UE) == 0);
    }

  g_regs[i][offset / 4] = value;
}

static void modifyreg32(uint32_t address, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(address) & ~clear) | set, address);
}

static int stm32_configgpio(uint32_t pin)
{
  assert(pin != 0);
  return 0;
}

static void stm32_unconfiggpio(uint32_t pin)
{
  assert(pin != 0);
}

#ifdef STM32_USART1_TXDMA
static int stm32_dmastop(DMA_HANDLE handle)
{
  UNUSED(handle);
  assert(false);
  return 0;
}

static int stm32_dmafree(DMA_HANDLE handle)
{
  UNUSED(handle);
  assert(false);
  return 0;
}
#endif

static int uart_register(const char *path, struct uart_dev_s *dev)
{
  assert(g_registered < 11);
  snprintf(g_names[g_registered], sizeof(g_names[0]), "%s", path);
  g_devices[g_registered++] = dev;
  return g_registration_failure ? -EIO : 0;
}

int stm32_usart_disable(uint32_t base);

/* DRIVER_ROUTINES */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  unsigned int i;
  unsigned int minor = 0;
  unsigned int entry = 0;

#if CONSOLE_UART > 0
  struct stm32_usart_format_s format;
  const struct stm32_usart_s *console = &g_usart_config[CONSOLE_UART - 1];

  assert(stm32_usart_initialize(console, false) == 0);
  assert(stm32_usart_format(stm32_usart_clock(console), 115200, 8, 0,
                            false, &format) == 0);
  assert(stm32_usart_configure(console->base, &format, 0) == 0);
#endif

  for (i = 0; i < 10; i++)
    {
      struct stm32_serial_s *priv = g_uart_devs[i];
      const struct stm32_usart_s *hw;
      uint32_t before[2];
      unsigned int selector;
      unsigned int bus;
      unsigned int status;

      if (priv == NULL)
        {
          continue;
        }

      hw = priv->config;
      memcpy(before, g_selectors, sizeof(before));
      assert(hw->irq == STM32_IRQ_USART1 + i);
      assert(hw->rxrequest == 107 + 2 * i && hw->txrequest == 108 + 2 * i);
      assert(stm32serial_setup(&priv->dev) == 0);
      assert(g_resets[i] == (priv->dev.isconsole ? 0 : 1));
      assert(g_regs[i][STM32_USART_BRR_OFFSET / 4] == 139);
      assert(stm32serial_setup(&priv->dev) == 0);
      assert(g_resets[i] == (priv->dev.isconsole ? 0 : 1));
      selector = hw->selector == STM32_RCC_CCIPR14;
      bus = hw->enable == STM32_RCC_APB2ENSR;

      assert((g_selectors[selector] & ~hw->selmask) ==
             (before[selector] & ~hw->selmask));
      assert(g_selectors[selector ^ 1] == before[selector ^ 1]);
      assert((g_sleep[bus] & hw->rcc_bit) != 0);
      stm32serial_send(&priv->dev, 0x40 + i);
      assert(g_regs[i][STM32_USART_TDR_OFFSET / 4] == 0x40 + i);
      g_regs[i][STM32_USART_RDR_OFFSET / 4] = 0x60 + i;
      g_regs[i][STM32_USART_ISR_OFFSET / 4] |= USART_ISR_PE;
      assert(stm32serial_receive(&priv->dev, &status) == (int)(0x60 + i));
      assert((status >> 16 & USART_ISR_PE) != 0);
      assert(g_regs[i][STM32_USART_ICR_OFFSET / 4] == USART_ISR_PE);
#ifdef STM32_USART1_TXDMA
      assert(priv->dev.ops == (i == 0 ?
             (const void *)&g_uart_dma_ops : (const void *)&g_uart_ops));
#endif
    }

  arm_serialinit();
#if CONSOLE_UART > 0
  assert(strcmp(g_names[entry], "/dev/console") == 0);
  assert(g_devices[entry++] == &g_uart_devs[CONSOLE_UART - 1]->dev);
#ifndef CONFIG_STM32_SERIAL_DISABLE_REORDERING
  assert(strcmp(g_names[entry], "/dev/ttyS0") == 0);
  assert(g_devices[entry++] == &g_uart_devs[CONSOLE_UART - 1]->dev);
  minor++;
#endif
#endif

  for (i = 0; i < 10; i++)
    {
      char expected[16];

      if (g_uart_devs[i] == NULL)
        {
          continue;
        }

#ifndef CONFIG_STM32_SERIAL_DISABLE_REORDERING
      if (g_uart_devs[i]->dev.isconsole)
        {
          continue;
        }
#endif

      snprintf(expected, sizeof(expected), "/dev/ttyS%u", minor++);
      assert(strcmp(g_names[entry], expected) == 0);
      assert(g_devices[entry++] == &g_uart_devs[i]->dev);
    }

  assert(entry == g_registered);
  g_registration_failure = true;
  g_registered = 0;
  arm_serialinit();
  assert(g_errors == g_registered);

  for (i = 0; i < 10; i++)
    {
      if (g_uart_devs[i] != NULL && !g_uart_devs[i]->dev.isconsole)
        {
          stm32serial_shutdown(&g_uart_devs[i]->dev);
          assert(!g_uart_devs[i]->initialized);
#if CONSOLE_UART > 0
          assert(g_regs[CONSOLE_UART - 1][0] & USART_CR1_UE);
#endif
        }
    }

  return 0;
}
