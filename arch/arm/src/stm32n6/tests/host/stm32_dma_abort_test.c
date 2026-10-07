/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_dma_abort_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "stm32_dma.h"
#include "hardware/stm32n6xxx_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UNUSED(value) (void)(value)
#define STM32_DMA_WAIT_LOOPS 4
/* DMA_INTERRUPTS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef unsigned int irqstate_t;
/* DMA_TYPES */;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_dma_channel_s g_channel;
static uint32_t g_regs[64];
static unsigned int g_invalidations;
static unsigned int g_resets;
static bool g_suspend_timeout;
static bool g_reset_timeout;
static bool g_irq_enabled;
static bool g_suspended;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static irqstate_t enter_critical_section(void)
{
  return 0;
}

static void leave_critical_section(irqstate_t flags)
{
  UNUSED(flags);
}

static void up_disable_irq(int irq)
{
  assert(irq == g_channel.irq);
  g_irq_enabled = false;
}

static void up_enable_irq(int irq)
{
  assert(irq == g_channel.irq);
  g_irq_enabled = true;
}

static struct stm32_dma_channel_s *stm32_dma_getchannel(DMA_HANDLE handle)
{
  return handle == &g_channel ? &g_channel : NULL;
}

static uint32_t stm32_dma_getreg(struct stm32_dma_channel_s *channel,
                                unsigned int offset)
{
  assert(channel == &g_channel && offset / 4 < 64);
  if (offset == STM32_DMA_CXBR1_OFFSET(0))
    {
      assert(g_suspended ||
             (g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] & STM32_DMA_CR_EN) == 0);
    }

  return g_regs[offset / 4];
}

static void stm32_dma_putreg(struct stm32_dma_channel_s *channel,
                             unsigned int offset, uint32_t value)
{
  assert(channel == &g_channel && offset / 4 < 64);
  if (offset == STM32_DMA_CXCR_OFFSET(0))
    {
      if ((value & STM32_DMA_CR_RESET) != 0)
        {
          assert(g_suspended ||
                 (g_regs[offset / 4] & STM32_DMA_CR_EN) == 0);
          g_resets++;
          if (!g_reset_timeout)
            {
              memset(g_regs, 0, sizeof(g_regs));
              return;
            }

          g_regs[offset / 4] |= value;
          return;
        }
      else if ((value & STM32_DMA_CR_SUSP) != 0 && !g_suspend_timeout)
        {
          g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] |= STM32_DMA_FLAG_SUSPF;
          g_suspended = true;
        }
    }

  g_regs[offset / 4] = value;
}

static void stm32_dma_cache_invalidate_current(
  struct stm32_dma_channel_s *channel)
{
  assert(channel == &g_channel);
  g_invalidations++;
}

/* DMA_ROUTINES */

static void reset(void)
{
  memset(&g_channel, 0, sizeof(g_channel));
  memset(g_regs, 0, sizeof(g_regs));
  g_channel.allocated = true;
  g_channel.configured = true;
  g_channel.in_flight = true;
  g_channel.config.nbytes = 17;
  g_channel.config.width = 1;
  g_channel.request.direction = STM32_DMA_MEMORY_TO_PERIPHERAL;
  g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = STM32_DMA_CR_EN;
  g_suspend_timeout = g_reset_timeout = g_suspended = false;
  g_irq_enabled = true;
  g_invalidations = g_resets = 0;
  assert(g_channel.abort_state == STM32_DMA_ABORT_NONE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  size_t transferred;
  unsigned int width;
  unsigned int remaining;
  unsigned int buffered;
  unsigned int controller;

  for (controller = 0; controller < 2; controller++)
    {
      for (width = 1; width <= 4; width *= 2)
        {
          for (remaining = 0; remaining <= 16; remaining += width)
            {
              for (buffered = 0; buffered * width + remaining <= 16;
                   buffered++)
                {
                  reset();
                  g_channel.controller = controller;
                  g_channel.config.nbytes = 16;
                  g_channel.config.width = width;
                  g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = remaining;
                  g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] =
                    buffered << STM32_DMA_SR_FIFOL_SHIFT;
                  transferred = 99;
                  assert(stm32_dmaabort(&g_channel, &transferred) == 0);
                  assert(transferred == 16 - remaining - buffered * width);
                  assert(g_channel.abort_state == STM32_DMA_ABORT_VALID);
                  assert(!g_channel.configured && !g_channel.in_flight);
                  assert(g_resets == 1 && g_invalidations == 1);
                  assert(g_irq_enabled);
                }
            }
        }
    }

  reset();
  g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = 0;
  g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = 10;
  g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] = 3 << STM32_DMA_SR_FIFOL_SHIFT;
  assert(stm32_dmaabort(&g_channel, &transferred) == 0);
  assert(transferred == 4);

  reset();
  g_suspend_timeout = true;
  transferred = 99;
  assert(stm32_dmaabort(&g_channel, &transferred) == -ETIMEDOUT);
  assert(transferred == 99 && g_channel.configured && g_channel.in_flight);
  assert(g_resets == 0 && g_invalidations == 0 && g_irq_enabled);
  assert(g_channel.abort_state == STM32_DMA_ABORT_NONE);
  g_suspend_timeout = false;
  g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = 5;
  assert(stm32_dmaabort(&g_channel, &transferred) == 0);
  assert(transferred == 12);

  reset();
  g_reset_timeout = true;
  g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = 10;
  g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] = 3 << STM32_DMA_SR_FIFOL_SHIFT;
  transferred = 99;
  assert(stm32_dmaabort(&g_channel, &transferred) == -ETIMEDOUT);
  assert(transferred == 99 && g_channel.configured && g_channel.in_flight);
  assert(g_invalidations == 0 && g_irq_enabled);
  assert(g_channel.abort_state == STM32_DMA_ABORT_VALID);
  assert(g_channel.abort_transferred == 4);
  g_reset_timeout = false;
  memset(g_regs, 0, sizeof(g_regs));
  assert(stm32_dmaabort(&g_channel, &transferred) == 0);
  assert(transferred == 4);

  reset();
  g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = 18;
  transferred = 99;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EIO);
  assert(transferred == 99 && g_channel.configured && g_channel.in_flight);
  assert(g_resets == 0 && g_invalidations == 0);
  assert(g_channel.abort_state == STM32_DMA_ABORT_UNKNOWN);
  memset(g_regs, 0, sizeof(g_regs));
  assert(stm32_dmaabort(&g_channel, &transferred) == -EIO);
  assert(transferred == 99);
  assert(g_channel.abort_state == STM32_DMA_ABORT_UNKNOWN);

  reset();
  g_channel.status = DMA_STATUS_DTEF;
  transferred = 99;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EIO);
  assert(transferred == 99 && g_channel.configured);
  assert(g_channel.abort_state == STM32_DMA_ABORT_UNKNOWN);
  assert(g_resets == 0 && stm32_dmastop(&g_channel) == 0);

  reset();
  g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] = STM32_DMA_FLAG_DTEF;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EIO);
  assert(g_resets == 0);
  assert(g_channel.abort_state == STM32_DMA_ABORT_UNKNOWN);

  reset();
  g_channel.descriptor_count = 2;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EINVAL);
  g_channel.descriptor_count = 0;
  g_channel.request.direction = STM32_DMA_PERIPHERAL_TO_MEMORY;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EINVAL);
  g_channel.request.direction = STM32_DMA_MEMORY_TO_PERIPHERAL;
  g_channel.starting = true;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EBUSY);
  assert(stm32_dmaabort(NULL, &transferred) == -EINVAL);
  assert(stm32_dmaabort(&g_channel, NULL) == -EINVAL);

  reset();
  assert(stm32_dmastop(&g_channel) == 0);
  assert(!g_channel.configured && !g_channel.in_flight);
  puts("STM32N6 DMA abort snapshot tests passed");
  return 0;
}
