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

#include "stm32_dma_internal.h"
#include "hardware/stm32n6xxx_gpdma.h"
#include "hardware/stm32n6xxx_hpdma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UNUSED(value) (void)(value)
#define STM32_DMA_WAIT_LOOPS 4
#define STM32_DMA_CHANNELS 32
#define g_channel g_dma_channels[0]
/* DMA_INTERRUPTS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef unsigned int irqstate_t;
/* DMA_TYPES */;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_dma_channel_s g_dma_channels[STM32_DMA_CHANNELS];
static struct stm32_dma_lli_s g_descriptors[3];
static uint32_t g_regs[64];
static unsigned int g_invalidations;
static unsigned int g_resets;
static bool g_suspend_timeout;
static bool g_reset_timeout;
static bool g_irq_enabled;
static bool g_suspended;
static bool g_complete_on_suspend;
static bool g_cache_valid;
static unsigned int g_critical_depth;
static unsigned int g_prepares;
static unsigned int g_callbacks;
static uint8_t g_callback_status;
static bool g_rearm;
static void (*g_on_unlock)(void);
static void (*g_on_invalidate)(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static irqstate_t enter_critical_section(void)
{
  return g_critical_depth++;
}

static void leave_critical_section(irqstate_t flags)
{
  assert(g_critical_depth == flags + 1);
  g_critical_depth = flags;
  if (g_critical_depth == 0 && g_on_unlock != NULL &&
      (g_channel.state == STM32_DMA_STARTING ||
       g_channel.state == STM32_DMA_STOPPING))
    {
      void (*hook)(void) = g_on_unlock;

      g_on_unlock = NULL;
      hook();
    }
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
  if (offset == STM32_DMA_CXBR1_OFFSET(0) &&
      g_channel.state == STM32_DMA_STOPPING)
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
  if (offset == STM32_DMA_CXFCR_OFFSET(0))
    {
      g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] &= ~value;
      return;
    }

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
      if ((value & STM32_DMA_CR_SUSP) != 0 && g_complete_on_suspend)
        {
          assert((value & STM32_DMA_CR_EN) == 0);
          g_regs[offset / 4] &= ~STM32_DMA_CR_EN;
          g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = 0;
          g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] = STM32_DMA_FLAG_TCF;
        }
      else if ((value & STM32_DMA_CR_SUSP) != 0 && !g_suspend_timeout)
        {
          g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] |= STM32_DMA_FLAG_SUSPF;
          g_suspended = true;
        }

      /* EN ignores writes of zero. */

      value |= g_regs[offset / 4] & STM32_DMA_CR_EN;
    }

  g_regs[offset / 4] = value;
}

static void stm32_dma_cache_invalidate_current(
  struct stm32_dma_channel_s *channel)
{
  assert(channel == &g_channel);
  if (g_on_invalidate != NULL)
    {
      g_on_invalidate();
    }

  g_invalidations++;
}

static bool stm32_dma_cache_valid(struct stm32_dma_channel_s *channel)
{
  assert(channel == &g_channel);
  return g_cache_valid;
}

static void stm32_dma_cache_prepare(struct stm32_dma_channel_s *channel)
{
  assert(channel == &g_channel && channel->state == STM32_DMA_STARTING);
  assert(g_critical_depth == 0);
  g_prepares++;
}

/* DMA_ROUTINES */

static void reset(void)
{
  assert(g_critical_depth == 0);
  memset(g_dma_channels, 0, sizeof(g_dma_channels));
  memset(g_regs, 0, sizeof(g_regs));
  g_channel.allocated = true;
  g_channel.state = STM32_DMA_RUNNING;
  g_channel.config.source_address = 0x20000000;
  g_channel.config.destination_address = 0x40000000;
  g_channel.config.nbytes = 17;
  g_channel.config.width = 1;
  g_channel.config.source_increment = true;
  g_channel.request.request = 1;
  g_channel.request.direction = STM32_DMA_MEMORY_TO_PERIPHERAL;
  g_channel.request.peripheral_address = 0x40000000;
  g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = STM32_DMA_CR_EN;
  g_suspend_timeout = g_reset_timeout = g_suspended = false;
  g_irq_enabled = true;
  g_invalidations = g_resets = 0;
  g_cache_valid = true;
  g_prepares = g_callbacks = 0;
  g_callback_status = 0;
  g_rearm = false;
  g_on_unlock = NULL;
  g_on_invalidate = NULL;
  g_complete_on_suspend = false;
  assert(g_channel.abort_state == STM32_DMA_ABORT_NONE);
}

static void callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  assert(handle == &g_channel && arg == &g_callbacks);
  assert(g_critical_depth == 0);
  g_callbacks++;
  g_callback_status = status;
  if (g_rearm)
    {
      struct stm32_dma_config_s config = g_channel.config;

      assert(g_channel.state == STM32_DMA_COMPLETE);
      assert(stm32_dmasetup(handle, &config) == 0);
      assert(stm32_dmastart(handle) == 0);
    }
}

static void interrupt(uint32_t status)
{
  if ((status & STM32_DMA_FLAG_CLEAR_MASK) == STM32_DMA_FLAG_TCF &&
      (g_channel.descriptor_count == 0 ||
       (g_channel.list_mode == STM32_DMA_LIST_TERMINAL &&
        g_channel.descriptor_index + 1 == g_channel.descriptor_count)))
    {
      g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] &= ~STM32_DMA_CR_EN;
      g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] = 0;
    }
  else if ((status & (STM32_DMA_FLAG_DTEF | STM32_DMA_FLAG_ULEF |
                      STM32_DMA_FLAG_USEF)) != 0)
    {
      g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] &= ~STM32_DMA_CR_EN;
    }

  g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] = status;
  assert(stm32_dma_interrupt(g_channel.irq, NULL, &g_channel) == 0);
  assert((g_regs[STM32_DMA_CXSR_OFFSET(0) / 4] &
          STM32_DMA_FLAG_CLEAR_MASK) == 0);
}

static void check_busy(void)
{
  struct stm32_dma_config_s config = g_channel.config;
  struct stm32_dma_status_s status;
  enum stm32_dma_transfer_state_e state = g_channel.state;
  size_t transferred = 99;
  uint32_t control = g_regs[STM32_DMA_CXCR_OFFSET(0) / 4];

  assert(stm32_dmastart(&g_channel) == -EBUSY);
  assert(stm32_dmasetup(&g_channel, &config) == -EBUSY);
  assert(stm32_dmallibuild(&g_channel, &config, 1, g_descriptors, 3,
                         STM32_DMA_LIST_TERMINAL) == -EBUSY);
  assert(stm32_dmafree(&g_channel) == -EBUSY);
  if (state == STM32_DMA_STARTING || state == STM32_DMA_STOPPING)
    {
      assert(stm32_dmastop(&g_channel) == -EBUSY);
      assert(stm32_dmaabort(&g_channel, &transferred) == -EBUSY);
      assert(transferred == 99);
    }

  assert(stm32_dmastatus(&g_channel, &status) == 0);
  assert(status.in_flight == (state != STM32_DMA_ERROR));
  if (state == STM32_DMA_STOPPING || state == STM32_DMA_RECOVERY)
    {
      assert(status.remaining == 0);
    }

  assert(g_channel.state == state && g_channel.allocated);
  assert(g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] == control);
}

static void test_states(void)
{
  struct stm32_dma_config_s configs[3];
  struct stm32_dma_status_s status;
  struct stm32_dma_lli_s saved[3];
  const uint32_t errors[] =
    {
      STM32_DMA_FLAG_DTEF, STM32_DMA_FLAG_ULEF, STM32_DMA_FLAG_USEF
    };
  unsigned int i;
  unsigned int j;
  size_t transferred;

  reset();
  g_channel.state = STM32_DMA_UNCONFIGURED;
  g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = 0;
  assert(stm32_dmastart(&g_channel) == -EINVAL);
  assert(stm32_dmaabort(&g_channel, &transferred) == -EINVAL);
  configs[0] = g_channel.config;
  assert(stm32_dmasetup(&g_channel, configs) == 0);
  assert(g_channel.state == STM32_DMA_READY);
  g_cache_valid = false;
  assert(stm32_dmastart(&g_channel) == -EINVAL);
  assert(g_channel.state == STM32_DMA_READY && g_prepares == 0);
  g_cache_valid = true;
  g_on_unlock = check_busy;
  assert(stm32_dmastart(&g_channel) == 0);
  assert(g_on_unlock == NULL && g_prepares == 1);
  assert(g_channel.state == STM32_DMA_RUNNING);
  check_busy();
  assert(stm32_dmacallback(&g_channel, callback, &g_callbacks) == 0);
  interrupt(STM32_DMA_FLAG_HTF);
  assert(g_channel.state == STM32_DMA_RUNNING && g_invalidations == 0);
  interrupt(STM32_DMA_FLAG_TCF);
  assert(g_channel.state == STM32_DMA_COMPLETE && g_invalidations == 1);
  assert(g_callbacks == 2 && g_callback_status == DMA_STATUS_TCF);
  assert(stm32_dmastatus(&g_channel, &status) == 0);
  assert(!status.in_flight && status.remaining == 0);
  assert(status.flags == (DMA_STATUS_HTF | DMA_STATUS_TCF));
  assert(stm32_dmastart(&g_channel) == -EINVAL);
  assert(stm32_dmasetup(&g_channel, configs) == 0);
  assert(g_regs[STM32_DMA_CXBR1_OFFSET(0) / 4] == configs[0].nbytes);
  assert(stm32_dmastart(&g_channel) == 0);
  g_rearm = true;
  interrupt(STM32_DMA_FLAG_TCF);
  assert(g_channel.state == STM32_DMA_RUNNING && g_prepares == 3);
  g_rearm = false;
  g_on_unlock = check_busy;
  g_on_invalidate = check_busy;
  assert(stm32_dmastop(&g_channel) == 0);
  g_on_invalidate = NULL;
  assert(g_on_unlock == NULL && g_channel.state == STM32_DMA_UNCONFIGURED);
  assert(stm32_dmafree(&g_channel) == 0);
  assert(stm32_dmastatus(&g_channel, &status) == -EINVAL);

  for (i = 0; i < sizeof(errors) / sizeof(errors[0]); i++)
    {
      reset();
      assert(stm32_dmacallback(&g_channel, callback, &g_callbacks) == 0);
      interrupt(errors[i] | STM32_DMA_FLAG_TCF);
      assert(g_channel.state == STM32_DMA_ERROR && g_channel.error == -EIO);
      assert(g_callbacks == 1 && g_invalidations == 1);
      check_busy();
      if (errors[i] == STM32_DMA_FLAG_DTEF)
        {
          transferred = 99;
          assert(stm32_dmaabort(&g_channel, &transferred) == -EIO);
          assert(transferred == 99 && g_channel.state == STM32_DMA_RECOVERY);
          check_busy();
          interrupt(STM32_DMA_FLAG_TCF | STM32_DMA_FLAG_DTEF);
          assert(g_channel.state == STM32_DMA_RECOVERY);
          assert(g_channel.error == -EIO && g_callbacks == 1);
        }

      assert(stm32_dmastop(&g_channel) == 0 && g_resets == 1);
      assert(stm32_dmasetup(&g_channel, configs) == 0);
      assert(stm32_dmastart(&g_channel) == 0);
    }

  for (i = STM32_DMA_READY; i <= STM32_DMA_COMPLETE; i++)
    {
      if (i == STM32_DMA_STARTING)
        {
          continue;
        }

      reset();
      g_channel.state = i;
      if (i != STM32_DMA_RUNNING)
        {
          g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = 0;
        }

      g_on_unlock = check_busy;
      assert(stm32_dmastop(&g_channel) == 0);
      assert(g_on_unlock == NULL && g_resets == 1 && g_irq_enabled);
    }

  for (i = STM32_DMA_LIST_TERMINAL; i <= STM32_DMA_LIST_PINGPONG; i++)
    {
      reset();
      g_channel.state = STM32_DMA_READY;
      g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = 0;
      configs[0] = g_channel.config;
      configs[1] = configs[2] = configs[0];
      memset(g_descriptors, 0xa5, sizeof(g_descriptors));
      memcpy(saved, g_descriptors, sizeof(saved));
      configs[1].width = 3;
      assert(stm32_dmallibuild(&g_channel, configs, 2, g_descriptors, 3,
                             i) == -EINVAL);
      assert(memcmp(saved, g_descriptors, sizeof(saved)) == 0);
      assert(g_channel.state == STM32_DMA_READY);
      configs[1].width = 1;
      g_channel.abort_state = STM32_DMA_ABORT_UNKNOWN;
      assert(stm32_dmallibuild(&g_channel, configs, 2, g_descriptors, 3,
                             i) == 0);
      assert(g_channel.abort_state == STM32_DMA_ABORT_NONE);
      assert(stm32_dmastart(&g_channel) == 0);
      for (j = 0; j < (i == STM32_DMA_LIST_TERMINAL ? 2 : 6); j++)
        {
          interrupt(STM32_DMA_FLAG_TCF);
          if (i == STM32_DMA_LIST_TERMINAL && j == 1)
            {
              assert(g_channel.state == STM32_DMA_COMPLETE);
              assert(stm32_dmastart(&g_channel) == -EINVAL);
            }
          else
            {
              assert(g_channel.state == STM32_DMA_RUNNING);
              assert(g_channel.descriptor_index == (j + 1) % 2);
            }
        }

      assert(stm32_dmastop(&g_channel) == 0);
    }

  for (i = STM32_DMA_STARTING; i <= STM32_DMA_RECOVERY; i++)
    {
      reset();
      g_channel.state = STM32_DMA_READY;
      g_regs[STM32_DMA_CXCR_OFFSET(0) / 4] = 0;
      g_dma_channels[1] = g_channel;
      g_dma_channels[1].state = i;
      assert(stm32_dmastart(&g_channel) ==
             (i == STM32_DMA_COMPLETE ? 0 : -EBUSY));
    }
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
                  assert(g_channel.state == STM32_DMA_UNCONFIGURED);
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
  assert(transferred == 99 && g_channel.state == STM32_DMA_RECOVERY);
  check_busy();
  interrupt(STM32_DMA_FLAG_TCF);
  assert(g_channel.state == STM32_DMA_RECOVERY);
  assert(g_channel.error == -ETIMEDOUT);
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
  assert(transferred == 99 && g_channel.state == STM32_DMA_RECOVERY);
  check_busy();
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
  assert(transferred == 99 && g_channel.state == STM32_DMA_RECOVERY);
  check_busy();
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
  assert(transferred == 99 && g_channel.state == STM32_DMA_RECOVERY);
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
  g_channel.state = STM32_DMA_STARTING;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EBUSY);
  assert(stm32_dmaabort(NULL, &transferred) == -EINVAL);
  assert(stm32_dmaabort(&g_channel, NULL) == -EINVAL);

  reset();
  assert(stm32_dmastop(&g_channel) == 0);
  assert(g_channel.state == STM32_DMA_UNCONFIGURED);
  assert(stm32_dmastop(&g_channel) == 0 && g_resets == 1);

  reset();
  g_complete_on_suspend = true;
  assert(stm32_dmaabort(&g_channel, &transferred) == 0);
  assert(transferred == g_channel.config.nbytes && !g_suspended);
  assert(g_channel.state == STM32_DMA_UNCONFIGURED && g_resets == 1);
  assert(g_channel.status == DMA_STATUS_TCF);

  reset();
  g_reset_timeout = true;
  assert(stm32_dmastop(&g_channel) == -ETIMEDOUT);
  assert(g_channel.state == STM32_DMA_RECOVERY);
  assert(g_channel.abort_state == STM32_DMA_ABORT_UNKNOWN);
  memset(g_regs, 0, sizeof(g_regs));
  transferred = 99;
  assert(stm32_dmaabort(&g_channel, &transferred) == -EIO);
  assert(transferred == 99);
  g_reset_timeout = false;
  assert(stm32_dmastop(&g_channel) == 0);

  test_states();
  puts("STM32N6 DMA state and abort snapshot tests passed");
  return 0;
}
