/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dma.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_dma.h"
#include "hardware/stm32n6xxx_dmasigmap.h"
#include "hardware/stm32n6xxx_gpdma.h"
#include "hardware/stm32n6xxx_hpdma.h"
#include "stm32_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_DMA_CHANNELS_PER_CONTROLLER 16
#define STM32_DMA_CHANNELS                32
#define STM32_DMA_GPDMA_OFFSET            16
#define STM32_DMA_WAIT_LOOPS              1000000

#define STM32_DMA_INTERRUPT_MASK          (STM32_DMA_CR_TCIE | \
                                           STM32_DMA_CR_HTIE | \
                                           STM32_DMA_CR_DTEIE | \
                                           STM32_DMA_CR_ULEIE | \
                                           STM32_DMA_CR_USEIE | \
                                           STM32_DMA_CR_SUSPIE | \
                                           STM32_DMA_CR_TOIE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_dma_channel_s
{
  enum stm32_dma_controller_e controller;
  uint32_t base;
  uint8_t channel;
  int irq;
  bool initialized;
  bool allocated;
  bool configured;
  bool in_flight;
  struct stm32_dma_request_s request;
  struct stm32_dma_config_s config;
  dma_callback_t callback;
  void *callback_arg;
  uint32_t status;
  int error;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_dma_interrupt(int irq, void *context, void *arg);
static struct stm32_dma_channel_s *stm32_dma_getchannel(DMA_HANDLE handle);
static bool stm32_dma_request_valid(
  const struct stm32_dma_request_s *request);
static int stm32_dma_check_config(
  const struct stm32_dma_channel_s *channel,
  const struct stm32_dma_config_s *config, uint32_t *tr1, uint32_t *tr2);
static int stm32_dma_initialize_controller(
  enum stm32_dma_controller_e controller, uint32_t base, int offset,
  const int *irqs, unsigned int nchannels, unsigned int reserved);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_dma_channel_s g_dma_channels[STM32_DMA_CHANNELS];
static bool g_dma_initialized;

static const int g_hpdma_irqs[STM32_DMA_CHANNELS_PER_CONTROLLER] =
{
  STM32_IRQ_HPDMA1_CH0,  STM32_IRQ_HPDMA1_CH1,
  STM32_IRQ_HPDMA1_CH2,  STM32_IRQ_HPDMA1_CH3,
  STM32_IRQ_HPDMA1_CH4,  STM32_IRQ_HPDMA1_CH5,
  STM32_IRQ_HPDMA1_CH6,  STM32_IRQ_HPDMA1_CH7,
  STM32_IRQ_HPDMA1_CH8,  STM32_IRQ_HPDMA1_CH9,
  STM32_IRQ_HPDMA1_CH10, STM32_IRQ_HPDMA1_CH11,
  STM32_IRQ_HPDMA1_CH12, STM32_IRQ_HPDMA1_CH13,
  STM32_IRQ_HPDMA1_CH14, STM32_IRQ_HPDMA1_CH15
};

static const int g_gpdma_irqs[STM32_DMA_CHANNELS_PER_CONTROLLER] =
{
  STM32_IRQ_GPDMA1_CH0,  STM32_IRQ_GPDMA1_CH1,
  STM32_IRQ_GPDMA1_CH2,  STM32_IRQ_GPDMA1_CH3,
  STM32_IRQ_GPDMA1_CH4,  STM32_IRQ_GPDMA1_CH5,
  STM32_IRQ_GPDMA1_CH6,  STM32_IRQ_GPDMA1_CH7,
  STM32_IRQ_GPDMA1_CH8,  STM32_IRQ_GPDMA1_CH9,
  STM32_IRQ_GPDMA1_CH10, STM32_IRQ_GPDMA1_CH11,
  STM32_IRQ_GPDMA1_CH12, STM32_IRQ_GPDMA1_CH13,
  STM32_IRQ_GPDMA1_CH14, STM32_IRQ_GPDMA1_CH15
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t stm32_dma_getreg(
  const struct stm32_dma_channel_s *channel, uint32_t offset)
{
  return getreg32(channel->base + offset);
}

static inline void stm32_dma_putreg(
  const struct stm32_dma_channel_s *channel, uint32_t offset, uint32_t value)
{
  putreg32(value, channel->base + offset);
}

static struct stm32_dma_channel_s *stm32_dma_getchannel(DMA_HANDLE handle)
{
  unsigned int i;

  if (handle == NULL)
    {
      return NULL;
    }

  for (i = 0; i < STM32_DMA_CHANNELS; i++)
    {
      if (handle == (DMA_HANDLE)&g_dma_channels[i])
        {
          return &g_dma_channels[i];
        }
    }

  return NULL;
}

static bool stm32_dma_controller_enabled(
  enum stm32_dma_controller_e controller)
{
  switch (controller)
    {
#ifdef CONFIG_STM32_HPDMA1
      case STM32_DMA_CONTROLLER_HPDMA1:
        return true;
#endif
#ifdef CONFIG_STM32_GPDMA1
      case STM32_DMA_CONTROLLER_GPDMA1:
        return true;
#endif
      default:
        return false;
    }
}

static bool stm32_dma_request_valid(
  const struct stm32_dma_request_s *request)
{
  int expected;

  if (request == NULL ||
      request->controller < STM32_DMA_CONTROLLER_GPDMA1 ||
      request->controller > STM32_DMA_CONTROLLER_HPDMA1 ||
      request->direction < STM32_DMA_PERIPHERAL_TO_MEMORY ||
      request->direction > STM32_DMA_MEMORY_TO_MEMORY)
    {
      return false;
    }

  if (request->direction == STM32_DMA_MEMORY_TO_MEMORY)
    {
      return request->request == STM32_DMA_REQUEST_NONE &&
             request->peripheral_address == 0;
    }

  if (request->request == STM32_DMA_REQUEST_NONE ||
      request->peripheral_address == 0)
    {
      return false;
    }

  /* Requests with an explicit RX/TX signal have a fixed direction. ADC
   * signals are input-only; timer signals can be used for either direction.
   */

  if (request->request == STM32_DMA_REQ_ADC1 ||
      request->request == STM32_DMA_REQ_ADC2)
    {
      expected = STM32_DMA_PERIPHERAL_TO_MEMORY;
    }
  else if ((request->request >= STM32_DMA_REQ_TIM1_CC1 &&
            request->request <= STM32_DMA_REQ_TIM8_COM) ||
           (request->request >= STM32_DMA_REQ_TIM15_CC1 &&
            request->request <= STM32_DMA_REQ_LPTIM3_UE))
    {
      expected = -1;
    }
  else if ((request->request >= STM32_DMA_REQ_SPI1_RX &&
            request->request <= STM32_DMA_REQ_SPI6_TX) ||
           (request->request >= STM32_DMA_REQ_I2C1_RX &&
            request->request <= STM32_DMA_REQ_I2C4_TX) ||
           (request->request >= STM32_DMA_REQ_USART1_RX &&
            request->request <= STM32_DMA_REQ_LPUART1_TX))
    {
      expected = (request->request & 1) != 0 ?
                 STM32_DMA_PERIPHERAL_TO_MEMORY :
                 STM32_DMA_MEMORY_TO_PERIPHERAL;
    }
  else
    {
      return false;
    }

  return expected < 0 || request->direction == expected;
}

static int stm32_dma_check_config(
  const struct stm32_dma_channel_s *channel,
  const struct stm32_dma_config_s *config, uint32_t *tr1, uint32_t *tr2)
{
  uint32_t width;
  uint32_t source_port = 0;
  uint32_t destination_port = 0;
  unsigned int max_width;

  if (config == NULL || tr1 == NULL || tr2 == NULL ||
      config->nbytes == 0 || config->nbytes > STM32_DMA_BLOCK_MAX ||
      config->priority > 3)
    {
      return -EINVAL;
    }

  switch (config->width)
    {
      case 1:
        width = STM32_DMA_WIDTH_1BYTE;
        break;

      case 2:
        width = STM32_DMA_WIDTH_2BYTES;
        break;

      case 4:
        width = STM32_DMA_WIDTH_4BYTES;
        break;

      case 8:
        width = STM32_DMA_WIDTH_8BYTES;
        break;

      default:
        return -EINVAL;
    }

  max_width = channel->controller == STM32_DMA_CONTROLLER_HPDMA1 ?
              STM32_HPDMA1_MAX_WIDTH_AXI : STM32_GPDMA1_MAX_WIDTH;
  if (config->width > max_width ||
      config->nbytes % config->width != 0 ||
      config->source_address > UINT32_MAX ||
      config->destination_address > UINT32_MAX ||
      config->source_address % config->width != 0 ||
      config->destination_address % config->width != 0)
    {
      return -EINVAL;
    }

  if ((config->source_increment &&
       config->nbytes - 1 >
         UINT32_MAX - config->source_address) ||
      (!config->source_increment &&
       config->width - 1 > UINT32_MAX - config->source_address) ||
      (config->destination_increment &&
       config->nbytes - 1 >
         UINT32_MAX - config->destination_address) ||
      (!config->destination_increment &&
       config->width - 1 > UINT32_MAX - config->destination_address))
    {
      return -EINVAL;
    }

  /* The STM32N6 access policy configures the DMA channel pools as secure.
   * Keep the transfer's source and destination attributes secure as well.
   */

  *tr1 = (width << STM32_DMA_TR1_SDW_SHIFT) |
         (width << STM32_DMA_TR1_DDW_SHIFT) |
         STM32_DMA_TR1_SSEC | STM32_DMA_TR1_DSEC;

  *tr2 = 0;
  switch (channel->request.direction)
    {
      case STM32_DMA_PERIPHERAL_TO_MEMORY:
        if (config->source_address != channel->request.peripheral_address ||
            config->source_increment)
          {
            return -EINVAL;
          }

        if (config->destination_increment)
          {
            *tr1 |= STM32_DMA_TR1_DINC;
          }

        *tr2 = channel->request.request;
        source_port = 1;
        break;

      case STM32_DMA_MEMORY_TO_PERIPHERAL:
        if (config->destination_address !=
              channel->request.peripheral_address ||
            config->destination_increment)
          {
            return -EINVAL;
          }

        if (config->source_increment)
          {
            *tr1 |= STM32_DMA_TR1_SINC;
          }

        *tr2 = channel->request.request | STM32_DMA_TR2_DREQ;
        destination_port = 1;
        break;

      case STM32_DMA_MEMORY_TO_MEMORY:
        if (!config->source_increment || !config->destination_increment)
          {
            return -EINVAL;
          }

        *tr1 |= STM32_DMA_TR1_SINC | STM32_DMA_TR1_DINC;
        *tr2 = STM32_DMA_TR2_SWREQ;

        if (channel->controller == STM32_DMA_CONTROLLER_HPDMA1 &&
            config->width <= STM32_HPDMA1_MAX_WIDTH_AHB)
          {
            source_port = 1;
            destination_port = 1;
          }

        break;

      default:
        return -EINVAL;
    }

  /* HPDMA port 0 is AXI and port 1 is AHB. Peripheral transfers use its AHB
   * port for the peripheral endpoint and AXI for the memory endpoint. For
   * memory-to-memory, select AHB up to 4-byte widths and AXI for 8-byte
   * widths. GPDMA port selections are both AHB and remain at their reset
   * selection.
   */

  if (channel->controller == STM32_DMA_CONTROLLER_HPDMA1)
    {
      if (config->width == 8 &&
          channel->request.direction != STM32_DMA_MEMORY_TO_MEMORY)
        {
          return -EINVAL;
        }

      if (source_port != 0)
        {
          *tr1 |= STM32_DMA_TR1_SAP;
        }

      if (destination_port != 0)
        {
          *tr1 |= STM32_DMA_TR1_DAP;
        }
    }

  return 0;
}

static uint8_t stm32_dma_status(uint32_t status)
{
  uint8_t result = 0;

  if ((status & STM32_DMA_FLAG_TCF) != 0)
    {
      result |= DMA_STATUS_TCF;
    }

  if ((status & STM32_DMA_FLAG_HTF) != 0)
    {
      result |= DMA_STATUS_HTF;
    }

  if ((status & STM32_DMA_FLAG_DTEF) != 0)
    {
      result |= DMA_STATUS_DTEF;
    }

  if ((status & STM32_DMA_FLAG_ULEF) != 0)
    {
      result |= DMA_STATUS_ULEF;
    }

  if ((status & STM32_DMA_FLAG_USEF) != 0)
    {
      result |= DMA_STATUS_USEF;
    }

  if ((status & STM32_DMA_FLAG_SUSPF) != 0)
    {
      result |= DMA_STATUS_SUSPF;
    }

  if ((status & STM32_DMA_FLAG_TOF) != 0)
    {
      result |= DMA_STATUS_TOF;
    }

  return result;
}

static int stm32_dma_interrupt(int irq, void *context, void *arg)
{
  struct stm32_dma_channel_s *channel =
    (struct stm32_dma_channel_s *)arg;
  dma_callback_t callback;
  void *callback_arg;
  irqstate_t flags;
  uint32_t raw_status;
  uint8_t status;

  UNUSED(context);

  if (channel == NULL || irq != channel->irq)
    {
      return -EINVAL;
    }

  raw_status = stm32_dma_getreg(channel,
                                STM32_DMA_CXSR_OFFSET(channel->channel)) &
               STM32_DMA_FLAG_CLEAR_MASK;
  if (raw_status == 0)
    {
      return 0;
    }

  /* CXFCR is write-one-to-clear: clear only flags observed in CXSR. */

  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   raw_status);
  status = stm32_dma_status(raw_status);

  flags = enter_critical_section();
  channel->status |= status;
  if ((status & DMA_STATUS_FATAL) != 0)
    {
      channel->error = -EIO;
      channel->in_flight = false;
    }
  else if ((status & DMA_STATUS_TCF) != 0)
    {
      channel->in_flight = false;
    }

  callback = channel->allocated ? channel->callback : NULL;
  callback_arg = channel->callback_arg;
  leave_critical_section(flags);

  if (callback != NULL)
    {
      callback((DMA_HANDLE)channel, status, callback_arg);
    }

  return 0;
}

static int stm32_dma_initialize_controller(
  enum stm32_dma_controller_e controller, uint32_t base, int offset,
  const int *irqs, unsigned int nchannels, unsigned int reserved)
{
  struct stm32_dma_channel_s *channel;
  irqstate_t flags;
  unsigned int i;
  int ret;

  if (nchannels > STM32_DMA_CHANNELS_PER_CONTROLLER ||
      reserved > nchannels)
    {
      return -EINVAL;
    }

  for (i = 0; i < nchannels; i++)
    {
      channel = &g_dma_channels[offset + i];
      channel->controller = controller;
      channel->base = base;
      channel->channel = i;
      channel->irq = irqs[i];
      channel->initialized = false;

      stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(i), 0);

      if (controller == STM32_DMA_CONTROLLER_HPDMA1)
        {
          /* Allocate the channel to secure OS CID 1.  Enable CID filtering
           * so only that CID can configure or use this channel.
           */

          putreg32(STM32_HPDMA_CIDCFGR_CFEN |
                   STM32_HPDMA_CIDCFGR_SCID(1),
                   STM32_HPDMA1_CXCIDCFGR(i));
        }

      stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(i),
                       STM32_DMA_FLAG_CLEAR_MASK);

      ret = irq_attach(channel->irq, stm32_dma_interrupt, channel);
      if (ret < 0)
        {
          _err("ERROR: Failed to attach DMA channel %u IRQ %d: %d\n",
               i, channel->irq, ret);
          return ret;
        }

      flags = enter_critical_section();
      channel->allocated = false;
      channel->configured = false;
      channel->in_flight = false;
      channel->callback = NULL;
      channel->callback_arg = NULL;
      channel->status = 0;
      channel->error = 0;
      channel->initialized = true;
      leave_critical_section(flags);

      up_enable_irq(channel->irq);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_dma_initialize(void)
{
  int ret;

#if !defined(CONFIG_STM32_HPDMA1) && !defined(CONFIG_STM32_GPDMA1)
  UNUSED(ret);
#endif

  if (g_dma_initialized)
    {
      return 0;
    }

#ifdef CONFIG_STM32_HPDMA1
  ret = stm32_dma_initialize_controller(STM32_DMA_CONTROLLER_HPDMA1,
                                        STM32_HPDMA1_BASE, 0,
                                        g_hpdma_irqs,
                                        CONFIG_STM32_HPDMA1_NCHANNELS,
                                        CONFIG_STM32_HPDMA1_RESERVED_CHANNELS);
  if (ret < 0)
    {
      return ret;
    }
#else
  UNUSED(g_hpdma_irqs);
#endif

#ifdef CONFIG_STM32_GPDMA1
  ret = stm32_dma_initialize_controller(STM32_DMA_CONTROLLER_GPDMA1,
                                        STM32_GPDMA1_BASE,
                                        STM32_DMA_GPDMA_OFFSET,
                                        g_gpdma_irqs,
                                        CONFIG_STM32_GPDMA1_NCHANNELS,
                                        CONFIG_STM32_GPDMA1_RESERVED_CHANNELS);
  if (ret < 0)
    {
      return ret;
    }
#else
  UNUSED(g_gpdma_irqs);
#endif

  g_dma_initialized = true;
  return 0;
}

/* NuttX calls this after irq_initialize(), so the attached DMA handlers
 * remain installed when the NVIC starts dispatching interrupts.
 */

void weak_function arm_dma_initialize(void)
{
  int ret = stm32_dma_initialize();

  if (ret < 0)
    {
      _err("ERROR: DMA initialization failed: %d\n", ret);
      PANIC();
    }
}

DMA_HANDLE stm32_dmachannel(const struct stm32_dma_request_s *request)
{
  struct stm32_dma_channel_s *channel;
  irqstate_t flags;
  unsigned int first;
  unsigned int count;
  unsigned int reserved;
  unsigned int i;
  DMA_HANDLE handle = NULL;

  if (!g_dma_initialized || !stm32_dma_request_valid(request) ||
      !stm32_dma_controller_enabled(request->controller))
    {
      return NULL;
    }

  if (request->controller == STM32_DMA_CONTROLLER_HPDMA1)
    {
#ifdef CONFIG_STM32_HPDMA1
      first = 0;
      count = CONFIG_STM32_HPDMA1_NCHANNELS;
      reserved = CONFIG_STM32_HPDMA1_RESERVED_CHANNELS;
#else
      return NULL;
#endif
    }
  else
    {
#ifdef CONFIG_STM32_GPDMA1
      first = STM32_DMA_GPDMA_OFFSET;
      count = CONFIG_STM32_GPDMA1_NCHANNELS;
      reserved = CONFIG_STM32_GPDMA1_RESERVED_CHANNELS;
#else
      return NULL;
#endif
    }

  flags = enter_critical_section();
  for (i = reserved; i < count; i++)
    {
      channel = &g_dma_channels[first + i];
      if (channel->initialized && !channel->allocated)
        {
          channel->allocated = true;
          channel->request = *request;
          channel->configured = false;
          channel->in_flight = false;
          channel->callback = NULL;
          channel->callback_arg = NULL;
          channel->status = 0;
          channel->error = 0;
          handle = (DMA_HANDLE)channel;
          break;
        }
    }

  leave_critical_section(flags);
  return handle;
}

int stm32_dmafree(DMA_HANDLE handle)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  irqstate_t flags;
  int ret = 0;

  if (channel == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!channel->allocated)
    {
      ret = -EINVAL;
    }
  else if (channel->in_flight ||
           (stm32_dma_getreg(channel,
                             STM32_DMA_CXCR_OFFSET(channel->channel)) &
            STM32_DMA_CR_EN) != 0)
    {
      ret = -EBUSY;
    }
  else
    {
      stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel), 0);
      stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                       STM32_DMA_FLAG_CLEAR_MASK);
      channel->allocated = false;
      channel->configured = false;
      channel->callback = NULL;
      channel->callback_arg = NULL;
    }

  leave_critical_section(flags);
  return ret;
}

int stm32_dmasetup(DMA_HANDLE handle,
                   const struct stm32_dma_config_s *config)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  irqstate_t flags;
  uint32_t tr1;
  uint32_t tr2;
  int ret;

  if (channel == NULL || config == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!channel->allocated)
    {
      ret = -EINVAL;
      goto out;
    }

  if (channel->in_flight ||
      (stm32_dma_getreg(channel,
                        STM32_DMA_CXCR_OFFSET(channel->channel)) &
       STM32_DMA_CR_EN) != 0)
    {
      ret = -EBUSY;
      goto out;
    }

  ret = stm32_dma_check_config(channel, config, &tr1, &tr2);
  if (ret < 0)
    {
      goto out;
    }

  stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel), 0);
  stm32_dma_putreg(channel, STM32_DMA_CXLBAR_OFFSET(channel->channel), 0);
  stm32_dma_putreg(channel, STM32_DMA_CXLLR_OFFSET(channel->channel), 0);
  stm32_dma_putreg(channel, STM32_DMA_CXSAR_OFFSET(channel->channel),
                   config->source_address);
  stm32_dma_putreg(channel, STM32_DMA_CXDAR_OFFSET(channel->channel),
                   config->destination_address);
  stm32_dma_putreg(channel, STM32_DMA_CXTR1_OFFSET(channel->channel), tr1);
  stm32_dma_putreg(channel, STM32_DMA_CXTR2_OFFSET(channel->channel), tr2);
  stm32_dma_putreg(channel, STM32_DMA_CXBR1_OFFSET(channel->channel),
                   config->nbytes);
  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   STM32_DMA_FLAG_CLEAR_MASK);

  channel->config = *config;
  channel->configured = true;
  channel->status = 0;
  channel->error = 0;

out:
  leave_critical_section(flags);
  return ret;
}

int stm32_dmacallback(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  irqstate_t flags;
  int ret = 0;

  if (channel == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!channel->allocated)
    {
      ret = -EINVAL;
    }
  else
    {
      channel->callback = callback;
      channel->callback_arg = arg;
    }

  leave_critical_section(flags);
  return ret;
}

int stm32_dmastart(DMA_HANDLE handle)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  struct stm32_dma_channel_s *other;
  irqstate_t flags;
  uint32_t control;
  unsigned int i;
  int ret = 0;

  if (channel == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!channel->allocated)
    {
      ret = -EINVAL;
      goto out;
    }

  if (!channel->configured)
    {
      ret = -EINVAL;
      goto out;
    }

  control = stm32_dma_getreg(channel,
                             STM32_DMA_CXCR_OFFSET(channel->channel));
  if (channel->in_flight || (control & STM32_DMA_CR_EN) != 0)
    {
      ret = -EBUSY;
      goto out;
    }

  if (channel->request.direction != STM32_DMA_MEMORY_TO_MEMORY)
    {
      for (i = 0; i < STM32_DMA_CHANNELS; i++)
        {
          other = &g_dma_channels[i];
          if (other != channel && other->allocated && other->in_flight &&
              other->controller == channel->controller &&
              other->request.request == channel->request.request)
            {
              ret = -EBUSY;
              goto out;
            }
        }
    }

  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   STM32_DMA_FLAG_CLEAR_MASK);
  channel->status = 0;
  channel->error = 0;
  channel->in_flight = true;
  control = (channel->config.priority << STM32_DMA_CR_PRIO_SHIFT) |
            STM32_DMA_INTERRUPT_MASK | STM32_DMA_CR_EN;
  stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel), control);

out:
  leave_critical_section(flags);
  return ret;
}

int stm32_dmastop(DMA_HANDLE handle)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  irqstate_t flags;
  uint32_t control;
  volatile unsigned int timeout;
  bool was_enabled;
  int ret = 0;

  if (channel == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!channel->allocated)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  control = stm32_dma_getreg(channel,
                             STM32_DMA_CXCR_OFFSET(channel->channel));
  was_enabled = (control & STM32_DMA_CR_EN) != 0;
  if (was_enabled)
    {
      up_disable_irq(channel->irq);
      stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel),
                       (control & ~STM32_DMA_INTERRUPT_MASK) |
                       STM32_DMA_CR_SUSP);
    }

  leave_critical_section(flags);

  if (was_enabled)
    {
      for (timeout = STM32_DMA_WAIT_LOOPS; timeout > 0; timeout--)
        {
          if ((stm32_dma_getreg(channel,
                                STM32_DMA_CXSR_OFFSET(channel->channel)) &
               STM32_DMA_FLAG_SUSPF) != 0)
            {
              break;
            }
        }

      if (timeout == 0)
        {
          flags = enter_critical_section();
          channel->error = -ETIMEDOUT;
          leave_critical_section(flags);
          up_enable_irq(channel->irq);
          return -ETIMEDOUT;
        }
    }

  /* RM0486 requires reset only after the channel has reached SUSPF. */

  stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel),
                   STM32_DMA_CR_RESET);
  for (timeout = STM32_DMA_WAIT_LOOPS; timeout > 0; timeout--)
    {
      control = stm32_dma_getreg(channel,
                                 STM32_DMA_CXCR_OFFSET(channel->channel));
      if ((control & (STM32_DMA_CR_EN | STM32_DMA_CR_SUSP)) == 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      flags = enter_critical_section();
      channel->error = -ETIMEDOUT;
      leave_critical_section(flags);
      up_enable_irq(channel->irq);
      return -ETIMEDOUT;
    }

  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   STM32_DMA_FLAG_CLEAR_MASK);
  stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel), 0);

  if (was_enabled)
    {
      up_enable_irq(channel->irq);
    }

  flags = enter_critical_section();
  channel->in_flight = false;
  channel->configured = false;
  if (was_enabled)
    {
      channel->status |= DMA_STATUS_SUSPF;
    }

  channel->error = ret;
  leave_critical_section(flags);
  return ret;
}

int stm32_dmastatus(DMA_HANDLE handle, struct stm32_dma_status_s *status)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  irqstate_t flags;
  int ret = 0;

  if (channel == NULL || status == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (!channel->allocated)
    {
      ret = -EINVAL;
    }
  else
    {
      status->flags = channel->status;
      status->remaining = channel->configured ?
        stm32_dma_getreg(channel,
                         STM32_DMA_CXBR1_OFFSET(channel->channel)) &
        STM32_DMA_BR1_BNDT_MASK : 0;
      status->error = channel->error;
      status->in_flight = channel->in_flight;
    }

  leave_critical_section(flags);
  return ret;
}
