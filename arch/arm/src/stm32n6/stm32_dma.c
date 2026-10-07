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
#include <nuttx/cache.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_dma.h"
#include "hardware/stm32n6xxx_dmasigmap.h"
#include "hardware/stm32n6xxx_gpdma.h"
#include "hardware/stm32n6xxx_hpdma.h"
#include "stm32_dma.h"
#include "stm32_dma_internal.h"

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

enum stm32_dma_abort_state_e
{
  STM32_DMA_ABORT_NONE = 0,
  STM32_DMA_ABORT_VALID,
  STM32_DMA_ABORT_UNKNOWN
};

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
  bool starting;
  enum stm32_dma_abort_state_e abort_state;
  size_t abort_transferred; /* Only meaningful in STM32_DMA_ABORT_VALID */
  struct stm32_dma_request_s request;
  struct stm32_dma_config_s config;
  uint32_t tr1;
  struct stm32_dma_lli_s *descriptors;
  size_t descriptor_count;
  size_t descriptor_index;
  enum stm32_dma_list_mode_e list_mode;
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
static bool stm32_dma_cache_valid(struct stm32_dma_channel_s *channel);
static void stm32_dma_cache_prepare(struct stm32_dma_channel_s *channel);
static void stm32_dma_cache_complete(struct stm32_dma_channel_s *channel,
                                     size_t index);
static void stm32_dma_cache_invalidate_current(
  struct stm32_dma_channel_s *channel);
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
       config->nbytes >
         UINT32_MAX - config->source_address) ||
      (!config->source_increment &&
       config->width > UINT32_MAX - config->source_address) ||
      (config->destination_increment &&
       config->nbytes >
         UINT32_MAX - config->destination_address) ||
      (!config->destination_increment &&
       config->width > UINT32_MAX - config->destination_address))
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

static void stm32_dma_cache_clean(uintptr_t address, size_t length)
{
  if (length != 0)
    {
      up_clean_dcache(address, address + length);
    }
}

static void stm32_dma_cache_invalidate(uintptr_t address, size_t length)
{
  if (length != 0)
    {
      up_invalidate_dcache(address, address + length);
    }
}

static void stm32_dma_cache_transfer(
  struct stm32_dma_channel_s *channel, uintptr_t source,
  uintptr_t destination, size_t nbytes, uint32_t tr1, bool complete)
{
  size_t source_length = nbytes;
  size_t destination_length = nbytes;
  size_t source_width =
    1u << ((tr1 & STM32_DMA_TR1_SDW_MASK) >>
           STM32_DMA_TR1_SDW_SHIFT);
  size_t destination_width =
    1u << ((tr1 & STM32_DMA_TR1_DDW_MASK) >>
           STM32_DMA_TR1_DDW_SHIFT);

  if ((tr1 & STM32_DMA_TR1_SINC) == 0)
    {
      source_length = source_width;
    }

  if ((tr1 & STM32_DMA_TR1_DINC) == 0)
    {
      destination_length = destination_width;
    }

  if (!complete &&
      (channel->request.direction == STM32_DMA_MEMORY_TO_PERIPHERAL ||
       channel->request.direction == STM32_DMA_MEMORY_TO_MEMORY))
    {
      stm32_dma_cache_clean(source, source_length);
    }

  if (channel->request.direction == STM32_DMA_PERIPHERAL_TO_MEMORY ||
      channel->request.direction == STM32_DMA_MEMORY_TO_MEMORY)
    {
      if (complete)
        {
          stm32_dma_cache_invalidate(destination, destination_length);
        }
      else
        {
          /* Flush old dirty lines before the DMA writes memory. */

          stm32_dma_cache_clean(destination, destination_length);
        }
    }
}

static bool stm32_dma_cache_valid(struct stm32_dma_channel_s *channel)
{
  const struct stm32_dma_config_s *config;
  uintptr_t destination;
  size_t length;
  size_t width;
  size_t i;

  if (channel->request.direction != STM32_DMA_PERIPHERAL_TO_MEMORY &&
      channel->request.direction != STM32_DMA_MEMORY_TO_MEMORY)
    {
      return true;
    }

  if (channel->descriptor_count == 0)
    {
      config = &channel->config;
      destination = config->destination_address;
      length = config->destination_increment ? config->nbytes :
                                               config->width;
      return stm32_dma_cache_range_valid(destination, length,
                                         up_get_dcache_linesize());
    }

  for (i = 0; i < channel->descriptor_count; i++)
    {
      struct stm32_dma_lli_s *descriptor = &channel->descriptors[i];
      uint32_t tr1 = descriptor->tr1;
      uint32_t bytes = descriptor->br1 & STM32_DMA_BR1_BNDT_MASK;

      width = 1u << ((tr1 & STM32_DMA_TR1_DDW_MASK) >>
                     STM32_DMA_TR1_DDW_SHIFT);
      destination = descriptor->dar;
      length = (tr1 & STM32_DMA_TR1_DINC) != 0 ? bytes : width;
      if (!stm32_dma_cache_range_valid(destination, length,
                                       up_get_dcache_linesize()))
        {
          return false;
        }
    }

  return true;
}

static void stm32_dma_cache_prepare(struct stm32_dma_channel_s *channel)
{
  size_t i;

  if (channel->descriptor_count == 0)
    {
      stm32_dma_cache_transfer(channel,
                               channel->config.source_address,
                               channel->config.destination_address,
                               channel->config.nbytes,
                               channel->tr1,
                               false);
      return;
    }

  stm32_dma_cache_clean((uintptr_t)channel->descriptors,
                        channel->descriptor_count *
                        sizeof(*channel->descriptors));

  for (i = 0; i < channel->descriptor_count; i++)
    {
      struct stm32_dma_lli_s *descriptor = &channel->descriptors[i];
      stm32_dma_cache_transfer(channel, descriptor->sar, descriptor->dar,
                               descriptor->br1 & STM32_DMA_BR1_BNDT_MASK,
                               descriptor->tr1, false);
    }
}

static void stm32_dma_cache_complete(struct stm32_dma_channel_s *channel,
                                    size_t index)
{
  uint32_t tr1;
  uint32_t nbytes;
  uintptr_t source;
  uintptr_t destination;

  if (channel->descriptor_count == 0)
    {
      tr1 = channel->tr1;
      stm32_dma_cache_transfer(channel, channel->config.source_address,
                               channel->config.destination_address,
                               channel->config.nbytes, tr1, true);
      return;
    }

  if (index >= channel->descriptor_count)
    {
      return;
    }

  tr1 = channel->descriptors[index].tr1;
  nbytes = channel->descriptors[index].br1 & STM32_DMA_BR1_BNDT_MASK;
  source = channel->descriptors[index].sar;
  destination = channel->descriptors[index].dar;
  stm32_dma_cache_transfer(channel, source, destination, nbytes, tr1, true);
}

static void stm32_dma_cache_invalidate_current(
  struct stm32_dma_channel_s *channel)
{
  stm32_dma_cache_complete(channel, channel->descriptor_index);
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
  status = stm32_dma_status_from_register(raw_status);

  if ((status & (DMA_STATUS_TCF | DMA_STATUS_FATAL)) != 0)
    {
      stm32_dma_cache_invalidate_current(channel);
    }

  flags = enter_critical_section();
  channel->status |= status;
  if ((status & DMA_STATUS_FATAL) != 0)
    {
      channel->error = -EIO;
      channel->in_flight = false;
    }
  else if ((status & DMA_STATUS_TCF) != 0)
    {
      if (channel->descriptor_count == 0)
        {
          channel->in_flight = false;
        }
      else if (channel->list_mode == STM32_DMA_LIST_CIRCULAR ||
               channel->list_mode == STM32_DMA_LIST_PINGPONG)
        {
          channel->descriptor_index =
            (channel->descriptor_index + 1) % channel->descriptor_count;
        }
      else if (channel->descriptor_index + 1 < channel->descriptor_count)
        {
          channel->descriptor_index++;
        }
      else
        {
          channel->in_flight = false;
        }
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
      channel->starting = false;
      channel->descriptors = NULL;
      channel->descriptor_count = 0;
      channel->descriptor_index = 0;
      channel->list_mode = STM32_DMA_LIST_TERMINAL;
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
  uint32_t used_mask = 0;
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
  for (i = 0; i < count; i++)
    {
      channel = &g_dma_channels[first + i];

      if (!channel->initialized || channel->allocated)
        {
          used_mask |= 1u << i;
        }
    }

  i = stm32_dma_find_free_channel(used_mask, count, reserved);
  if (i < count)
    {
      channel = &g_dma_channels[first + i];
      channel->allocated = true;
      channel->request = *request;
      channel->configured = false;
      channel->in_flight = false;
      channel->starting = false;
      channel->descriptors = NULL;
      channel->descriptor_count = 0;
      channel->descriptor_index = 0;
      channel->list_mode = STM32_DMA_LIST_TERMINAL;
      channel->callback = NULL;
      channel->callback_arg = NULL;
      channel->status = 0;
      channel->error = 0;
      handle = (DMA_HANDLE)channel;
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
  else if (channel->starting)
    {
      ret = -EBUSY;
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
      channel->descriptors = NULL;
      channel->descriptor_count = 0;
      channel->descriptor_index = 0;
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
  if (channel->channel >= STM32_GPDMA1_2D_FIRST_CHANNEL)
    {
      stm32_dma_putreg(channel, STM32_DMA_CXTR3_OFFSET(channel->channel), 0);
      stm32_dma_putreg(channel, STM32_DMA_CXBR2_OFFSET(channel->channel), 0);
    }

  stm32_dma_putreg(channel, STM32_DMA_CXBR1_OFFSET(channel->channel),
                   config->nbytes);
  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   STM32_DMA_FLAG_CLEAR_MASK);

  channel->config = *config;
  channel->tr1 = tr1;
  channel->descriptors = NULL;
  channel->descriptor_count = 0;
  channel->descriptor_index = 0;
  channel->list_mode = STM32_DMA_LIST_TERMINAL;
  channel->configured = true;
  channel->abort_state = STM32_DMA_ABORT_NONE;
  channel->status = 0;
  channel->error = 0;

out:
  leave_critical_section(flags);
  return ret;
}

int stm32_dmallibuild(DMA_HANDLE handle,
                      const struct stm32_dma_config_s *configs,
                      size_t count, struct stm32_dma_lli_s *descriptors,
                      size_t capacity, enum stm32_dma_list_mode_e mode)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  uintptr_t address;
  uintptr_t last;
  uint32_t tr1;
  uint32_t tr2;
  uint32_t link;
  uint32_t update_mask;
  bool extended;
  size_t i;
  irqstate_t flags;
  int ret = 0;

  if (channel == NULL || configs == NULL || descriptors == NULL ||
      count == 0 || count > capacity ||
      count > 0x10000u / sizeof(*descriptors) ||
      (mode != STM32_DMA_LIST_TERMINAL &&
       mode != STM32_DMA_LIST_CIRCULAR &&
       mode != STM32_DMA_LIST_PINGPONG) ||
      (mode == STM32_DMA_LIST_PINGPONG && count != 2))
    {
      return -EINVAL;
    }

  address = (uintptr_t)descriptors;
  if ((address & (STM32_DMA_LLI_ALIGNMENT - 1)) != 0 ||
      address > UINT32_MAX ||
      count * sizeof(*descriptors) > UINT32_MAX - address)
    {
      return -EINVAL;
    }

  last = address + count * sizeof(*descriptors) - 1;
  if ((address & STM32_DMA_LBAR_MASK) != (last & STM32_DMA_LBAR_MASK))
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

  extended = channel->channel >= STM32_GPDMA1_2D_FIRST_CHANNEL;
  update_mask = extended ? STM32_DMA_LLR_UPDATE_MASK_2D :
                           STM32_DMA_LLR_UPDATE_MASK;

  for (i = 0; i < count; i++)
    {
      if (configs[i].priority != configs[0].priority)
        {
          ret = -EINVAL;
          goto out;
        }

      ret = stm32_dma_check_config(channel, &configs[i], &tr1, &tr2);
      if (ret < 0)
        {
          goto out;
        }
    }

  for (i = 0; i < count; i++)
    {
      struct stm32_dma_lli_s *descriptor = &descriptors[i];

      ret = stm32_dma_check_config(channel, &configs[i], &tr1, &tr2);
      if (ret < 0)
        {
          goto out;
        }

      memset(descriptor, 0, sizeof(*descriptor));
      descriptor->tr1 = tr1;
      descriptor->tr2 = tr2 | STM32_DMA_TR2_TCEM_LLI;
      descriptor->br1 = configs[i].nbytes;
      descriptor->sar = configs[i].source_address;
      descriptor->dar = configs[i].destination_address;

      link = stm32_dma_lli_link(address, i, count, mode, update_mask);

      if (extended)
        {
          descriptor->tail.extended.tr3 = 0;
          descriptor->tail.extended.br2 = 0;
          descriptor->tail.extended.llr = link;
        }
      else
        {
          descriptor->tail.llr = link;
        }
    }

  channel->config = configs[0];
  channel->tr1 = descriptors[0].tr1;
  channel->descriptors = descriptors;
  channel->descriptor_count = count;
  channel->descriptor_index = 0;
  channel->list_mode = mode;
  channel->configured = true;
  channel->status = 0;
  channel->error = 0;

  stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel), 0);
  stm32_dma_putreg(channel, STM32_DMA_CXLBAR_OFFSET(channel->channel),
                   address & STM32_DMA_LBAR_MASK);
  stm32_dma_putreg(channel, STM32_DMA_CXSAR_OFFSET(channel->channel),
                   descriptors[0].sar);
  stm32_dma_putreg(channel, STM32_DMA_CXDAR_OFFSET(channel->channel),
                   descriptors[0].dar);
  stm32_dma_putreg(channel, STM32_DMA_CXTR1_OFFSET(channel->channel),
                   descriptors[0].tr1);
  stm32_dma_putreg(channel, STM32_DMA_CXTR2_OFFSET(channel->channel),
                   descriptors[0].tr2);
  if (extended)
    {
      stm32_dma_putreg(channel, STM32_DMA_CXTR3_OFFSET(channel->channel), 0);
      stm32_dma_putreg(channel, STM32_DMA_CXBR2_OFFSET(channel->channel), 0);
    }

  stm32_dma_putreg(channel, STM32_DMA_CXBR1_OFFSET(channel->channel),
                   descriptors[0].br1);
  stm32_dma_putreg(channel, STM32_DMA_CXLLR_OFFSET(channel->channel),
                   extended ? descriptors[0].tail.extended.llr :
                              descriptors[0].tail.llr);
  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   STM32_DMA_FLAG_CLEAR_MASK);

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

  if (!stm32_dma_cache_valid(channel))
    {
      ret = -EINVAL;
      goto out;
    }

  channel->starting = true;
  channel->in_flight = true;
  leave_critical_section(flags);

  stm32_dma_cache_prepare(channel);

  flags = enter_critical_section();
  channel->starting = false;
  stm32_dma_putreg(channel, STM32_DMA_CXFCR_OFFSET(channel->channel),
                   STM32_DMA_FLAG_CLEAR_MASK);
  channel->status = 0;
  channel->error = 0;
  channel->descriptor_index = 0;
  channel->abort_state = STM32_DMA_ABORT_NONE;
  control = (channel->config.priority << STM32_DMA_CR_PRIO_SHIFT) |
            STM32_DMA_INTERRUPT_MASK | STM32_DMA_CR_EN;
  stm32_dma_putreg(channel, STM32_DMA_CXCR_OFFSET(channel->channel),
                   control);

out:
  leave_critical_section(flags);
  return ret;
}

static int stm32_dma_stop(DMA_HANDLE handle, size_t *transferred)
{
  struct stm32_dma_channel_s *channel = stm32_dma_getchannel(handle);
  irqstate_t flags;
  uint32_t control;
  volatile unsigned int timeout;
  bool was_enabled;
  bool was_active;
  size_t completed = 0;
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

  if (channel->starting)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  if (transferred != NULL &&
      (!channel->configured || channel->descriptor_count != 0 ||
       channel->request.direction != STM32_DMA_MEMORY_TO_PERIPHERAL))
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  control = stm32_dma_getreg(channel,
                             STM32_DMA_CXCR_OFFSET(channel->channel));
  was_active = channel->in_flight;
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

  if (transferred != NULL &&
      channel->abort_state == STM32_DMA_ABORT_VALID)
    {
      /* A reset timeout may have destroyed the hardware counters. */

      completed = channel->abort_transferred;
    }
  else if (transferred != NULL)
    {
      size_t remaining;
      size_t buffered;
      uint32_t status;
      uint32_t fifo_mask;

      status = stm32_dma_getreg(channel,
                                STM32_DMA_CXSR_OFFSET(channel->channel));
      if (channel->abort_state == STM32_DMA_ABORT_UNKNOWN ||
          (channel->status & DMA_STATUS_DTEF) != 0 ||
          (status & STM32_DMA_FLAG_DTEF) != 0)
        {
          /* A bus error does not identify whether the failing destination
           * write had side effects. Do not invent an exact retry boundary.
           */

          channel->error = -EIO;
          channel->abort_state = STM32_DMA_ABORT_UNKNOWN;
          up_enable_irq(channel->irq);
          return -EIO;
        }

      fifo_mask = channel->controller == STM32_DMA_CONTROLLER_GPDMA1 ?
                  STM32_GPDMA_SR_FIFOL_MASK : STM32_HPDMA_SR_FIFOL_MASK;
      remaining = stm32_dma_getreg(channel,
                       STM32_DMA_CXBR1_OFFSET(channel->channel)) &
                  STM32_DMA_BR1_BNDT_MASK;
      buffered = ((status & fifo_mask) >> STM32_DMA_SR_FIFOL_SHIFT) *
                 channel->config.width;
      if (remaining + buffered > channel->config.nbytes)
        {
          channel->error = -EIO;
          channel->abort_state = STM32_DMA_ABORT_UNKNOWN;
          up_enable_irq(channel->irq);
          return -EIO;
        }

      completed = channel->config.nbytes - remaining - buffered;
      channel->abort_transferred = completed;
      channel->abort_state = STM32_DMA_ABORT_VALID;
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

  if (was_active)
    {
      stm32_dma_cache_invalidate_current(channel);
    }

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
  if (transferred != NULL)
    {
      *transferred = completed;
    }

  return ret;
}

int stm32_dmastop(DMA_HANDLE handle)
{
  return stm32_dma_stop(handle, NULL);
}

int stm32_dmaabort(DMA_HANDLE handle, size_t *transferred)
{
  if (transferred == NULL)
    {
      return -EINVAL;
    }

  return stm32_dma_stop(handle, transferred);
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
