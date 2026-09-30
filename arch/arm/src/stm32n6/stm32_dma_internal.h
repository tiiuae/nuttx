/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dma_internal.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_DMA_INTERNAL_H
#define __ARCH_ARM_SRC_STM32N6_STM32_DMA_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/stm32n6xxx_dma.h"
#include "stm32_dma.h"

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint8_t stm32_dma_status_from_register(uint32_t status)
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

static inline unsigned int stm32_dma_find_free_channel(uint32_t used_mask,
                                                       unsigned int count,
                                                       unsigned int reserved)
{
  unsigned int channel;

  if (count > 32 || reserved > count)
    {
      return UINT32_MAX;
    }

  for (channel = reserved; channel < count; channel++)
    {
      if ((used_mask & (1u << channel)) == 0)
        {
          return channel;
        }
    }

  return count;
}

static inline bool stm32_dma_cache_range_valid(uintptr_t address,
                                               size_t length,
                                               size_t linesize)
{
  if (length == 0 || address > UINTPTR_MAX - length)
    {
      return false;
    }

  if (linesize == 0)
    {
      return true;
    }

  return address % linesize == 0 && length % linesize == 0;
}

static inline uint32_t stm32_dma_lli_link(uintptr_t address, size_t index,
                                          size_t count,
                                          enum stm32_dma_list_mode_e mode,
                                          uint32_t update_mask)
{
  uintptr_t next;

  if (index + 1 < count)
    {
      next = address + (index + 1) * sizeof(struct stm32_dma_lli_s);
    }
  else if (mode == STM32_DMA_LIST_CIRCULAR ||
           mode == STM32_DMA_LIST_PINGPONG)
    {
      next = address;
    }
  else
    {
      return 0;
    }

  return update_mask |
         ((next - (address & STM32_DMA_LBAR_MASK)) &
          STM32_DMA_LLR_LA_MASK);
}

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_DMA_INTERNAL_H */
