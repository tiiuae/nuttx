/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_dma_helpers_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../stm32_dma_internal.h"

#define CHECK(condition) \
  do \
    { \
      if (!(condition)) \
        { \
          fprintf(stderr, "%s:%d: check failed: %s\n", \
                  __FILE__, __LINE__, #condition); \
          return EXIT_FAILURE; \
        } \
    } \
  while (0)

static int test_status_translation(void)
{
  const uint32_t hw_flags[] =
  {
    STM32_DMA_FLAG_TCF,
    STM32_DMA_FLAG_HTF,
    STM32_DMA_FLAG_DTEF,
    STM32_DMA_FLAG_ULEF,
    STM32_DMA_FLAG_USEF,
    STM32_DMA_FLAG_SUSPF,
    STM32_DMA_FLAG_TOF
  };
  const uint8_t dma_flags[] =
  {
    DMA_STATUS_TCF,
    DMA_STATUS_HTF,
    DMA_STATUS_DTEF,
    DMA_STATUS_ULEF,
    DMA_STATUS_USEF,
    DMA_STATUS_SUSPF,
    DMA_STATUS_TOF
  };
  uint32_t all_hw_flags = 0;
  uint8_t all_dma_flags = 0;
  size_t i;

  for (i = 0; i < sizeof(hw_flags) / sizeof(hw_flags[0]); i++)
    {
      CHECK(stm32_dma_status_from_register(hw_flags[i]) == dma_flags[i]);
      all_hw_flags |= hw_flags[i];
      all_dma_flags |= dma_flags[i];
    }

  CHECK(stm32_dma_status_from_register(all_hw_flags) == all_dma_flags);
  CHECK(stm32_dma_status_from_register(STM32_DMA_FLAG_IDLEF) == 0);
  CHECK(stm32_dma_status_from_register(UINT32_MAX) == all_dma_flags);
  return EXIT_SUCCESS;
}

static int test_cache_ranges(void)
{
  CHECK(stm32_dma_cache_range_valid(0x20001000, 64, 32));
  CHECK(!stm32_dma_cache_range_valid(0x20001004, 64, 32));
  CHECK(!stm32_dma_cache_range_valid(0x20001000, 63, 32));
  CHECK(!stm32_dma_cache_range_valid(0x20001000, 0, 32));
  CHECK(!stm32_dma_cache_range_valid(UINTPTR_MAX - 15, 32, 32));
  CHECK(stm32_dma_cache_range_valid(0x20001004, 17, 0));
  return EXIT_SUCCESS;
}

struct allocation_test_s
{
  pthread_mutex_t lock;
  uint32_t used_mask;
  unsigned int count;
  unsigned int reserved;
  unsigned int claimed;
};

static void *allocation_worker(void *arg)
{
  struct allocation_test_s *test = arg;
  unsigned int channel;

  pthread_mutex_lock(&test->lock);
  channel = stm32_dma_find_free_channel(test->used_mask, test->count,
                                        test->reserved);
  if (channel < test->count)
    {
      test->used_mask |= 1u << channel;
      test->claimed++;
    }

  pthread_mutex_unlock(&test->lock);
  return NULL;
}

static int test_concurrent_channel_selection(void)
{
  enum
  {
    CHANNELS = 16,
    RESERVED = 3,
    WORKERS = 32
  };
  struct allocation_test_s test =
  {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .used_mask = 0,
    .count = CHANNELS,
    .reserved = RESERVED,
    .claimed = 0
  };
  pthread_t workers[WORKERS];
  size_t created = 0;
  size_t i;

  for (i = 0; i < WORKERS; i++)
    {
      if (pthread_create(&workers[i], NULL, allocation_worker, &test) != 0)
        {
          break;
        }

      created++;
    }

  for (i = 0; i < created; i++)
    {
      CHECK(pthread_join(workers[i], NULL) == 0);
    }

  CHECK(created == WORKERS);
  CHECK(test.claimed == CHANNELS - RESERVED);
  CHECK((test.used_mask & ((1u << RESERVED) - 1)) == 0);
  CHECK(test.used_mask == (((1u << CHANNELS) - 1) &
                           ~((1u << RESERVED) - 1)));
  CHECK(stm32_dma_find_free_channel(test.used_mask, CHANNELS, RESERVED) ==
        CHANNELS);
  CHECK(stm32_dma_find_free_channel(0, 33, 0) == UINT32_MAX);
  return EXIT_SUCCESS;
}

static int test_link_encoding(void)
{
  struct stm32_dma_lli_s descriptors[3];
  uintptr_t address = 0x20010020;
  uint32_t update_mask = STM32_DMA_LLR_UPDATE_MASK;
  uint32_t update_mask_2d = STM32_DMA_LLR_UPDATE_MASK_2D;
  uint32_t link_base_offset = address & ~STM32_DMA_LBAR_MASK;
  uint32_t next_offset = link_base_offset +
                         sizeof(struct stm32_dma_lli_s);

  CHECK(stm32_dma_lli_link(address, 0, 3, STM32_DMA_LIST_TERMINAL,
                           update_mask) ==
        (update_mask | next_offset));
  CHECK(stm32_dma_lli_link(address, 1, 3, STM32_DMA_LIST_TERMINAL,
                           update_mask) ==
        (update_mask | (link_base_offset +
                        2 * sizeof(struct stm32_dma_lli_s))));
  CHECK(stm32_dma_lli_link(address, 2, 3, STM32_DMA_LIST_TERMINAL,
                           update_mask) == 0);
  CHECK(stm32_dma_lli_link(address, 2, 3, STM32_DMA_LIST_CIRCULAR,
                           update_mask) == (update_mask | link_base_offset));
  CHECK(stm32_dma_lli_link(address, 1, 2, STM32_DMA_LIST_PINGPONG,
                           update_mask_2d) ==
        (update_mask_2d | link_base_offset));
  CHECK(stm32_dma_lli_link(address, 0, 1, STM32_DMA_LIST_CIRCULAR,
                           update_mask) == (update_mask | link_base_offset));
  CHECK((address & STM32_DMA_LBAR_MASK) == 0x20010000);
  CHECK(sizeof(descriptors) <= 0x10000);
  return EXIT_SUCCESS;
}

int main(void)
{
  if (test_status_translation() != EXIT_SUCCESS ||
      test_cache_ranges() != EXIT_SUCCESS ||
      test_concurrent_channel_selection() != EXIT_SUCCESS ||
      test_link_encoding() != EXIT_SUCCESS)
    {
      return EXIT_FAILURE;
    }

  puts("STM32N6 DMA helper tests passed");
  return EXIT_SUCCESS;
}
