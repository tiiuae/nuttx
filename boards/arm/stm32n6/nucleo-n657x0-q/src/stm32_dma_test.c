/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_dma_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <syslog.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_dmasigmap.h"
#include "hardware/stm32n6xxx_gpdma.h"
#include "hardware/stm32n6xxx_hpdma.h"
#include "hardware/stm32n6xxx_uart.h"
#include "stm32_dma.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_DMA_TEST_NWORDS       64
#define STM32_DMA_TEST_LLI_COUNT    3
#define STM32_DMA_TEST_LLI_NWORDS   4096
#define STM32_DMA_TEST_LLI_NBYTES   \
  (STM32_DMA_TEST_LLI_NWORDS * sizeof(uint32_t))
#define STM32_DMA_TEST_WAIT_LOOPS   100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_dma_test_callback_s
{
  volatile uint32_t count;
  volatile uint32_t tcf_count;
  volatile uint8_t status;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_dma_test_source[STM32_DMA_TEST_NWORDS]
  __attribute__((aligned(256)));
static uint32_t g_dma_test_destination[STM32_DMA_TEST_NWORDS]
  __attribute__((aligned(256)));
static uint32_t
  g_dma_test_lli_source[STM32_DMA_TEST_LLI_COUNT][STM32_DMA_TEST_LLI_NWORDS]
  __attribute__((aligned(256)));
static uint32_t
  g_dma_test_lli_destination[STM32_DMA_TEST_LLI_COUNT]
                            [STM32_DMA_TEST_LLI_NWORDS]
  __attribute__((aligned(256)));
static struct stm32_dma_lli_s g_dma_test_lli_descriptors
  [STM32_DMA_TEST_LLI_COUNT] __attribute__((aligned(256)));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stm32_dma_test_callback(DMA_HANDLE handle, uint8_t status,
                                    void *arg)
{
  struct stm32_dma_test_callback_s *callback = arg;

  UNUSED(handle);
  callback->status |= status;
  callback->count++;
  if ((status & DMA_STATUS_TCF) != 0)
    {
      callback->tcf_count++;
    }
}

static int stm32_dma_test_copy(enum stm32_dma_controller_e controller,
                               unsigned int width)
{
  struct stm32_dma_test_callback_s callback =
  {
    .count = 0,
    .tcf_count = 0,
    .status = 0
  };
  struct stm32_dma_request_s request =
  {
    .controller = controller,
    .direction = STM32_DMA_MEMORY_TO_MEMORY,
    .request = STM32_DMA_REQUEST_NONE,
    .peripheral_address = 0
  };
  struct stm32_dma_config_s config =
  {
    .source_address = (uintptr_t)g_dma_test_source,
    .destination_address = (uintptr_t)g_dma_test_destination,
    .nbytes = sizeof(g_dma_test_source),
    .width = width,
    .priority = 1,
    .source_increment = true,
    .destination_increment = true
  };
  struct stm32_dma_status_s status;
  DMA_HANDLE handle;
  unsigned int i;
  unsigned int mismatch;
  int ret = 0;
  bool started = false;

  for (i = 0; i < STM32_DMA_TEST_NWORDS; i++)
    {
      g_dma_test_source[i] = 0x13570000 | (i * 0x101);
      g_dma_test_destination[i] = 0;
    }

  handle = stm32_dmachannel(&request);
  if (handle == NULL)
    {
      syslog(LOG_ERR, "DMA core: controller %d allocation failed\n",
             controller);
      return -EBUSY;
    }

  /* Verify width, alignment, and block-size validation before programming a
   * valid direct memory-to-memory transfer.
   */

  config.width = 3;
  if (stm32_dmasetup(handle, &config) != -EINVAL)
    {
      ret = -EIO;
      goto out;
    }

  config.width = width;
  config.source_address++;
  if (stm32_dmasetup(handle, &config) != -EINVAL)
    {
      ret = -EIO;
      goto out;
    }

  config.source_address--;
  config.destination_address++;
  if (stm32_dmasetup(handle, &config) != -EINVAL)
    {
      ret = -EIO;
      goto out;
    }

  config.destination_address--;
  config.nbytes--;
  if (stm32_dmasetup(handle, &config) != -EINVAL)
    {
      ret = -EIO;
      goto out;
    }

  config.nbytes = 65536;
  if (stm32_dmasetup(handle, &config) != -EINVAL)
    {
      ret = -EIO;
      goto out;
    }

  config.nbytes = sizeof(g_dma_test_source);
  if (controller == STM32_DMA_CONTROLLER_GPDMA1)
    {
      config.width = 8;
      if (stm32_dmasetup(handle, &config) != -EINVAL)
        {
          ret = -EIO;
          goto out;
        }

      config.width = sizeof(uint32_t);
    }

  if (up_get_dcache_linesize() > config.width)
    {
      ret = stm32_dmasetup(handle, &config);
      if (ret < 0)
        {
          goto out;
        }

      config.destination_address += config.width;
      ret = stm32_dmasetup(handle, &config);
      if (ret < 0)
        {
          goto out;
        }

      if (stm32_dmastart(handle) != -EINVAL)
        {
          ret = -EIO;
          goto out;
        }

      config.destination_address -= config.width;
    }

  ret = stm32_dmacallback(handle, stm32_dma_test_callback, &callback);
  if (ret < 0)
    {
      goto out;
    }

  ret = stm32_dmasetup(handle, &config);
  if (ret < 0)
    {
      goto out;
    }

  ret = stm32_dmastart(handle);
  if (ret < 0)
    {
      goto out;
    }

  started = true;
  for (i = 0; i < STM32_DMA_TEST_WAIT_LOOPS; i++)
    {
      ret = stm32_dmastatus(handle, &status);
      if (ret < 0 || !status.in_flight)
        {
          break;
        }

      up_udelay(1);
    }

  if (ret < 0)
    {
      goto out;
    }

  if (status.in_flight)
    {
      ret = -ETIMEDOUT;
      goto out;
    }

  for (mismatch = 0; mismatch < STM32_DMA_TEST_NWORDS; mismatch++)
    {
      if (g_dma_test_source[mismatch] !=
          g_dma_test_destination[mismatch])
        {
          break;
        }
    }

  if ((status.flags & (DMA_STATUS_HTF | DMA_STATUS_TCF)) !=
        (DMA_STATUS_HTF | DMA_STATUS_TCF) ||
      status.remaining != 0 || status.error != 0 ||
      callback.count == 0 ||
      callback.tcf_count != 1 ||
      (callback.status & (DMA_STATUS_HTF | DMA_STATUS_TCF)) !=
        (DMA_STATUS_HTF | DMA_STATUS_TCF) ||
      mismatch != STM32_DMA_TEST_NWORDS)
    {
      syslog(LOG_ERR,
             "DMA core: controller %d width %u copy/status/callback failed "
             "(flags=%02lx cb=%02x count=%lu remain=%lu err=%d "
             "mismatch=%u src=%08lx dst=%08lx)\n",
             controller, width, (unsigned long)status.flags,
             (unsigned int)callback.status,
             (unsigned long)callback.count,
             (unsigned long)status.remaining, status.error, mismatch,
             mismatch < STM32_DMA_TEST_NWORDS ?
               (unsigned long)g_dma_test_source[mismatch] : 0,
             mismatch < STM32_DMA_TEST_NWORDS ?
               (unsigned long)g_dma_test_destination[mismatch] : 0);

      ret = -EIO;
      goto out;
    }

  ret = 0;

out:
  if (started)
    {
      int stopret = stm32_dmastop(handle);

      if (ret == 0 && stopret < 0)
        {
          ret = stopret;
        }
    }

  {
    int freeret = stm32_dmafree(handle);

    if (ret == 0 && freeret < 0)
      {
        ret = freeret;
      }
  }

  if (ret < 0)
    {
      syslog(LOG_ERR, "DMA core: controller %d copy test failed: %d\n",
             controller, ret);
    }
  else
    {
      syslog(LOG_INFO,
             "DMA core: controller %d width %u copy, callback, status and "
             "validation OK\n", controller, width);
    }

  return ret;
}

static int stm32_dma_test_linked_list(
  enum stm32_dma_controller_e controller,
  enum stm32_dma_list_mode_e mode, size_t count, bool extended,
  unsigned int nchannels, unsigned int reserved)
{
  struct stm32_dma_test_callback_s callback =
  {
    .count = 0,
    .tcf_count = 0,
    .status = 0
  };
  struct stm32_dma_request_s request =
  {
    .controller = controller,
    .direction = STM32_DMA_MEMORY_TO_MEMORY,
    .request = STM32_DMA_REQUEST_NONE,
    .peripheral_address = 0
  };
  struct stm32_dma_config_s configs[STM32_DMA_TEST_LLI_COUNT];
  struct stm32_dma_status_s status;
  DMA_HANDLE handles[16] = { NULL };
  DMA_HANDLE handle;
  unsigned int target;
  size_t nhandles;
  size_t i;
  size_t j;
  size_t nallocated = 0;
  int ret = 0;
  bool started = false;

  if (count == 0 || count > STM32_DMA_TEST_LLI_COUNT ||
      nchannels > 16 || reserved > nchannels)
    {
      return -EINVAL;
    }

  if (extended)
    {
      target = reserved < STM32_GPDMA1_2D_FIRST_CHANNEL ?
               STM32_GPDMA1_2D_FIRST_CHANNEL : reserved;
      if (target >= nchannels)
        {
          return -ENOSPC;
        }

      nhandles = target - reserved + 1;
    }
  else
    {
      nhandles = 1;
    }

  if (nhandles > nchannels - reserved)
    {
      return -ENOSPC;
    }

  for (i = 0; i < count; i++)
    {
      configs[i].source_address =
        (uintptr_t)g_dma_test_lli_source[i];
      configs[i].destination_address =
        (uintptr_t)g_dma_test_lli_destination[i];
      configs[i].nbytes = STM32_DMA_TEST_LLI_NBYTES;
      configs[i].width = sizeof(uint32_t);
      configs[i].priority = 1;
      configs[i].source_increment = true;
      configs[i].destination_increment = true;

      for (j = 0; j < STM32_DMA_TEST_LLI_NWORDS; j++)
        {
          g_dma_test_lli_source[i][j] =
            ((i + 1) << 24) ^ (j * 0x10201);
          g_dma_test_lli_destination[i][j] = 0;
        }
    }

  for (i = 0; i < nhandles; i++)
    {
      handles[i] = stm32_dmachannel(&request);
      if (handles[i] == NULL)
        {
          ret = -EBUSY;
          goto out;
        }

      nallocated++;
    }

  handle = handles[nhandles - 1];

  ret = stm32_dmacallback(handle, stm32_dma_test_callback, &callback);
  if (ret < 0)
    {
      goto out;
    }

  if (mode == STM32_DMA_LIST_TERMINAL &&
      stm32_dmallibuild(handle, configs, count, g_dma_test_lli_descriptors,
                        count - 1, mode) != -EINVAL)
    {
      ret = -EIO;
      goto out;
    }

  ret = stm32_dmallibuild(handle, configs, count,
                          g_dma_test_lli_descriptors,
                          STM32_DMA_TEST_LLI_COUNT, mode);
  if (ret < 0)
    {
      goto out;
    }

  syslog(LOG_INFO,
         "DMA core: controller %d list mode %d starting %u descriptors\n",
         controller, mode, (unsigned int)count);

  ret = stm32_dmastart(handle);
  if (ret < 0)
    {
      goto out;
    }

  started = true;
  for (i = 0; i < STM32_DMA_TEST_WAIT_LOOPS; i++)
    {
      ret = stm32_dmastatus(handle, &status);
      if (ret < 0)
        {
          goto out;
        }

      if ((mode == STM32_DMA_LIST_TERMINAL && !status.in_flight) ||
          (mode != STM32_DMA_LIST_TERMINAL && callback.tcf_count >= 2))
        {
          break;
        }

      up_udelay(1);
    }

  if (i == STM32_DMA_TEST_WAIT_LOOPS)
    {
      ret = -ETIMEDOUT;
      goto out;
    }

  if (mode == STM32_DMA_LIST_TERMINAL)
    {
      if (status.in_flight || status.remaining != 0 || status.error != 0 ||
          (status.flags & DMA_STATUS_TCF) == 0 ||
          callback.tcf_count != count)
        {
          ret = -EIO;
          goto out;
        }
    }
  else
    {
      if (!status.in_flight || callback.tcf_count < 2 ||
          (callback.status & DMA_STATUS_TCF) == 0)
        {
          ret = -EIO;
          goto out;
        }

      syslog(LOG_INFO,
             "DMA core: controller %d list mode %d completed a ring; "
             "requesting suspend\n", controller, mode);

      ret = stm32_dmastop(handle);
      if (ret < 0)
        {
          goto out;
        }

      syslog(LOG_INFO,
             "DMA core: controller %d list mode %d suspended\n",
             controller, mode);

      started = false;
      ret = stm32_dmastatus(handle, &status);
      if (ret < 0 || status.in_flight ||
          (status.flags & DMA_STATUS_SUSPF) == 0 || status.error != 0)
        {
          ret = ret < 0 ? ret : -EIO;
          goto out;
        }
    }

  for (i = 0; i < count; i++)
    {
      for (j = 0; j < STM32_DMA_TEST_LLI_NWORDS; j++)
        {
          if (g_dma_test_lli_source[i][j] !=
              g_dma_test_lli_destination[i][j])
            {
              syslog(LOG_ERR,
                     "DMA core: controller %d list mode %d descriptor "
                     "%u word %u mismatch\n",
                     controller, mode, (unsigned int)i, (unsigned int)j);
              ret = -EIO;
              goto out;
            }
        }
    }

  syslog(LOG_INFO,
         "DMA core: controller %d list mode %d %s-channel test OK "
         "(TCF=%lu)\n",
         controller, mode,
         (extended || reserved >= STM32_GPDMA1_2D_FIRST_CHANNEL) ?
           "extended" : "standard",
         (unsigned long)callback.tcf_count);

out:
  if (started)
    {
      int stopret = stm32_dmastop(handle);

      if (ret == 0 && stopret < 0)
        {
          ret = stopret;
        }
    }

  while (nallocated > 0)
    {
      int freeret;

      nallocated--;
      freeret = stm32_dmafree(handles[nallocated]);
      if (ret == 0 && freeret < 0)
        {
          ret = freeret;
        }
    }

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "DMA core: controller %d list mode %d test failed: %d\n",
             controller, mode, ret);
    }

  return ret;
}

static int stm32_dma_test_linked_lists(
  enum stm32_dma_controller_e controller, unsigned int nchannels,
  unsigned int reserved)
{
  unsigned int extended_channel;
  int ret = 0;

  if (stm32_dma_test_linked_list(controller, STM32_DMA_LIST_TERMINAL,
                                 STM32_DMA_TEST_LLI_COUNT, false,
                                 nchannels, reserved) < 0)
    {
      ret = -EIO;
    }

  if (stm32_dma_test_linked_list(controller, STM32_DMA_LIST_CIRCULAR,
                                 2, false, nchannels, reserved) < 0)
    {
      ret = -EIO;
    }

  if (stm32_dma_test_linked_list(controller, STM32_DMA_LIST_PINGPONG,
                                 2, false, nchannels, reserved) < 0)
    {
      ret = -EIO;
    }

  extended_channel = reserved < STM32_GPDMA1_2D_FIRST_CHANNEL ?
                     STM32_GPDMA1_2D_FIRST_CHANNEL : reserved;
  if (reserved < STM32_GPDMA1_2D_FIRST_CHANNEL &&
      extended_channel < nchannels)
    {
      if (stm32_dma_test_linked_list(controller, STM32_DMA_LIST_TERMINAL,
                                     STM32_DMA_TEST_LLI_COUNT, true,
                                     nchannels, reserved) < 0)
        {
          ret = -EIO;
        }
    }
  else if (reserved < STM32_GPDMA1_2D_FIRST_CHANNEL)
    {
      syslog(LOG_WARNING,
             "DMA core: controller %d extended-list test skipped; "
             "no channel 12+ available\n", controller);
    }

  return ret;
}

#ifdef CONFIG_STM32_USART1
static int stm32_dma_test_usart(enum stm32_dma_controller_e controller)
{
  struct stm32_dma_request_s request =
  {
    .controller = controller,
    .direction = STM32_DMA_PERIPHERAL_TO_MEMORY,
    .request = STM32_DMA_REQ_USART1_RX,
    .peripheral_address = STM32_USART1_RDR
  };
  struct stm32_dma_config_s config =
  {
    .source_address = STM32_USART1_RDR,
    .destination_address = (uintptr_t)g_dma_test_destination,
    .nbytes = sizeof(g_dma_test_destination),
    .width = 1,
    .priority = 1,
    .source_increment = false,
    .destination_increment = true
  };
  struct stm32_dma_status_s status;
  DMA_HANDLE handle;
  bool started = false;
  int ret;

  handle = stm32_dmachannel(&request);
  if (handle == NULL)
    {
      return -EBUSY;
    }

  ret = stm32_dmasetup(handle, &config);
  if (ret < 0)
    {
      goto out;
    }

  ret = stm32_dmafree(handle);
  if (ret < 0)
    {
      return ret;
    }

  request.direction = STM32_DMA_MEMORY_TO_PERIPHERAL;
  request.request = STM32_DMA_REQ_USART1_TX;
  request.peripheral_address = STM32_USART1_TDR;
  config.source_address = (uintptr_t)g_dma_test_source;
  config.destination_address = STM32_USART1_TDR;
  config.source_increment = true;
  config.destination_increment = false;
  handle = stm32_dmachannel(&request);
  if (handle == NULL)
    {
      return -EBUSY;
    }

  ret = stm32_dmasetup(handle, &config);
  if (ret < 0)
    {
      goto out;
    }

  /* DMAT must be disabled to guarantee that this abort-path test cannot
   * write test data to the console's transmit register.
   */

  if ((getreg32(STM32_USART1_CR3) & USART_CR3_DMAT) == 0)
    {
      ret = stm32_dmastart(handle);
      if (ret < 0)
        {
          goto out;
        }

      started = true;
      ret = stm32_dmastatus(handle, &status);
      if (ret < 0 || !status.in_flight)
        {
          ret = ret < 0 ? ret : -EIO;
          goto out;
        }

      if (stm32_dmasetup(handle, &config) != -EBUSY ||
          stm32_dmafree(handle) != -EBUSY)
        {
          ret = -EIO;
          goto out;
        }

      ret = stm32_dmastop(handle);
      if (ret < 0)
        {
          started = false;
          goto out;
        }

      started = false;
      ret = stm32_dmastatus(handle, &status);
      if (ret < 0 || status.in_flight ||
          (status.flags & DMA_STATUS_SUSPF) == 0 || status.error != 0)
        {
          ret = ret < 0 ? ret : -EIO;
          goto out;
        }

      syslog(LOG_INFO, "DMA core: controller %d suspend/abort OK\n",
             controller);
    }
  else
    {
      syslog(LOG_WARNING,
             "DMA core: skipping USART1 abort test while DMAT is enabled\n");
    }

  ret = 0;

out:
  if (started)
    {
      int stopret = stm32_dmastop(handle);

      if (ret == 0 && stopret < 0)
        {
          ret = stopret;
        }
    }

  {
    int freeret = stm32_dmafree(handle);

    if (ret == 0 && freeret < 0)
      {
        ret = freeret;
      }
  }

  if (ret < 0)
    {
      syslog(LOG_ERR, "DMA core: controller %d USART test failed: %d\n",
             controller, ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma_policy_test
 *
 * Description:
 *   Verify DMA channel security policy and step 3 direct-transfer API
 *   behavior on the enabled controllers.
 *
 ****************************************************************************/

int stm32_dma_policy_test(void)
{
  int ret = 0;
  struct stm32_dma_request_s invalid_request =
  {
#ifdef CONFIG_STM32_GPDMA1
    .controller = STM32_DMA_CONTROLLER_GPDMA1,
#else
    .controller = STM32_DMA_CONTROLLER_HPDMA1,
#endif
    .direction = STM32_DMA_MEMORY_TO_PERIPHERAL,
    .request = STM32_DMA_REQ_USART1_RX,
    .peripheral_address = STM32_USART1_TDR
  };

#ifndef CONFIG_ARCH_DCACHE
  syslog(LOG_WARNING,
         "DMA core: D-cache is disabled; cache-coherency checks are skipped\n");
#endif

  if (stm32_dmachannel(&invalid_request) != NULL)
    {
      syslog(LOG_ERR, "DMA core: accepted an incompatible request direction\n");
      return -EIO;
    }

  invalid_request.request = STM32_DMA_REQUEST_NONE;
  if (stm32_dmachannel(&invalid_request) != NULL)
    {
      syslog(LOG_ERR, "DMA core: accepted an invalid request ID\n");
      return -EIO;
    }

#ifdef CONFIG_STM32_GPDMA1
  uint32_t expected_gpdma1 =
    (1u << CONFIG_STM32_GPDMA1_NCHANNELS) - 1u;
  uint32_t secure_gpdma1 = getreg32(STM32_GPDMA1_SECCFGR);
  uint32_t privileged_gpdma1 = getreg32(STM32_GPDMA1_PRIVCFGR);

  if (secure_gpdma1 != expected_gpdma1 ||
      privileged_gpdma1 != expected_gpdma1)
    {
      syslog(LOG_ERR,
             "DMA policy: GPDMA1 expected %08lx/%08lx, got %08lx/%08lx\n",
             (unsigned long)expected_gpdma1,
             (unsigned long)expected_gpdma1,
             (unsigned long)secure_gpdma1,
             (unsigned long)privileged_gpdma1);
      ret = -EIO;
    }
  else
    {
      syslog(LOG_INFO, "DMA policy: GPDMA1 channel mask OK: %08lx\n",
             (unsigned long)expected_gpdma1);
    }
#endif

#ifdef CONFIG_STM32_HPDMA1
  uint32_t expected_hpdma1 =
    (1u << CONFIG_STM32_HPDMA1_NCHANNELS) - 1u;
  uint32_t secure_hpdma1 = getreg32(STM32_HPDMA1_SECCFGR);
  uint32_t privileged_hpdma1 = getreg32(STM32_HPDMA1_PRIVCFGR);
  unsigned int channel;

  if (secure_hpdma1 != expected_hpdma1 ||
      privileged_hpdma1 != expected_hpdma1)
    {
      syslog(LOG_ERR,
             "DMA policy: HPDMA1 expected %08lx/%08lx, got %08lx/%08lx\n",
             (unsigned long)expected_hpdma1,
             (unsigned long)expected_hpdma1,
             (unsigned long)secure_hpdma1,
             (unsigned long)privileged_hpdma1);
      ret = -EIO;
    }
  else
    {
      syslog(LOG_INFO, "DMA policy: HPDMA1 channel mask OK: %08lx\n",
             (unsigned long)expected_hpdma1);
    }

  for (channel = 0; channel < CONFIG_STM32_HPDMA1_NCHANNELS; channel++)
    {
      uint32_t cidcfg = getreg32(STM32_HPDMA1_CXCIDCFGR(channel));
      uint32_t expected_cid = STM32_HPDMA_CIDCFGR_CFEN |
                              STM32_HPDMA_CIDCFGR_SCID(1);

      if ((cidcfg & (STM32_HPDMA_CIDCFGR_CFEN |
                     STM32_HPDMA_CIDCFGR_SEM_EN |
                     STM32_HPDMA_CIDCFGR_SCID_MASK)) != expected_cid)
        {
          syslog(LOG_ERR,
                 "DMA policy: HPDMA1 ch%u CID config expected %08lx "
                 "got %08lx\n",
                 channel, (unsigned long)expected_cid,
                 (unsigned long)cidcfg);
          ret = -EIO;
        }
    }
#endif

#ifdef CONFIG_STM32_GPDMA1
#  if CONFIG_STM32_GPDMA1_NCHANNELS > CONFIG_STM32_GPDMA1_RESERVED_CHANNELS
  if (stm32_dma_test_copy(STM32_DMA_CONTROLLER_GPDMA1,
                          sizeof(uint32_t)) < 0)
    {
      ret = -EIO;
    }

  if (stm32_dma_test_linked_lists(STM32_DMA_CONTROLLER_GPDMA1,
                                  CONFIG_STM32_GPDMA1_NCHANNELS,
                                  CONFIG_STM32_GPDMA1_RESERVED_CHANNELS) < 0)
    {
      ret = -EIO;
    }
#  else
  syslog(LOG_WARNING, "DMA core: GPDMA1 test skipped; no channels available\n");
#  endif
#endif

#ifdef CONFIG_STM32_HPDMA1
#  if CONFIG_STM32_HPDMA1_NCHANNELS > CONFIG_STM32_HPDMA1_RESERVED_CHANNELS
  if (stm32_dma_test_copy(STM32_DMA_CONTROLLER_HPDMA1,
                          sizeof(uint32_t)) < 0)
    {
      ret = -EIO;
    }

  if (stm32_dma_test_copy(STM32_DMA_CONTROLLER_HPDMA1, 8) < 0)
    {
      ret = -EIO;
    }

  if (stm32_dma_test_linked_lists(STM32_DMA_CONTROLLER_HPDMA1,
                                  CONFIG_STM32_HPDMA1_NCHANNELS,
                                  CONFIG_STM32_HPDMA1_RESERVED_CHANNELS) < 0)
    {
      ret = -EIO;
    }
#  else
  syslog(LOG_WARNING, "DMA core: HPDMA1 test skipped; no channels available\n");
#  endif
#endif

#ifdef CONFIG_STM32_USART1
#  if defined(CONFIG_STM32_GPDMA1) && \
      CONFIG_STM32_GPDMA1_NCHANNELS > CONFIG_STM32_GPDMA1_RESERVED_CHANNELS
  if (stm32_dma_test_usart(STM32_DMA_CONTROLLER_GPDMA1) < 0)
    {
      ret = -EIO;
    }
#  endif
#  if defined(CONFIG_STM32_HPDMA1) && \
      CONFIG_STM32_HPDMA1_NCHANNELS > CONFIG_STM32_HPDMA1_RESERVED_CHANNELS
  if (stm32_dma_test_usart(STM32_DMA_CONTROLLER_HPDMA1) < 0)
    {
      ret = -EIO;
    }
#  endif
#endif

  return ret;
}
