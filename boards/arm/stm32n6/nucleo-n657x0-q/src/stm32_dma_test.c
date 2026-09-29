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
#include <syslog.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_gpdma.h"
#include "hardware/stm32n6xxx_hpdma.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma_policy_test
 *
 * Description:
 *   Check that architecture initialization applied the configured secure
 *   and privileged channel masks to the enabled DMA controllers.
 *
 ****************************************************************************/

int stm32_dma_policy_test(void)
{
  int ret = 0;

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
#endif

  return ret;
}
