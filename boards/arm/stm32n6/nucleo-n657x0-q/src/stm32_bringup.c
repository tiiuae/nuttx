/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_bringup.c
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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#include "nucleo-n657x0-q.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  int ret;

  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#ifdef CONFIG_NUCLEO_N657X0_Q_GPIO_EXTI_TEST
  else
    {
      ret = stm32_gpio_exti_test_initialize();
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: GPIO EXTI test setup failed: %d\n", ret);
        }
    }
#endif
#endif

#if defined(CONFIG_NUCLEO_N657X0_Q_TIMER_CLOCKTEST)
  syslog(LOG_INFO, "=== TIMER TEST BEGIN ===\n");
  stm32_timer_clocktest();
  syslog(LOG_INFO, "=== TIMER TEST END ===\n");
#endif

#if defined(CONFIG_NUCLEO_N657X0_Q_DMA_POLICYTEST)
  syslog(LOG_INFO, "=== DMA POLICY TEST BEGIN ===\n");
  if (stm32_dma_policy_test() < 0)
    {
      syslog(LOG_ERR, "ERROR: DMA access policy test failed\n");
    }

  syslog(LOG_INFO, "=== DMA POLICY TEST END ===\n");
#endif

#if defined(CONFIG_NUCLEO_N657X0_Q_SPI5_LOOPBACK_TEST)
  syslog(LOG_INFO, "=== SPI5 LOOPBACK TEST BEGIN ===\n");
  if (stm32_spi5_loopback_test() < 0)
    {
      syslog(LOG_ERR, "ERROR: SPI5 loopback test failed\n");
    }

  syslog(LOG_INFO, "=== SPI5 LOOPBACK TEST END ===\n");
#endif

#if defined(CONFIG_NUCLEO_N657X0_Q_SPI5_BMP280_TEST)
  syslog(LOG_INFO, "=== SPI5 BMP280 TEST BEGIN ===\n");
  if (stm32_spi5_bmp280_test() < 0)
    {
      syslog(LOG_ERR, "ERROR: SPI5 BMP280 test failed\n");
    }

  syslog(LOG_INFO, "=== SPI5 BMP280 TEST END ===\n");
#endif

  return OK;
}
