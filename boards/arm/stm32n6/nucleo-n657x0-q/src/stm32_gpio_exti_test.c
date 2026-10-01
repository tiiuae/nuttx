/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_gpio_exti_test.c
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

#include <stddef.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/board.h>

#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_EXTI_TEST_PINSET \
  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTE | GPIO_PIN12)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_gpio_exti_test_isr(int irq, void *context, void *arg)
{
  bool active;

  (void)irq;
  (void)context;
  (void)arg;

  active = !stm32_gpioread(GPIO_PORTE | GPIO_PIN12);
  board_userled(BOARD_LED_BLUE, active);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_gpio_exti_test_initialize(void)
{
  int ret;

  board_userled(BOARD_LED_BLUE, false);

  ret = stm32_gpiosetevent(GPIO_EXTI_TEST_PINSET, true, true, false,
                           stm32_gpio_exti_test_isr, NULL);
  if (ret < 0)
    {
      return ret;
    }

  board_userled(BOARD_LED_BLUE,
                !stm32_gpioread(GPIO_PORTE | GPIO_PIN12));

  syslog(LOG_INFO,
         "GPIO EXTI test active: ground PE12 for blue LED on\n");
  return OK;
}
