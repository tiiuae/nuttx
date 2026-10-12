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

#include <errno.h>
#include <stddef.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>

#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The blue user button is active-high on PC13 (EXTI13). */

#define GPIO_EXTI_TEST_PINSET \
  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTC | GPIO_PIN13)

#define GPIO_EXTI_TEST_TIMEOUT SEC2TICK(3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_exti_int;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_gpio_exti_test_isr(int irq, void *context, void *arg)
{
  bool active;

  (void)irq;
  (void)context;
  (void)arg;

  active = stm32_gpioread(GPIO_EXTI_TEST_PINSET);
  if (active)
    {
      /* Latch the press so release cannot hide it from the polling task. */

      g_exti_int = true;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_gpio_exti_test_initialize(void)
{
  int ret;

  ret = stm32_gpiosetevent(GPIO_EXTI_TEST_PINSET, true, true, false,
                           stm32_gpio_exti_test_isr, NULL);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

int stm32_gpio_exti_test(void)
{
  irqstate_t flags;
  clock_t start;
  bool pressed;
  int ret;

  flags = enter_critical_section();
  g_exti_int = false;
  start = clock_systime_ticks();
  leave_critical_section(flags);

  syslog(LOG_INFO, "Press blue user button within 3 seconds\n");

  for (; ; )
    {
      flags = enter_critical_section();
      pressed = g_exti_int;
      leave_critical_section(flags);

      if (pressed)
        {
          syslog(LOG_INFO, "GPIO EXTI test: button press detected\n");
          return OK;
        }

      if (clock_systime_ticks() - start >= GPIO_EXTI_TEST_TIMEOUT)
        {
          return -ETIMEDOUT;
        }

      ret = nxsig_usleep(10000);
      if (ret < 0 && ret != -EINTR)
        {
          return ret;
        }
    }
}
