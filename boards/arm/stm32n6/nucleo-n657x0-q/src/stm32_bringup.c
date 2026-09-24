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
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#include "nucleo-n657x0-q.h"

#include <arch/board/board.h>

#if defined(CONFIG_NUCLEO_N657X0_Q_TIMER_CLOCKTEST)
#include "stm32_rcc.h"
#include "stm32_tim.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 ****************************************************************************/

#if defined(CONFIG_NUCLEO_N657X0_Q_TIMER_CLOCKTEST)
void stm32_timer_clocktest(void)
{
  uint32_t reg, reg2;

  /* 1. Read and log RCC CFGR2 register value */
  reg = getreg32(STM32_RCC_CFGR2);
  syslog(LOG_INFO, "TIMER TEST START");
  syslog(LOG_INFO, "RCC CFGR2: 0x%lx", reg);

  /* TODO: 2. Configure RIFSC access for TIM1 and TIM5 */

  /* 3. Enable APB clocks for TIM1 and TIM5 */
  putreg32(RCC_APB2ENR_TIM1EN, STM32_RCC_APB2ENSR);
  putreg32(RCC_APB1LENR_TIM5EN, STM32_RCC_APB1LENSR);

  /* TODO: 4. Set PSC=99, a maximal ARR, issue EGR.UG,
   * and set CR1.CEN
   */

  /* TIM1 is 16-bit AC timer */
  putreg16(99, STM32_TIM1_PSC);
  putreg16(0xffff, STM32_TIM1_ARR);
  putreg16(ATIM_EGR_UG, STM32_TIM1_EGR);

  /* TIM5 is 32-bit GP timer */
  putreg16(99, STM32_TIM5_PSC);
  putreg32(0xffffffff, STM32_TIM5_ARR);
  putreg16(GTIM_EGR_UG, STM32_TIM5_EGR);

  reg = getreg32(STM32_TIM1_CNT);
  reg2 = getreg32(STM32_TIM5_CNT);
  syslog(LOG_INFO, "TIM1 CNT: %lu", reg);
  syslog(LOG_INFO, "TIM5 CNT: %lu", reg2);

  /* Start both timers */
  syslog(LOG_INFO, "Start timers");
  reg = getreg16(STM32_TIM1_CR1);
  reg2 = getreg16(STM32_TIM5_CR1);
  putreg16(reg | ATIM_CR1_CEN, STM32_TIM1_CR1);
  putreg16(reg2 | GTIM_CR1_CEN, STM32_TIM5_CR1);

  /* TODO: 5. Read and log CNT, delay with usleep and log CNT again */
  reg = getreg32(STM32_TIM1_CNT);
  reg2 = getreg32(STM32_TIM5_CNT);

  // 10ms delay using SYSTICK cnt
  up_udelay(10000);

  reg = getreg32(STM32_TIM1_CNT);
  reg2 = getreg32(STM32_TIM5_CNT);
  syslog(LOG_INFO, "After 10ms:");
  syslog(LOG_INFO, "TIM1 CNT: %lu", reg);
  syslog(LOG_INFO, "TIM5 CNT: %lu", reg2);

  /* TODO: 6. Stop both timers */
  reg = getreg16(STM32_TIM1_CR1);
  reg2 = getreg16(STM32_TIM5_CR1);
  putreg16(reg & ~ATIM_CR1_CEN, STM32_TIM1_CR1);
  putreg16(reg2 & ~GTIM_CR1_CEN, STM32_TIM5_CR1);
  syslog(LOG_INFO, "TIMER TEST END");
}

#endif

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
#endif

#if defined(CONFIG_NUCLEO_N657X0_Q_TIMER_CLOCKTEST)
  stm32_timer_clocktest();
#endif
  return OK;
}
