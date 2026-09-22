/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_bootloader.c
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

#include <stdint.h>

#include <arch/barriers.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"
#include "hardware/stm32n6xxx_memorymap.h"
#include "hardware/stm32n6xxx_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The application is signed with a 0x400-byte STM32 v2.3 header at
 * 0x70100000.  Its vector table is the first item in the payload.
 */

#define NSH_XSPI_VECTOR_BASE           0x70100400ul

/* XSPI device configuration register 2.  A value of zero selects a divide
 * ratio of one, so the XSPI2 serial clock equals its IC3 kernel clock.
 */

#define STM32_XSPI_DCR2                (STM32_XSPI2_BASE + 0x000c)
#define XSPI_DCR2_PRESCALER_MASK       0xffu

/* PLL1 runs at 800 MHz on this board.  IC3 /4 supplies the 200 MHz XSPI2
 * kernel clock configured below.
 */

#define STM32_XSPI2_IC3_DIV            4u

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xspi2_set_200mhz
 *
 * Description:
 *   The ROM has already put XSPI2 into memory-mapped mode.  Preserve that
 *   transaction configuration and only remove the controller prescaler and
 *   set IC3 to 800 MHz / 4.  This function is called while executing from
 *   SRAM2, so changing the flash clock cannot interrupt instruction fetches.
 *
 ****************************************************************************/

static void stm32_xspi2_set_200mhz(void)
{
  modifyreg32(STM32_XSPI_DCR2, XSPI_DCR2_PRESCALER_MASK, 0);

  modifyreg32(STM32_RCC_IC3CFGR, RCC_ICCFGR_INT_MASK,
              (STM32_XSPI2_IC3_DIV - 1) << RCC_ICCFGR_INT_SHIFT);
  putreg32(RCC_DIVENR_IC3EN, STM32_RCC_DIVENSR);

  UP_DSB();
  UP_ISB();
}

/****************************************************************************
 * Name: stm32_boot_nsh_xspi
 *
 * Description:
 *   Install the XIP application's vector table and branch to its reset
 *   handler.  This has the same architectural effect as a reset into the
 *   application, without reconfiguring XSPI2 after it has reached 200 MHz.
 *
 ****************************************************************************/

static void __attribute__((noreturn)) stm32_boot_nsh_xspi(void)
{
  const uint32_t *vectors = (const uint32_t *)NSH_XSPI_VECTOR_BASE;
  uint32_t msp = vectors[0];
  uint32_t reset = vectors[1];

  up_irq_disable();
  putreg32(0, NVIC_SYSTICK_CTRL);

  stm32_xspi2_set_200mhz();

  putreg32(NSH_XSPI_VECTOR_BASE, NVIC_VECTAB);
  UP_DSB();
  UP_ISB();

  __asm__ volatile
    (
      "msr msp, %0\n\t"
      "bx %1\n\t"
      :
      : "r" (msp), "r" (reset)
      : "memory"
    );

  __builtin_unreachable();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bootloader_main
 *
 * Description:
 *   Initial NuttX application for the SRAM2-resident bootloader image.
 *
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  stm32_boot_nsh_xspi();
}
