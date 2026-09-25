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

#include <errno.h>
#include <stdint.h>

#include <arch/barriers.h>
#include <arch/irq.h>
#include <debug.h>
#include <nuttx/cache.h>

#include "arm_internal.h"
#include "nvic.h"
#include "hardware/stm32n6xxx_memorymap.h"
#include "hardware/stm32n6xxx_uart.h"
#include "nucleo-n657x0-q.h"

// For userleds
#include <stdbool.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSH_XSPI_VECTOR_BASE  0x70100400ul
#define XSPI2_IMAGE_BASE      STM32_XSPI2_BANK
#define XSPI2_IMAGE_SIZE      0x04000000ul

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xip_vector_s
{
  uint32_t stack;
  uint32_t reset;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
static void stm32_uart_flush_tx(void)
{
#ifdef CONFIG_USART1_SERIAL_CONSOLE
  while ((getreg32(STM32_USART1_ISR) & USART_ISR_TC) == 0)
    {
    }
#endif
}
*/

static int xip_vector_valid(const struct xip_vector_s *vector)
{
  uint32_t reset = vector->reset & ~1ul;

  if (vector->stack < 0x20000000ul || vector->stack >= 0x40000000ul ||
      reset < XSPI2_IMAGE_BASE ||
      reset >= XSPI2_IMAGE_BASE + XSPI2_IMAGE_SIZE ||
      (vector->reset & 1) == 0)
    {
      return -EINVAL;
    }

  return OK;
}

static void __attribute__((noreturn))
xip_jump(const struct xip_vector_s *vector)
{
  int i;

  //stm32_uart_flush_tx();
  up_irq_disable();
  putreg32(0, NVIC_SYSTICK_CTRL);
  putreg32(NVIC_INTCTRL_PENDSTCLR, NVIC_INTCTRL);

  for (i = 0; i < NR_IRQS; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
      putreg32(0xffffffff, NVIC_IRQ_CLRPEND(i));
    }

#ifdef CONFIG_ARMV8M_DCACHE
  up_disable_dcache();
#endif
#ifdef CONFIG_ARMV8M_ICACHE
  up_disable_icache();
#endif

  putreg32(NSH_XSPI_VECTOR_BASE, NVIC_VECTAB);
  UP_DSB();
  UP_ISB();

  __asm__ volatile
    (
      "movs r2, #0\n\t"
      "msr control, r2\n\t"
      "isb\n\t"
      "msr msp, %0\n\t"
      "msr psp, r2\n\t"
      "bx %1\n\t"
      :
      : "r" (vector->stack), "r" (vector->reset)
      : "r2", "memory"
    );

  __builtin_unreachable();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  const struct xip_vector_s *vector;
  int ret;

  UNUSED(argc);
  UNUSED(argv);

  /*
  board_userled_initialize();
  board_userled(BOARD_LED_GREEN, true);
  board_userled(BOARD_LED_BLUE, true);
  */

  _alert("initialize MX25UM51245G at 100 MHz DTR");

  vector = (const struct xip_vector_s *)NSH_XSPI_VECTOR_BASE;

  ret = stm32_xspi_flash_initialize();
  if (ret == OK)
    {
      ret = xip_vector_valid(vector);
    }

  if (ret < 0)
    {
      _alert("ERROR: XIP boot failed: %d", ret);
      return ret;
    }

  xip_jump(vector);
}
