/****************************************************************************
 * arch/arm/src/stm32n6/stm32_lowputc.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console.  Only USART1 is
 * supported in this initial port.
 */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define STM32N6_CONSOLE_BASE     STM32_USART1_BASE
#    define STM32N6_CONSOLE_APBREG   STM32_RCC_APB2ENSR
#    define STM32N6_CONSOLE_APBEN    RCC_APB2ENR_USART1EN
#    define STM32N6_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define STM32N6_CONSOLE_BITS     CONFIG_USART1_BITS
#    define STM32N6_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define STM32N6_CONSOLE_2STOP    CONFIG_USART1_2STOP
#    define STM32N6_CONSOLE_TX       GPIO_USART1_TX
#    define STM32N6_CONSOLE_RX       GPIO_USART1_RX
#  endif

#  if STM32N6_CONSOLE_BITS != 7 && STM32N6_CONSOLE_BITS != 8
#    error "STM32N6 serial supports only 7-bit and 8-bit payloads"
#  endif
#  if STM32N6_CONSOLE_BAUD <= 0 || STM32N6_CONSOLE_PARITY > 2
#    error "Invalid STM32N6 console line configuration"
#  endif
#endif /* HAVE_CONSOLE */

#define USART_ACK_TIMEOUT_US 100

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_usart_waitack(uint32_t base, uint32_t expected)
{
  unsigned int i;

  for (i = 0; i < USART_ACK_TIMEOUT_US; i++)
    {
      if ((getreg32(base + STM32_USART_ISR_OFFSET) &
           (USART_ISR_TEACK | USART_ISR_REACK)) == expected)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usart_clock
 *
 * Description:
 *   Return the nominal hsi_div_ck frequency inherited from ROM or FSBL.
 *
 ****************************************************************************/

uint32_t stm32_usart_clock(void)
{
  uint32_t hsidiv = (getreg32(STM32_RCC_HSICFGR) &
                    RCC_HSICFGR_HSIDIV_MASK) >> RCC_HSICFGR_HSIDIV_SHIFT;

  return STM32_HSI_FREQUENCY >> hsidiv;
}

int stm32_usart_disable(uint32_t base)
{
  uint32_t cr1 = getreg32(base + STM32_USART_CR1_OFFSET);
  uint32_t expected = 0;
  int ret;

  putreg32(cr1 & ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE),
           base + STM32_USART_CR1_OFFSET);
  ret = stm32_usart_waitack(base, 0);
  if (ret < 0)
    {
      putreg32(cr1, base + STM32_USART_CR1_OFFSET);
      if ((cr1 & USART_CR1_UE) != 0)
        {
          expected = (cr1 & USART_CR1_TE) != 0 ? USART_ISR_TEACK : 0;
          expected |= (cr1 & USART_CR1_RE) != 0 ? USART_ISR_REACK : 0;
        }

      if (stm32_usart_waitack(base, expected) < 0)
        {
          return -EIO;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_usart_configure
 *
 * Description:
 *   Apply a validated asynchronous format with DMA and wire TX quiescent.
 *   FIFOEN, format and PRESC are programmed with UE=0.  Restore the old
 *   registers if an acknowledgement times out.
 *
 ****************************************************************************/

int stm32_usart_configure(uint32_t base,
                          const struct stm32_usart_format_s *format,
                          uint32_t flow)
{
  uint32_t cr1 = getreg32(base + STM32_USART_CR1_OFFSET);
  uint32_t cr2 = getreg32(base + STM32_USART_CR2_OFFSET);
  uint32_t cr3 = getreg32(base + STM32_USART_CR3_OFFSET);
  uint32_t brr = getreg32(base + STM32_USART_BRR_OFFSET);
  uint32_t presc = getreg32(base + STM32_USART_PRESC_OFFSET);
  uint32_t enables = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  uint32_t newcr1;
  uint32_t newcr2;
  uint32_t newcr3;
  uint32_t expected;
  int ret;

  newcr1 = (cr1 & ~(enables | USART_CR1_FORMAT_MASK | USART_CR1_ALLINTS |
                   USART_CR1_UESM | USART_CR1_MME)) |
           format->cr1 | USART_CR1_FIFOEN;
  newcr2 = (cr2 & ~(USART_CR2_STOP_MASK | USART_CR2_CLKEN |
                   USART_CR2_CPOL | USART_CR2_CPHA | USART_CR2_LBCL |
                   USART_CR2_LINEN | USART_CR2_LBDIE | USART_CR2_ABREN |
                   USART_CR2_RTOEN)) | format->cr2;
  newcr3 = (cr3 & ~(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ALLINTS |
                   USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_SCEN |
                   USART_CR3_IREN | USART_CR3_IRLP | USART_CR3_OVRDIS |
                   USART_CR3_DDRE | USART_CR3_ONEBIT)) | flow;

  /* Early and full console setup may request the same format while the
   * initial idle frame is still being transmitted.  Do not restart it.
   */

  if ((cr1 & ~USART_CR1_ALLINTS) == (newcr1 | enables) &&
      cr2 == newcr2 && (cr3 & ~USART_CR3_ALLINTS) == newcr3 &&
      brr == format->brr && presc == format->presc)
    {
      putreg32(newcr1 | enables, base + STM32_USART_CR1_OFFSET);
      putreg32(newcr3, base + STM32_USART_CR3_OFFSET);
      return stm32_usart_waitack(base, USART_ISR_TEACK | USART_ISR_REACK);
    }

  if ((cr1 & USART_CR1_UE) != 0 &&
      (getreg32(base + STM32_USART_ISR_OFFSET) & USART_ISR_TC) == 0)
    {
      return -EBUSY;
    }

  ret = stm32_usart_disable(base);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(newcr1, base + STM32_USART_CR1_OFFSET);
  putreg32(newcr2, base + STM32_USART_CR2_OFFSET);
  putreg32(format->presc, base + STM32_USART_PRESC_OFFSET);
  putreg32(format->brr, base + STM32_USART_BRR_OFFSET);
  putreg32(newcr3, base + STM32_USART_CR3_OFFSET);
  putreg32(newcr1 | enables, base + STM32_USART_CR1_OFFSET);

  ret = stm32_usart_waitack(base, USART_ISR_TEACK | USART_ISR_REACK);
  if (ret < 0)
    {
      putreg32(newcr1, base + STM32_USART_CR1_OFFSET);
      putreg32(cr2, base + STM32_USART_CR2_OFFSET);
      putreg32(presc, base + STM32_USART_PRESC_OFFSET);
      putreg32(brr, base + STM32_USART_BRR_OFFSET);
      putreg32(cr3, base + STM32_USART_CR3_OFFSET);
      putreg32(cr1 & ~USART_CR1_UE, base + STM32_USART_CR1_OFFSET);
      putreg32(cr1, base + STM32_USART_CR1_OFFSET);

      expected = 0;
      if ((cr1 & USART_CR1_UE) != 0)
        {
          expected = (cr1 & USART_CR1_TE) != 0 ? USART_ISR_TEACK : 0;
          expected |= (cr1 & USART_CR1_RE) != 0 ? USART_ISR_REACK : 0;
        }

      if (stm32_usart_waitack(base, expected) < 0)
        {
          return -EIO;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  while ((getreg32(STM32N6_CONSOLE_BASE + STM32_USART_ISR_OFFSET) &
         USART_ISR_TXE) == 0);

  putreg32((uint32_t)ch, STM32N6_CONSOLE_BASE + STM32_USART_TDR_OFFSET);
#endif
}

/****************************************************************************
 * Name: stm32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void stm32_lowsetup(void)
{
#if defined(HAVE_UART)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  struct stm32_usart_format_s format;
  int ret;

  ret = stm32_usart_format(stm32_usart_clock(), STM32N6_CONSOLE_BAUD,
                           STM32N6_CONSOLE_BITS, STM32N6_CONSOLE_PARITY,
                           STM32N6_CONSOLE_2STOP != 0, &format);
  if (ret < 0)
    {
      _err("ERROR: Invalid console format: %d\n", ret);
      PANIC();
    }
#endif

#if defined(HAVE_CONSOLE)
  /* Use the write-1-to-set ENSR alias rather than RMW on ENR so we do
   * not race other producers of the clock-enable bitmap.
   */

  putreg32(STM32N6_CONSOLE_APBEN, STM32N6_CONSOLE_APBREG);
#endif

#ifdef STM32N6_CONSOLE_TX
  stm32_configgpio(STM32N6_CONSOLE_TX);
#endif
#ifdef STM32N6_CONSOLE_RX
  stm32_configgpio(STM32N6_CONSOLE_RX);
#endif

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  ret = stm32_usart_configure(STM32N6_CONSOLE_BASE, &format, 0);
  if (ret < 0)
    {
      _err("ERROR: Console configuration failed: %d\n", ret);
      PANIC();
    }
#endif
#endif
}
