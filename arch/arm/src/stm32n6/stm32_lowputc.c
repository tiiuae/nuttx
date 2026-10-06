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
#include "hardware/stm32n6xxx_dmasigmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select the configured console's initial line format. */

#ifdef HAVE_CONSOLE
#  if CONSOLE_UART == 1
#    define STM32N6_CONSOLE_BAUD CONFIG_USART1_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_USART1_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_USART1_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_USART1_2STOP
#  elif CONSOLE_UART == 2
#    define STM32N6_CONSOLE_BAUD CONFIG_USART2_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_USART2_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_USART2_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_USART2_2STOP
#  elif CONSOLE_UART == 3
#    define STM32N6_CONSOLE_BAUD CONFIG_USART3_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_USART3_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_USART3_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_USART3_2STOP
#  elif CONSOLE_UART == 4
#    define STM32N6_CONSOLE_BAUD CONFIG_UART4_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_UART4_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_UART4_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_UART4_2STOP
#  elif CONSOLE_UART == 5
#    define STM32N6_CONSOLE_BAUD CONFIG_UART5_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_UART5_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_UART5_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_UART5_2STOP
#  elif CONSOLE_UART == 6
#    define STM32N6_CONSOLE_BAUD CONFIG_USART6_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_USART6_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_USART6_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_USART6_2STOP
#  elif CONSOLE_UART == 7
#    define STM32N6_CONSOLE_BAUD CONFIG_UART7_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_UART7_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_UART7_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_UART7_2STOP
#  elif CONSOLE_UART == 8
#    define STM32N6_CONSOLE_BAUD CONFIG_UART8_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_UART8_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_UART8_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_UART8_2STOP
#  elif CONSOLE_UART == 9
#    define STM32N6_CONSOLE_BAUD CONFIG_UART9_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_UART9_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_UART9_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_UART9_2STOP
#  elif CONSOLE_UART == 10
#    define STM32N6_CONSOLE_BAUD CONFIG_USART10_BAUD
#    define STM32N6_CONSOLE_BITS CONFIG_USART10_BITS
#    define STM32N6_CONSOLE_PARITY CONFIG_USART10_PARITY
#    define STM32N6_CONSOLE_2STOP CONFIG_USART10_2STOP
#  endif

#  define STM32N6_CONSOLE (&g_usart_config[CONSOLE_UART - 1])
#  define STM32N6_CONSOLE_BASE (STM32N6_CONSOLE->base)

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

const struct stm32_usart_s
  g_usart_config[STM32_NUSART + STM32_NUART] =
{
#ifdef CONFIG_STM32_USART1_SERIALDRIVER
  [0] =
    {
      .base       = STM32_USART1_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_USART1,
      .tx_gpio    = GPIO_USART1_TX,
      .rx_gpio    = GPIO_USART1_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART1_IFLOWCONTROL)
      .rts_gpio   = GPIO_USART1_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART1_OFLOWCONTROL)
      .cts_gpio   = GPIO_USART1_CTS,
#  endif
      .enable     = STM32_RCC_APB2ENSR,
      .disable    = STM32_RCC_APB2ENCR,
      .resetset   = STM32_RCC_APB2RSTSR,
      .resetclear = STM32_RCC_APB2RSTCR,
      .lpen       = STM32_RCC_APB2LPENSR,
      .lpdisable  = STM32_RCC_APB2LPENCR,
      .rcc_bit    = RCC_APB2ENR_USART1EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_USART1SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_USART1SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_USART1_RX,
      .txrequest  = STM32_DMA_REQ_USART1_TX
    },
#endif
#ifdef CONFIG_STM32_USART2_SERIALDRIVER
  [1] =
    {
      .base       = STM32_USART2_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_USART2,
      .tx_gpio    = GPIO_USART2_TX,
      .rx_gpio    = GPIO_USART2_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART2_IFLOWCONTROL)
      .rts_gpio   = GPIO_USART2_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART2_OFLOWCONTROL)
      .cts_gpio   = GPIO_USART2_CTS,
#  endif
      .enable     = STM32_RCC_APB1LENSR,
      .disable    = STM32_RCC_APB1LENCR,
      .resetset   = STM32_RCC_APB1LRSTSR,
      .resetclear = STM32_RCC_APB1LRSTCR,
      .lpen       = STM32_RCC_APB1LLPENSR,
      .lpdisable  = STM32_RCC_APB1LLPENCR,
      .rcc_bit    = RCC_APB1LENR_USART2EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_USART2SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_USART2SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_USART2_RX,
      .txrequest  = STM32_DMA_REQ_USART2_TX
    },
#endif
#ifdef CONFIG_STM32_USART3_SERIALDRIVER
  [2] =
    {
      .base       = STM32_USART3_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_USART3,
      .tx_gpio    = GPIO_USART3_TX,
      .rx_gpio    = GPIO_USART3_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART3_IFLOWCONTROL)
      .rts_gpio   = GPIO_USART3_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART3_OFLOWCONTROL)
      .cts_gpio   = GPIO_USART3_CTS,
#  endif
      .enable     = STM32_RCC_APB1LENSR,
      .disable    = STM32_RCC_APB1LENCR,
      .resetset   = STM32_RCC_APB1LRSTSR,
      .resetclear = STM32_RCC_APB1LRSTCR,
      .lpen       = STM32_RCC_APB1LLPENSR,
      .lpdisable  = STM32_RCC_APB1LLPENCR,
      .rcc_bit    = RCC_APB1LENR_USART3EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_USART3SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_USART3SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_USART3_RX,
      .txrequest  = STM32_DMA_REQ_USART3_TX
    },
#endif
#ifdef CONFIG_STM32_UART4_SERIALDRIVER
  [3] =
    {
      .base       = STM32_UART4_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_UART4,
      .tx_gpio    = GPIO_UART4_TX,
      .rx_gpio    = GPIO_UART4_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART4_IFLOWCONTROL)
      .rts_gpio   = GPIO_UART4_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART4_OFLOWCONTROL)
      .cts_gpio   = GPIO_UART4_CTS,
#  endif
      .enable     = STM32_RCC_APB1LENSR,
      .disable    = STM32_RCC_APB1LENCR,
      .resetset   = STM32_RCC_APB1LRSTSR,
      .resetclear = STM32_RCC_APB1LRSTCR,
      .lpen       = STM32_RCC_APB1LLPENSR,
      .lpdisable  = STM32_RCC_APB1LLPENCR,
      .rcc_bit    = RCC_APB1LENR_UART4EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_UART4SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_UART4SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_UART4_RX,
      .txrequest  = STM32_DMA_REQ_UART4_TX
    },
#endif
#ifdef CONFIG_STM32_UART5_SERIALDRIVER
  [4] =
    {
      .base       = STM32_UART5_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_UART5,
      .tx_gpio    = GPIO_UART5_TX,
      .rx_gpio    = GPIO_UART5_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART5_IFLOWCONTROL)
      .rts_gpio   = GPIO_UART5_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART5_OFLOWCONTROL)
      .cts_gpio   = GPIO_UART5_CTS,
#  endif
      .enable     = STM32_RCC_APB1LENSR,
      .disable    = STM32_RCC_APB1LENCR,
      .resetset   = STM32_RCC_APB1LRSTSR,
      .resetclear = STM32_RCC_APB1LRSTCR,
      .lpen       = STM32_RCC_APB1LLPENSR,
      .lpdisable  = STM32_RCC_APB1LLPENCR,
      .rcc_bit    = RCC_APB1LENR_UART5EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_UART5SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_UART5SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_UART5_RX,
      .txrequest  = STM32_DMA_REQ_UART5_TX
    },
#endif
#ifdef CONFIG_STM32_USART6_SERIALDRIVER
  [5] =
    {
      .base       = STM32_USART6_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_USART6,
      .tx_gpio    = GPIO_USART6_TX,
      .rx_gpio    = GPIO_USART6_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART6_IFLOWCONTROL)
      .rts_gpio   = GPIO_USART6_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART6_OFLOWCONTROL)
      .cts_gpio   = GPIO_USART6_CTS,
#  endif
      .enable     = STM32_RCC_APB2ENSR,
      .disable    = STM32_RCC_APB2ENCR,
      .resetset   = STM32_RCC_APB2RSTSR,
      .resetclear = STM32_RCC_APB2RSTCR,
      .lpen       = STM32_RCC_APB2LPENSR,
      .lpdisable  = STM32_RCC_APB2LPENCR,
      .rcc_bit    = RCC_APB2ENR_USART6EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_USART6SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_USART6SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_USART6_RX,
      .txrequest  = STM32_DMA_REQ_USART6_TX
    },
#endif
#ifdef CONFIG_STM32_UART7_SERIALDRIVER
  [6] =
    {
      .base       = STM32_UART7_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_UART7,
      .tx_gpio    = GPIO_UART7_TX,
      .rx_gpio    = GPIO_UART7_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART7_IFLOWCONTROL)
      .rts_gpio   = GPIO_UART7_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART7_OFLOWCONTROL)
      .cts_gpio   = GPIO_UART7_CTS,
#  endif
      .enable     = STM32_RCC_APB1LENSR,
      .disable    = STM32_RCC_APB1LENCR,
      .resetset   = STM32_RCC_APB1LRSTSR,
      .resetclear = STM32_RCC_APB1LRSTCR,
      .lpen       = STM32_RCC_APB1LLPENSR,
      .lpdisable  = STM32_RCC_APB1LLPENCR,
      .rcc_bit    = RCC_APB1LENR_UART7EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_UART7SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_UART7SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_UART7_RX,
      .txrequest  = STM32_DMA_REQ_UART7_TX
    },
#endif
#ifdef CONFIG_STM32_UART8_SERIALDRIVER
  [7] =
    {
      .base       = STM32_UART8_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_UART8,
      .tx_gpio    = GPIO_UART8_TX,
      .rx_gpio    = GPIO_UART8_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART8_IFLOWCONTROL)
      .rts_gpio   = GPIO_UART8_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART8_OFLOWCONTROL)
      .cts_gpio   = GPIO_UART8_CTS,
#  endif
      .enable     = STM32_RCC_APB1LENSR,
      .disable    = STM32_RCC_APB1LENCR,
      .resetset   = STM32_RCC_APB1LRSTSR,
      .resetclear = STM32_RCC_APB1LRSTCR,
      .lpen       = STM32_RCC_APB1LLPENSR,
      .lpdisable  = STM32_RCC_APB1LLPENCR,
      .rcc_bit    = RCC_APB1LENR_UART8EN,
      .selector   = STM32_RCC_CCIPR13,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR13_UART8SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR13_UART8SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_UART8_RX,
      .txrequest  = STM32_DMA_REQ_UART8_TX
    },
#endif
#ifdef CONFIG_STM32_UART9_SERIALDRIVER
  [8] =
    {
      .base       = STM32_UART9_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_UART9,
      .tx_gpio    = GPIO_UART9_TX,
      .rx_gpio    = GPIO_UART9_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART9_IFLOWCONTROL)
      .rts_gpio   = GPIO_UART9_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART9_OFLOWCONTROL)
      .cts_gpio   = GPIO_UART9_CTS,
#  endif
      .enable     = STM32_RCC_APB2ENSR,
      .disable    = STM32_RCC_APB2ENCR,
      .resetset   = STM32_RCC_APB2RSTSR,
      .resetclear = STM32_RCC_APB2RSTCR,
      .lpen       = STM32_RCC_APB2LPENSR,
      .lpdisable  = STM32_RCC_APB2LPENCR,
      .rcc_bit    = RCC_APB2ENR_UART9EN,
      .selector   = STM32_RCC_CCIPR14,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR14_UART9SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR14_UART9SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_UART9_RX,
      .txrequest  = STM32_DMA_REQ_UART9_TX
    },
#endif
#ifdef CONFIG_STM32_USART10_SERIALDRIVER
  [9] =
    {
      .base       = STM32_USART10_BASE,
      .clock      = STM32_HSI_FREQUENCY,
      .irq        = STM32_IRQ_USART10,
      .tx_gpio    = GPIO_USART10_TX,
      .rx_gpio    = GPIO_USART10_RX,
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART10_IFLOWCONTROL)
      .rts_gpio   = GPIO_USART10_RTS,
#  endif
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART10_OFLOWCONTROL)
      .cts_gpio   = GPIO_USART10_CTS,
#  endif
      .enable     = STM32_RCC_APB2ENSR,
      .disable    = STM32_RCC_APB2ENCR,
      .resetset   = STM32_RCC_APB2RSTSR,
      .resetclear = STM32_RCC_APB2RSTCR,
      .lpen       = STM32_RCC_APB2LPENSR,
      .lpdisable  = STM32_RCC_APB2LPENCR,
      .rcc_bit    = RCC_APB2ENR_USART10EN,
      .selector   = STM32_RCC_CCIPR14,
      .selmask    = RCC_USARTSEL_MASK(RCC_CCIPR14_USART10SEL_SHIFT),
      .selsource  = RCC_USARTSEL_HSI(RCC_CCIPR14_USART10SEL_SHIFT),
      .rxrequest  = STM32_DMA_REQ_USART10_RX,
      .txrequest  = STM32_DMA_REQ_USART10_TX
    },
#endif
};

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

uint32_t stm32_usart_clock(const struct stm32_usart_s *config)
{
  uint32_t hsidiv = (getreg32(STM32_RCC_HSICFGR) &
                    RCC_HSICFGR_HSIDIV_MASK) >> RCC_HSICFGR_HSIDIV_SHIFT;

  return config->clock >> hsidiv;
}

void stm32_usart_setclock(const struct stm32_usart_s *config, bool on)
{
  if (on)
    {
      putreg32(config->rcc_bit, config->enable);
      putreg32(config->rcc_bit, config->lpen);
    }
  else
    {
      putreg32(config->rcc_bit, config->lpdisable);
      putreg32(config->rcc_bit, config->disable);
    }
}

int stm32_usart_initialize(const struct stm32_usart_s *config, bool reset)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  int ret;
#endif

  stm32_usart_setclock(config, true);

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  if (reset)
    {
      putreg32(config->rcc_bit, config->resetset);
      putreg32(config->rcc_bit, config->resetclear);
    }
  else if ((getreg32(config->selector) & config->selmask) !=
           config->selsource)
    {
      /* Preserve a running console; the previous stage must quiesce TX. */

      if ((getreg32(config->base + STM32_USART_CR1_OFFSET) &
           USART_CR1_UE) != 0 &&
          (getreg32(config->base + STM32_USART_ISR_OFFSET) &
           USART_ISR_TC) == 0)
        {
          return -EBUSY;
        }

      ret = stm32_usart_disable(config->base);
      if (ret < 0)
        {
          return ret;
        }
    }

  modifyreg32(config->selector, config->selmask, config->selsource);
#else
  UNUSED(reset);
#endif

  return OK;
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
#ifdef HAVE_CONSOLE
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct stm32_usart_format_s format;
#endif
  int ret;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  ret = stm32_usart_format(stm32_usart_clock(STM32N6_CONSOLE),
                           STM32N6_CONSOLE_BAUD,
                           STM32N6_CONSOLE_BITS, STM32N6_CONSOLE_PARITY,
                           STM32N6_CONSOLE_2STOP != 0, &format);
  if (ret < 0)
    {
      _err("ERROR: Invalid console format: %d\n", ret);
      PANIC();
    }
#endif

  ret = stm32_usart_initialize(STM32N6_CONSOLE, false);
  if (ret < 0)
    {
      _err("ERROR: Console clock setup failed: %d\n", ret);
      PANIC();
    }

  ret = stm32_configgpio(STM32N6_CONSOLE->tx_gpio);
  if (ret < 0)
    {
      _err("ERROR: Console TX GPIO setup failed: %d\n", ret);
      PANIC();
    }

  ret = stm32_configgpio(STM32N6_CONSOLE->rx_gpio);
  if (ret < 0)
    {
      _err("ERROR: Console RX GPIO setup failed: %d\n", ret);
      PANIC();
    }

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  ret = stm32_usart_configure(STM32N6_CONSOLE_BASE, &format, 0);
  if (ret < 0)
    {
      _err("ERROR: Console configuration failed: %d\n", ret);
      PANIC();
    }
#endif
#endif
}
