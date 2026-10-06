/****************************************************************************
 * arch/arm/src/stm32n6/stm32_uart.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_UART_H
#define __ARCH_ARM_SRC_STM32N6_STM32_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"

#include "hardware/stm32n6xxx_uart.h"
#include "stm32_serial_format.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity checks */

#if !defined(CONFIG_STM32_USART1)
#  undef CONFIG_STM32_USART1_SERIALDRIVER
#  undef CONFIG_STM32_USART1_1WIREDRIVER
#endif
#if !defined(CONFIG_STM32_USART2)
#  undef CONFIG_STM32_USART2_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_USART3)
#  undef CONFIG_STM32_USART3_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_UART4)
#  undef CONFIG_STM32_UART4_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_UART5)
#  undef CONFIG_STM32_UART5_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_USART6)
#  undef CONFIG_STM32_USART6_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_UART7)
#  undef CONFIG_STM32_UART7_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_UART8)
#  undef CONFIG_STM32_UART8_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_UART9)
#  undef CONFIG_STM32_UART9_SERIALDRIVER
#endif
#if !defined(CONFIG_STM32_USART10)
#  undef CONFIG_STM32_USART10_SERIALDRIVER
#endif

/* Is there a USART enabled? */

#if defined(CONFIG_STM32_USART1_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART2_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART3_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART4_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART5_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART6_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART7_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART8_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART9_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART10_SERIALDRIVER)
#  define HAVE_UART 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART1_SERIALDRIVER)
#  define CONSOLE_UART 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART2_SERIALDRIVER)
#  define CONSOLE_UART 2
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART3_SERIALDRIVER)
#  define CONSOLE_UART 3
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART4_SERIALDRIVER)
#  define CONSOLE_UART 4
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART5_SERIALDRIVER)
#  define CONSOLE_UART 5
#elif defined(CONFIG_USART6_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART6_SERIALDRIVER)
#  define CONSOLE_UART 6
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART7_SERIALDRIVER)
#  define CONSOLE_UART 7
#elif defined(CONFIG_UART8_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART8_SERIALDRIVER)
#  define CONSOLE_UART 8
#elif defined(CONFIG_UART9_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART9_SERIALDRIVER)
#  define CONSOLE_UART 9
#elif defined(CONFIG_USART10_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART10_SERIALDRIVER)
#  define CONSOLE_UART 10
#else
#  define CONSOLE_UART 0
#endif

#if CONSOLE_UART > 0
#  define HAVE_CONSOLE 1
#else
#  undef HAVE_CONSOLE
#endif

#define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct stm32_usart_s
{
  uint32_t base;
  uint32_t clock;             /* HSI frequency before inherited HSIDIV */
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t rts_gpio;
  uint32_t cts_gpio;
  uint32_t enable;
  uint32_t disable;
  uint32_t resetset;
  uint32_t resetclear;
  uint32_t lpen;
  uint32_t lpdisable;
  uint32_t rcc_bit;           /* Same bit position in EN/RST/LPEN aliases */
  uint32_t selector;
  uint32_t selmask;
  uint32_t selsource;
  uint16_t rxrequest;
  uint16_t txrequest;
  uint8_t irq;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct stm32_usart_s
  g_usart_config[STM32_NUSART + STM32_NUART];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* USART kernel clock before PRESC, with CCIPR13 selecting hsi_div_ck.
 * The global HSI divider must remain unchanged while serial is in use.
 */

uint32_t stm32_usart_clock(const struct stm32_usart_s *config);

#if defined(CONFIG_STM32_USART_INVERT) || \
    defined(CONFIG_STM32_USART_SINGLEWIRE)
int stm32_usart_setmode(uint32_t base, uint32_t cr2, uint32_t cr3,
                        uint32_t oldgpio, uint32_t newgpio);
#endif

void stm32_usart_setclock(const struct stm32_usart_s *config, bool on);
int stm32_usart_initialize(const struct stm32_usart_s *config, bool reset);
int stm32_usart_flowcontrol(const struct stm32_usart_s *config,
                            bool iflow, bool oflow, uint32_t *flow);

/* The caller must exclude IRQ/debug output and quiesce DMA and wire TX.
 * flow carries RTSE/CTSE and optional HDSEL; initial setup uses two pins.
 */

int stm32_usart_configure(uint32_t base,
                          const struct stm32_usart_format_s *format,
                          uint32_t flow);

int stm32_usart_disable(uint32_t base);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32N6_STM32_UART_H */
