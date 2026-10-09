/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_pinmap.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc.  Drivers, however, will use the pin selection without the numeric
 * suffix.  Additional definitions are required in the board.h file.  For
 * example, if USART1_TX connects via PE5 on some board, then the following
 * definition should appear in the board.h header file for that board:
 *
 * #define GPIO_USART1_TX GPIO_USART1_TX_1
 *
 * The driver will then automatically configure PE5 as the USART1 TX pin.
 */

/* USART1: PE5=TX (AF7), PE6=RX (AF7) - ST-Link Virtual COM Port */

#define GPIO_USART1_TX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN5)
#define GPIO_USART1_RX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHz | GPIO_PORTE | GPIO_PIN6)

/* USART3: PD8=TX, PD9=RX (AF7), Arduino D1/D0 on MB1940-C02. */

#define GPIO_USART3_TX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN8)
#define GPIO_USART3_RX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHz | GPIO_PORTD | GPIO_PIN9)

/* I2C2: PB10=SCL and PB11=SDA (AF4, Nucleo-N657X0-Q BSP). */

#define GPIO_I2C2_SCL_1    (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA_1    (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN11)

/* SPI5: PE15=SCK, PG1=MISO, PG2=MOSI (AF5 per STM32N657x0 datasheet
 * DS14791 Rev 2 Table 17).  ST's STM32CubeN6 NUCLEO-N657X0-Q full-duplex
 * example uses these same pins.
 */

#define GPIO_SPI5_SCK_1    (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN15)
#define GPIO_SPI5_MISO_1   (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTG | GPIO_PIN1)
#define GPIO_SPI5_MOSI_1   (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN2)

/* NUCLEO-N657X0-Q OctoSPI flash on XSPI2 port 2 (MB1940 C02 schematic).
 * The pins share AF9 and use the 1.8 V VDDIO3 domain.
 */

#define GPIO_XSPI2_DQS_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PORTN | GPIO_PIN0)
#define GPIO_XSPI2_NCS_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN1)
#define GPIO_XSPI2_IO0_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN2)
#define GPIO_XSPI2_IO1_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN3)
#define GPIO_XSPI2_IO2_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN4)
#define GPIO_XSPI2_IO3_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN5)
#define GPIO_XSPI2_CLK_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN6)
#define GPIO_XSPI2_IO4_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN8)
#define GPIO_XSPI2_IO5_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN9)
#define GPIO_XSPI2_IO6_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN10)
#define GPIO_XSPI2_IO7_1   (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTN | GPIO_PIN11)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H */
