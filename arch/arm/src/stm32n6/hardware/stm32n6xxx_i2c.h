/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_i2c.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_I2C_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_I2C_H

#include <nuttx/config.h>

#include "hardware/stm32n6xxx_memorymap.h"

/* Register offsets (RM0486 section 63.9). */

#define STM32_I2C_CR1_OFFSET          0x0000
#define STM32_I2C_CR2_OFFSET          0x0004
#define STM32_I2C_OAR1_OFFSET         0x0008
#define STM32_I2C_OAR2_OFFSET         0x000c
#define STM32_I2C_TIMINGR_OFFSET      0x0010
#define STM32_I2C_TIMEOUTR_OFFSET     0x0014
#define STM32_I2C_ISR_OFFSET          0x0018
#define STM32_I2C_ICR_OFFSET          0x001c
#define STM32_I2C_PECR_OFFSET         0x0020
#define STM32_I2C_RXDR_OFFSET         0x0024
#define STM32_I2C_TXDR_OFFSET         0x0028

/* CR1 */

#define I2C_CR1_PE                   (1u << 0)
#define I2C_CR1_TXIE                 (1u << 1)
#define I2C_CR1_RXIE                 (1u << 2)
#define I2C_CR1_ADDRIE               (1u << 3)
#define I2C_CR1_NACKIE               (1u << 4)
#define I2C_CR1_STOPIE               (1u << 5)
#define I2C_CR1_TCIE                 (1u << 6)
#define I2C_CR1_ERRIE                (1u << 7)
#define I2C_CR1_DNF_SHIFT            8
#define I2C_CR1_DNF_MASK             (0x0fu << I2C_CR1_DNF_SHIFT)
#define I2C_CR1_ANFOFF               (1u << 12)
#define I2C_CR1_TXDMAEN              (1u << 14)
#define I2C_CR1_RXDMAEN              (1u << 15)
#define I2C_CR1_NOSTRETCH            (1u << 17)
#define I2C_CR1_GCEN                 (1u << 19)
#define I2C_CR1_SMBHEN               (1u << 20)
#define I2C_CR1_SMBDEN               (1u << 21)
#define I2C_CR1_ALERTEN              (1u << 22)
#define I2C_CR1_PECEN                (1u << 23)
#define I2C_CR1_FMP                  (1u << 24)

/* CR2 */

#define I2C_CR2_SADD7_SHIFT          1
#define I2C_CR2_SADD7_MASK           (0x7fu << I2C_CR2_SADD7_SHIFT)
#define I2C_CR2_RD_WRN               (1u << 10)
#define I2C_CR2_ADD10                (1u << 11)
#define I2C_CR2_HEAD10R              (1u << 12)
#define I2C_CR2_START                (1u << 13)
#define I2C_CR2_STOP                 (1u << 14)
#define I2C_CR2_NACK                 (1u << 15)
#define I2C_CR2_NBYTES_SHIFT         16
#define I2C_CR2_NBYTES_MASK          (0xffu << I2C_CR2_NBYTES_SHIFT)
#define I2C_CR2_RELOAD               (1u << 24)
#define I2C_CR2_AUTOEND              (1u << 25)

/* TIMINGR */

#define I2C_TIMINGR_SCLL_SHIFT       0
#define I2C_TIMINGR_SCLL_MASK        (0xffu << I2C_TIMINGR_SCLL_SHIFT)
#define I2C_TIMINGR_SCLH_SHIFT       8
#define I2C_TIMINGR_SCLH_MASK        (0xffu << I2C_TIMINGR_SCLH_SHIFT)
#define I2C_TIMINGR_SDADEL_SHIFT     16
#define I2C_TIMINGR_SDADEL_MASK      (0x0fu << I2C_TIMINGR_SDADEL_SHIFT)
#define I2C_TIMINGR_SCLDEL_SHIFT     20
#define I2C_TIMINGR_SCLDEL_MASK      (0x0fu << I2C_TIMINGR_SCLDEL_SHIFT)
#define I2C_TIMINGR_PRESC_SHIFT      28
#define I2C_TIMINGR_PRESC_MASK       (0x0fu << I2C_TIMINGR_PRESC_SHIFT)

/* ISR and ICR share the error/status bit positions documented in RM0486. */

#define I2C_ISR_TXE                  (1u << 0)
#define I2C_ISR_TXIS                 (1u << 1)
#define I2C_ISR_RXNE                 (1u << 2)
#define I2C_ISR_ADDR                 (1u << 3)
#define I2C_ISR_NACKF                (1u << 4)
#define I2C_ISR_STOPF                (1u << 5)
#define I2C_ISR_TC                   (1u << 6)
#define I2C_ISR_TCR                  (1u << 7)
#define I2C_ISR_BERR                 (1u << 8)
#define I2C_ISR_ARLO                 (1u << 9)
#define I2C_ISR_OVR                  (1u << 10)
#define I2C_ISR_PECERR               (1u << 11)
#define I2C_ISR_TIMEOUT              (1u << 12)
#define I2C_ISR_ALERT                (1u << 13)
#define I2C_ISR_BUSY                 (1u << 15)

#define I2C_ICR_ADDRCF               (1u << 3)
#define I2C_ICR_NACKCF               (1u << 4)
#define I2C_ICR_STOPCF               (1u << 5)
#define I2C_ICR_BERRCF               (1u << 8)
#define I2C_ICR_ARLOCF               (1u << 9)
#define I2C_ICR_OVRCF                (1u << 10)
#define I2C_ICR_PECCF                (1u << 11)
#define I2C_ICR_TIMOUTCF             (1u << 12)
#define I2C_ICR_ALERTCF              (1u << 13)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_I2C_H */
