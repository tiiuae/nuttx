/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_spi.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_SPI_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_SPI_H

/****************************************************************************
 * Register Offsets
 ****************************************************************************/

/* RM0486 Rev 4, Table 667. */

#define STM32_SPI_CR1_OFFSET       0x0000
#define STM32_SPI_CR2_OFFSET       0x0004
#define STM32_SPI_CFG1_OFFSET      0x0008
#define STM32_SPI_CFG2_OFFSET      0x000c
#define STM32_SPI_IER_OFFSET       0x0010
#define STM32_SPI_SR_OFFSET        0x0014
#define STM32_SPI_IFCR_OFFSET      0x0018
#define STM32_SPI_TXDR_OFFSET      0x0020
#define STM32_SPI_RXDR_OFFSET      0x0030

/****************************************************************************
 * Register Bitfield Definitions
 ****************************************************************************/

/* SPI_CR1, RM0486 Rev 4 section 67.11.1. */

#define SPI_CR1_SSI                (1u << 12)
#define SPI_CR1_CSUSP              (1u << 10)
#define SPI_CR1_CSTART             (1u << 9)
#define SPI_CR1_SPE                (1u << 0)

/* SPI_CR2, RM0486 Rev 4 section 67.11.2. */

#define SPI_CR2_TSIZE_SHIFT        0
#define SPI_CR2_TSIZE_MASK         (0xffffu << SPI_CR2_TSIZE_SHIFT)

/* SPI_CFG1, RM0486 Rev 4 section 67.11.3. */

#define SPI_CFG1_BPASS             (1u << 31)
#define SPI_CFG1_MBR_SHIFT         28
#define SPI_CFG1_MBR_MASK          (7u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV2          (0u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV4          (1u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV8          (2u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV16         (3u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV32         (4u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV64         (5u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV128        (6u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_MBR_DIV256        (7u << SPI_CFG1_MBR_SHIFT)
#define SPI_CFG1_TXDMAEN           (1u << 15)
#define SPI_CFG1_RXDMAEN           (1u << 14)
#define SPI_CFG1_FTHLV_SHIFT       5
#define SPI_CFG1_FTHLV_MASK        (0xfu << SPI_CFG1_FTHLV_SHIFT)
#define SPI_CFG1_FTHLV_1DATA       (0u << SPI_CFG1_FTHLV_SHIFT)
#define SPI_CFG1_DSIZE_SHIFT       0
#define SPI_CFG1_DSIZE_MASK        (0x1fu << SPI_CFG1_DSIZE_SHIFT)
#define SPI_CFG1_DSIZE_8BIT        (7u << SPI_CFG1_DSIZE_SHIFT)
#define SPI_CFG1_DSIZE_16BIT       (15u << SPI_CFG1_DSIZE_SHIFT)

/* SPI_CFG2, RM0486 Rev 4 section 67.11.4. */

#define SPI_CFG2_AFCNTR             (1u << 31)
#define SPI_CFG2_SSM               (1u << 26)
#define SPI_CFG2_CPOL              (1u << 25)
#define SPI_CFG2_CPHA              (1u << 24)
#define SPI_CFG2_LSBFRST           (1u << 23)
#define SPI_CFG2_MASTER            (1u << 22)
#define SPI_CFG2_COMM_SHIFT        17
#define SPI_CFG2_COMM_MASK         (3u << SPI_CFG2_COMM_SHIFT)
#define SPI_CFG2_COMM_FULLDUPLEX   (0u << SPI_CFG2_COMM_SHIFT)

/* SPI_IER, RM0486 Rev 4 section 67.11.5. */

#define SPI_IER_MODFIE             (1u << 9)
#define SPI_IER_TIFREIE            (1u << 8)
#define SPI_IER_CRCEIE             (1u << 7)
#define SPI_IER_OVRIE              (1u << 6)
#define SPI_IER_UDRIE              (1u << 5)
#define SPI_IER_TXTFIE             (1u << 4)
#define SPI_IER_EOTIE              (1u << 3)
#define SPI_IER_DXPIE              (1u << 2)
#define SPI_IER_TXPIE              (1u << 1)
#define SPI_IER_RXPIE              (1u << 0)

/* SPI_SR, RM0486 Rev 4 section 67.11.6. */

#define SPI_SR_CTSIZE_SHIFT        16
#define SPI_SR_CTSIZE_MASK         (0xffffu << SPI_SR_CTSIZE_SHIFT)
#define SPI_SR_RXWNE               (1u << 15)
#define SPI_SR_RXPLVL_SHIFT        13
#define SPI_SR_RXPLVL_MASK         (3u << SPI_SR_RXPLVL_SHIFT)
#define SPI_SR_TXC                 (1u << 12)
#define SPI_SR_SUSP                (1u << 11)
#define SPI_SR_MODF                (1u << 9)
#define SPI_SR_TIFRE               (1u << 8)
#define SPI_SR_CRCE                (1u << 7)
#define SPI_SR_OVR                 (1u << 6)
#define SPI_SR_UDR                 (1u << 5)
#define SPI_SR_TXTF                (1u << 4)
#define SPI_SR_EOT                 (1u << 3)
#define SPI_SR_DXP                 (1u << 2)
#define SPI_SR_TXP                 (1u << 1)
#define SPI_SR_RXP                 (1u << 0)

/* SPI_IFCR, RM0486 Rev 4 section 67.11.7. */

#define SPI_IFCR_SUSPC             (1u << 11)
#define SPI_IFCR_MODFC             (1u << 9)
#define SPI_IFCR_TIFREC            (1u << 8)
#define SPI_IFCR_CRCEC             (1u << 7)
#define SPI_IFCR_OVRC              (1u << 6)
#define SPI_IFCR_UDRC              (1u << 5)
#define SPI_IFCR_TXTFC             (1u << 4)
#define SPI_IFCR_EOTC              (1u << 3)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_SPI_H */
