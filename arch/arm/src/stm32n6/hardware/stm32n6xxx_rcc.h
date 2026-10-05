/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32n6xxx_memorymap.h"

#if defined(CONFIG_STM32_STM32N6XXXX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RCC_CR_OFFSET           0x0000  /* Clock control register */
#define STM32_RCC_SR_OFFSET           0x0004  /* Clock status register */
#define STM32_RCC_CFGR1_OFFSET        0x0020  /* Clock configuration register 1 */
#define STM32_RCC_CFGR2_OFFSET        0x0024  /* Clock configuration register 2 */
#define STM32_RCC_HSICFGR_OFFSET      0x0048  /* HSI configuration register (RM0486 14.10.11) */
#define STM32_RCC_PLL1CFGR1_OFFSET    0x0080  /* PLL1 configuration register 1 */
#define STM32_RCC_PLL1CFGR3_OFFSET    0x0088  /* PLL1 configuration register 3 */
#define STM32_RCC_IC1CFGR_OFFSET      0x00c4  /* IC1 configuration register */
#define STM32_RCC_IC2CFGR_OFFSET      0x00c8  /* IC2 configuration register */
#define STM32_RCC_IC3CFGR_OFFSET      0x00cc  /* IC3 configuration register */
#define STM32_RCC_IC6CFGR_OFFSET      0x00d8  /* IC6 configuration register */
#define STM32_RCC_IC11CFGR_OFFSET     0x00ec  /* IC11 configuration register */
#define STM32_RCC_CCIPR4_OFFSET       0x0150  /* Kernel clock select register 4 */
#define STM32_RCC_CCIPR9_OFFSET       0x0164  /* Peripheral kernel clock select register 9 */
#define STM32_RCC_CCIPR13_OFFSET      0x0174  /* Peripheral kernel clock select register 13 */
#define STM32_RCC_AHB1RSTR_OFFSET     0x0210  /* AHB1 peripheral reset register */
#define STM32_RCC_AHB5RSTR_OFFSET     0x0220  /* AHB5 peripheral reset register */
#define STM32_RCC_APB1LRSTR_OFFSET    0x0224  /* APB1L peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET     0x022c  /* APB2 peripheral reset register */
#define STM32_RCC_APB4LRSTR_OFFSET    0x0234  /* APB4L peripheral reset register */
#define STM32_RCC_BUSENR_OFFSET       0x0244  /* Embedded bus clock enable register */
#define STM32_RCC_AHB1ENSR_OFFSET     0x0a50  /* AHB1 peripheral clock enable set register */
#define STM32_RCC_AHB5ENSR_OFFSET     0x0a60  /* AHB5 peripheral clock enable set register */
#define STM32_RCC_AHB1RSTSR_OFFSET    0x0a10  /* AHB1 peripheral reset set register */
#define STM32_RCC_AHB5RSTSR_OFFSET    0x0a20  /* AHB5 peripheral reset set register */
#define STM32_RCC_APB1LRSTSR_OFFSET   0x0a24  /* APB1L peripheral reset set register */
#define STM32_RCC_APB2RSTSR_OFFSET    0x0a2c  /* APB2 peripheral reset set register */
#define STM32_RCC_APB4LRSTSR_OFFSET   0x0a34  /* APB4L peripheral reset set register */
#define STM32_RCC_BUSENSR_OFFSET      0x0a44  /* Embedded bus clock enable set register */

/* Peripheral clock enable / set / clear register offsets.  Each enable
 * register (xxxENR) has a paired set register (xxxENSR) that performs an
 * atomic OR and a clear register (xxxENCR) that performs an atomic AND
 * (Reference: RM0486 14.5).  The same triplet pattern applies to the LPENR
 * (sleep mode) registers.
 */

#define STM32_RCC_MEMENR_OFFSET       0x024c  /* AXI/AHB SRAM clock enable register */
#define STM32_RCC_AHB4ENR_OFFSET      0x025c  /* AHB4 peripheral clock enable register */
#define STM32_RCC_APB1LENR_OFFSET     0x0264  /* APB1 peripheral clock enable register 1 */
#define STM32_RCC_APB2ENR_OFFSET      0x026c  /* APB2 peripheral clock enable register */
#define STM32_RCC_APB4LENR_OFFSET     0x0274  /* APB4 peripheral clock enable register 1 */
#define STM32_RCC_APB4HENR_OFFSET     0x0278  /* APB4 peripheral clock enable register 2 */
#define STM32_RCC_BUSLPENR_OFFSET     0x0284  /* Bus clocks enable in Sleep mode */
#define STM32_RCC_MEMLPENR_OFFSET     0x028c  /* SRAM clocks enable in Sleep mode */
#define STM32_RCC_APB1LLPENR_OFFSET   0x02a4  /* APB1 LP clock enable register 1 */
#define STM32_RCC_APB2LPENR_OFFSET    0x02ac  /* APB2 LP clock enable register */

#define STM32_RCC_DIVENR_OFFSET       0x0240  /* IC divider enable register */
#define STM32_RCC_DIVENSR_OFFSET      0x0a40  /* IC divider enable set register */

#define STM32_RCC_MEMENSR_OFFSET      0x0a4c  /* SRAM clock enable set register */
#define STM32_RCC_AHB4ENSR_OFFSET     0x0a5c  /* AHB4 clock enable set register */
#define STM32_RCC_APB1LENSR_OFFSET    0x0a64  /* APB1 clock enable set register 1 */
#define STM32_RCC_APB2ENSR_OFFSET     0x0a6c  /* APB2 clock enable set register */
#define STM32_RCC_APB4LENSR_OFFSET    0x0a74  /* APB4 clock enable set register 1 */
#define STM32_RCC_APB4HENSR_OFFSET    0x0a78  /* APB4 clock enable set register 2 */
#define STM32_RCC_BUSLPENSR_OFFSET    0x0a84  /* Bus LP clock enable set register */
#define STM32_RCC_MEMLPENSR_OFFSET    0x0a8c  /* SRAM LP clock enable set register */
#define STM32_RCC_APB1LLPENSR_OFFSET  0x0aa4  /* APB1 LP clock enable set register 1 */
#define STM32_RCC_APB2LPENSR_OFFSET   0x0aac  /* APB2 LP clock enable set register */

#define STM32_RCC_CCR_OFFSET          0x1000  /* Clock control clear register */
#define STM32_RCC_AHB1RSTCR_OFFSET    0x1210  /* AHB1 peripheral reset clear register */
#define STM32_RCC_AHB5RSTCR_OFFSET    0x1220  /* AHB5 peripheral reset clear register */
#define STM32_RCC_APB1LRSTCR_OFFSET   0x1224  /* APB1L peripheral reset clear register */
#define STM32_RCC_APB2RSTCR_OFFSET    0x122c  /* APB2 peripheral reset clear register */
#define STM32_RCC_APB4LRSTCR_OFFSET   0x1234  /* APB4L peripheral reset clear register */
#define STM32_RCC_APB1LENCR_OFFSET    0x1264  /* APB1L clock enable clear register */
#define STM32_RCC_APB2ENCR_OFFSET     0x126c  /* APB2 clock enable clear register */
#define STM32_RCC_APB4LENCR_OFFSET   0x1274  /* APB4L clock enable clear register */

#define STM32_RCC_CSR_OFFSET          0x0800  /* Clock status (set) register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                  (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_SR                  (STM32_RCC_BASE + STM32_RCC_SR_OFFSET)
#define STM32_RCC_CFGR1               (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2               (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_HSICFGR             (STM32_RCC_BASE + STM32_RCC_HSICFGR_OFFSET)
#define STM32_RCC_PLL1CFGR1           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR1_OFFSET)
#define STM32_RCC_PLL1CFGR3           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR3_OFFSET)
#define STM32_RCC_IC1CFGR             (STM32_RCC_BASE + STM32_RCC_IC1CFGR_OFFSET)
#define STM32_RCC_IC2CFGR             (STM32_RCC_BASE + STM32_RCC_IC2CFGR_OFFSET)
#define STM32_RCC_IC3CFGR             (STM32_RCC_BASE + STM32_RCC_IC3CFGR_OFFSET)
#define STM32_RCC_IC6CFGR             (STM32_RCC_BASE + STM32_RCC_IC6CFGR_OFFSET)
#define STM32_RCC_IC11CFGR            (STM32_RCC_BASE + STM32_RCC_IC11CFGR_OFFSET)
#define STM32_RCC_CCIPR4              (STM32_RCC_BASE + STM32_RCC_CCIPR4_OFFSET)
#define STM32_RCC_CCIPR9              (STM32_RCC_BASE + STM32_RCC_CCIPR9_OFFSET)
#define STM32_RCC_CCIPR13             (STM32_RCC_BASE + STM32_RCC_CCIPR13_OFFSET)
#define STM32_RCC_AHB1RSTR            (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB5RSTR            (STM32_RCC_BASE + STM32_RCC_AHB5RSTR_OFFSET)
#define STM32_RCC_APB1LRSTR           (STM32_RCC_BASE + STM32_RCC_APB1LRSTR_OFFSET)
#define STM32_RCC_APB2RSTR            (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB4LRSTR           (STM32_RCC_BASE + STM32_RCC_APB4LRSTR_OFFSET)
#define STM32_RCC_BUSENR               (STM32_RCC_BASE + STM32_RCC_BUSENR_OFFSET)
#define STM32_RCC_AHB1ENSR            (STM32_RCC_BASE + STM32_RCC_AHB1ENSR_OFFSET)
#define STM32_RCC_AHB5ENSR            (STM32_RCC_BASE + STM32_RCC_AHB5ENSR_OFFSET)
#define STM32_RCC_AHB1RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB1RSTSR_OFFSET)
#define STM32_RCC_AHB5RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB5RSTSR_OFFSET)
#define STM32_RCC_AHB1RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB1RSTCR_OFFSET)
#define STM32_RCC_AHB5RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB5RSTCR_OFFSET)
#define STM32_RCC_APB1LRSTSR          (STM32_RCC_BASE + STM32_RCC_APB1LRSTSR_OFFSET)
#define STM32_RCC_APB2RSTSR           (STM32_RCC_BASE + STM32_RCC_APB2RSTSR_OFFSET)
#define STM32_RCC_APB4LRSTSR          (STM32_RCC_BASE + STM32_RCC_APB4LRSTSR_OFFSET)
#define STM32_RCC_APB1LRSTCR          (STM32_RCC_BASE + STM32_RCC_APB1LRSTCR_OFFSET)
#define STM32_RCC_APB2RSTCR           (STM32_RCC_BASE + STM32_RCC_APB2RSTCR_OFFSET)
#define STM32_RCC_APB4LRSTCR          (STM32_RCC_BASE + STM32_RCC_APB4LRSTCR_OFFSET)
#define STM32_RCC_BUSENSR             (STM32_RCC_BASE + STM32_RCC_BUSENSR_OFFSET)

#define STM32_RCC_DIVENR              (STM32_RCC_BASE + STM32_RCC_DIVENR_OFFSET)
#define STM32_RCC_DIVENSR             (STM32_RCC_BASE + STM32_RCC_DIVENSR_OFFSET)

#define STM32_RCC_MEMENR              (STM32_RCC_BASE + STM32_RCC_MEMENR_OFFSET)
#define STM32_RCC_AHB4ENR             (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB1LENR            (STM32_RCC_BASE + STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB2ENR             (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB4LENR            (STM32_RCC_BASE + STM32_RCC_APB4LENR_OFFSET)
#define STM32_RCC_APB4HENR            (STM32_RCC_BASE + STM32_RCC_APB4HENR_OFFSET)
#define STM32_RCC_BUSLPENR            (STM32_RCC_BASE + STM32_RCC_BUSLPENR_OFFSET)
#define STM32_RCC_MEMLPENR            (STM32_RCC_BASE + STM32_RCC_MEMLPENR_OFFSET)
#define STM32_RCC_APB1LLPENR          (STM32_RCC_BASE + STM32_RCC_APB1LLPENR_OFFSET)
#define STM32_RCC_APB2LPENR           (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)

#define STM32_RCC_MEMENSR             (STM32_RCC_BASE + STM32_RCC_MEMENSR_OFFSET)
#define STM32_RCC_AHB4ENSR            (STM32_RCC_BASE + STM32_RCC_AHB4ENSR_OFFSET)
#define STM32_RCC_APB1LENSR           (STM32_RCC_BASE + STM32_RCC_APB1LENSR_OFFSET)
#define STM32_RCC_APB2ENSR            (STM32_RCC_BASE + STM32_RCC_APB2ENSR_OFFSET)
#define STM32_RCC_APB4LENSR           (STM32_RCC_BASE + STM32_RCC_APB4LENSR_OFFSET)
#define STM32_RCC_APB4HENSR           (STM32_RCC_BASE + STM32_RCC_APB4HENSR_OFFSET)
#define STM32_RCC_APB1LENCR           (STM32_RCC_BASE + STM32_RCC_APB1LENCR_OFFSET)
#define STM32_RCC_APB2ENCR            (STM32_RCC_BASE + STM32_RCC_APB2ENCR_OFFSET)
#define STM32_RCC_APB4LENCR           (STM32_RCC_BASE + STM32_RCC_APB4LENCR_OFFSET)
#define STM32_RCC_BUSLPENSR           (STM32_RCC_BASE + STM32_RCC_BUSLPENSR_OFFSET)
#define STM32_RCC_MEMLPENSR           (STM32_RCC_BASE + STM32_RCC_MEMLPENSR_OFFSET)
#define STM32_RCC_APB1LLPENSR         (STM32_RCC_BASE + STM32_RCC_APB1LLPENSR_OFFSET)
#define STM32_RCC_APB2LPENSR          (STM32_RCC_BASE + STM32_RCC_APB2LPENSR_OFFSET)

#define STM32_RCC_CCR                 (STM32_RCC_BASE + STM32_RCC_CCR_OFFSET)
#define STM32_RCC_CSR                 (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_PLL1ON                 (1 << 8)  /* Bit 8:  PLL1 enable */
#define RCC_CR_HSION                  (1 << 3)  /* Bit 3:  HSI enable */

/* Clock status register */

#define RCC_SR_PLL1RDY                (1 << 8)  /* Bit 8:  PLL1 clock ready */
#define RCC_SR_HSIRDY                 (1 << 3)  /* Bit 3:  HSI clock ready */

/* HSI configuration register (RM0486 section 14.10.11). */

#define RCC_HSICFGR_HSIDIV_SHIFT      (7)
#define RCC_HSICFGR_HSIDIV_MASK       (0x3u << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_DIV1       (0x0u << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_DIV2       (0x1u << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_DIV4       (0x2u << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_DIV8       (0x3u << RCC_HSICFGR_HSIDIV_SHIFT)

/* RCC clock configuration register 4 (RM0486 section 14.10.53). */

#define RCC_CCIPR4_I2C1SEL_SHIFT      (0)
#define RCC_CCIPR4_I2C1SEL_MASK       (0x7u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_PCLK1      (0x0u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_PER_CK     (0x1u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_IC10_CK    (0x2u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_IC15_CK    (0x3u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_MSI_CK     (0x4u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_HSI_DIV_CK (0x5u << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_SHIFT      (4)
#define RCC_CCIPR4_I2C2SEL_MASK       (0x7u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_PCLK1      (0x0u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_PER_CK     (0x1u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_IC10_CK    (0x2u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_IC15_CK    (0x3u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_MSI_CK     (0x4u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_HSI_DIV_CK (0x5u << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_SHIFT      (8)
#define RCC_CCIPR4_I2C3SEL_MASK       (0x7u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_PCLK1      (0x0u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_PER_CK     (0x1u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_IC10_CK    (0x2u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_IC15_CK    (0x3u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_MSI_CK     (0x4u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_HSI_DIV_CK (0x5u << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_SHIFT      (12)
#define RCC_CCIPR4_I2C4SEL_MASK       (0x7u << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_PCLK1      (0x0u << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_PER_CK     (0x1u << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_IC10_CK    (0x2u << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_IC15_CK    (0x3u << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_MSI_CK     (0x4u << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_HSI_DIV_CK (0x5u << RCC_CCIPR4_I2C4SEL_SHIFT)

/* Clock configuration register 1.  SYSSW = 0b11 selects three IC dividers
 * (IC2 for SYSCLK, IC6 for AHB, IC11 for APB) -- the SVD names this state
 * after the first IC only.
 */

#define RCC_CFGR1_SYSSWS_SHIFT        (28)
#define RCC_CFGR1_SYSSWS_MASK         (0x3 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSWS_IC2_IC6_IC11 (3 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSW_SHIFT         (24)
#define RCC_CFGR1_SYSSW_MASK          (0x3 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSW_IC2_IC6_IC11  (3 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_CPUSWS_SHIFT        (20)
#define RCC_CFGR1_CPUSWS_MASK         (0x3 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSWS_IC1          (3 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSW_SHIFT         (16)
#define RCC_CFGR1_CPUSW_MASK          (0x3 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSW_IC1           (3 << RCC_CFGR1_CPUSW_SHIFT)

/* Clock configuration register 2.  TIMPRE divides sys_bus_ck for timer
 * kernels, HPRE divides sys_bus_ck for the AHB bus, and PPRE1/2 divide
 * sys_bus2_ck for their APB buses.
 */

#define RCC_CFGR2_TIMPRE_SHIFT        (24)
#define RCC_CFGR2_TIMPRE_MASK         (0x3 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSBUS       (0 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSBUSd2     (1 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSBUSd4     (2 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSBUSd8     (3 << RCC_CFGR2_TIMPRE_SHIFT)

#define RCC_CFGR2_HPRE_SHIFT          (20)
#define RCC_CFGR2_HPRE_MASK           (0x7 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLK         (0 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd2       (1 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd4       (2 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd8       (3 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd16      (4 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd32      (5 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd64      (6 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd128     (7 << RCC_CFGR2_HPRE_SHIFT)

#define RCC_CFGR2_PPRE2_SHIFT         (4)
#define RCC_CFGR2_PPRE2_MASK          (0x7 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2       (0 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d2     (1 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d4     (2 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d8     (3 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d16    (4 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d32    (5 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d64    (6 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_SYSBUS2d128   (7 << RCC_CFGR2_PPRE2_SHIFT)

#define RCC_CFGR2_PPRE1_SHIFT         (0)
#define RCC_CFGR2_PPRE1_MASK          (0x7 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2       (0 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d2     (1 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d4     (2 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d8     (3 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d16    (4 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d32    (5 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d64    (6 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_SYSBUS2d128   (7 << RCC_CFGR2_PPRE1_SHIFT)

#define RCC_CFGR2                     STM32_RCC_CFGR2

/* PLL1 configuration register 1 */

#define RCC_PLL1CFGR1_SEL_SHIFT       (28)
#define RCC_PLL1CFGR1_SEL_MASK        (0x7 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_SEL_HSI         (0 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_DIVM_SHIFT      (20)       /* Bits 25-20: Reference divider */
#define RCC_PLL1CFGR1_DIVM_MASK       (0x3f << RCC_PLL1CFGR1_DIVM_SHIFT)
#define RCC_PLL1CFGR1_DIVN_SHIFT      (8)        /* Bits 19-8: Feedback divider */
#define RCC_PLL1CFGR1_DIVN_MASK       (0xfff << RCC_PLL1CFGR1_DIVN_SHIFT)

/* PLL1 configuration register 3 */

#define RCC_PLL1CFGR3_PDIVEN          (1 << 30)  /* Bit 30: Post-divider and PLL output enable */
#define RCC_PLL1CFGR3_PDIV1_SHIFT     (27)       /* Bits 29-27: Post-divider 1 */
#define RCC_PLL1CFGR3_PDIV1_MASK      (0x7 << RCC_PLL1CFGR3_PDIV1_SHIFT)
#define RCC_PLL1CFGR3_PDIV2_SHIFT     (24)       /* Bits 26-24: Post-divider 2 */
#define RCC_PLL1CFGR3_PDIV2_MASK      (0x7 << RCC_PLL1CFGR3_PDIV2_SHIFT)
#define RCC_PLL1CFGR3_MODSSDIS        (1 << 2)   /* Bit 2:  Modulation spread spectrum disable */

/* IC1..IC20 configuration registers -- all share the same layout.  Field
 * SEL selects the PLL source (PLL1..PLL4); field INT is an 8-bit integer
 * divider where INT[7:0] = N-1 yielding a divide ratio of N.
 */

#define RCC_ICCFGR_SEL_SHIFT          (28)
#define RCC_ICCFGR_SEL_MASK           (0x3 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_SEL_PLL1           (0 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_INT_SHIFT          (16)
#define RCC_ICCFGR_INT_MASK           (0xff << RCC_ICCFGR_INT_SHIFT)

/* IC divider enable register */

#define RCC_DIVENR_IC11EN             (1 << 10)  /* Bit 10: IC11 enable */
#define RCC_DIVENR_IC6EN              (1 << 5)   /* Bit 5:  IC6 enable */
#define RCC_DIVENR_IC3EN              (1 << 2)   /* Bit 2:  IC3 enable */
#define RCC_DIVENR_IC2EN              (1 << 1)   /* Bit 1:  IC2 enable */
#define RCC_DIVENR_IC1EN              (1 << 0)   /* Bit 0:  IC1 enable */

/* SRAM clock enable register */

#define RCC_MEMENR_CACHEAXIRAMEN      (1 << 10)  /* Bit 10: CACHEAXIRAM enable */
#define RCC_MEMENR_AXISRAM2EN         (1 << 8)   /* Bit 8:  AXISRAM2 enable */
#define RCC_MEMENR_AXISRAM1EN         (1 << 7)   /* Bit 7:  AXISRAM1 enable */
#define RCC_MEMENR_AXISRAM6EN         (1 << 3)   /* Bit 3:  AXISRAM6 enable */
#define RCC_MEMENR_AXISRAM5EN         (1 << 2)   /* Bit 2:  AXISRAM5 enable */
#define RCC_MEMENR_AXISRAM4EN         (1 << 1)   /* Bit 1:  AXISRAM4 enable */
#define RCC_MEMENR_AXISRAM3EN         (1 << 0)   /* Bit 0:  AXISRAM3 enable */

#define RCC_MEMENR_ALLAXISRAM         (RCC_MEMENR_AXISRAM1EN | RCC_MEMENR_AXISRAM2EN | \
                                       RCC_MEMENR_AXISRAM3EN | RCC_MEMENR_AXISRAM4EN | \
                                       RCC_MEMENR_AXISRAM5EN | RCC_MEMENR_AXISRAM6EN)

/* AHB4 peripheral clock enable register */

#define RCC_AHB4ENR_PWREN             (1 << 18)  /* Bit 18: PWR enable */
#define RCC_AHB4ENR_GPIOQEN           (1 << 16)  /* Bit 16: GPIOQ enable */
#define RCC_AHB4ENR_GPIOPEN           (1 << 15)  /* Bit 15: GPIOP enable */
#define RCC_AHB4ENR_GPIOOEN           (1 << 14)  /* Bit 14: GPIOO enable */
#define RCC_AHB4ENR_GPIONEN           (1 << 13)  /* Bit 13: GPION enable */
#define RCC_AHB4ENR_GPIOHEN           (1 << 7)   /* Bit 7:  GPIOH enable */
#define RCC_AHB4ENR_GPIOGEN           (1 << 6)   /* Bit 6:  GPIOG enable */
#define RCC_AHB4ENR_GPIOFEN           (1 << 5)   /* Bit 5:  GPIOF enable */
#define RCC_AHB4ENR_GPIOEEN           (1 << 4)   /* Bit 4:  GPIOE enable */
#define RCC_AHB4ENR_GPIODEN           (1 << 3)   /* Bit 3:  GPIOD enable */
#define RCC_AHB4ENR_GPIOCEN           (1 << 2)   /* Bit 2:  GPIOC enable */
#define RCC_AHB4ENR_GPIOBEN           (1 << 1)   /* Bit 1:  GPIOB enable */
#define RCC_AHB4ENR_GPIOAEN           (1 << 0)   /* Bit 0:  GPIOA enable */

/* APB1 peripheral reset register 1 */

#define RCC_APB1LRSTR_I2C1RST         (1 << 21)  /* Bit 21: I2C1 reset */
#define RCC_APB1LRSTR_I2C2RST         (1 << 22)  /* Bit 22: I2C2 reset */
#define RCC_APB1LRSTR_I2C3RST         (1 << 23)  /* Bit 23: I2C3 reset */
#define RCC_APB1LRSTR_TIM2RST         (1 << 0)   /* Bit 0:  TIM2 reset */
#define RCC_APB1LRSTR_TIM3RST         (1 << 1)   /* Bit 1:  TIM3 reset */
#define RCC_APB1LRSTR_TIM4RST         (1 << 2)   /* Bit 2:  TIM4 reset */
#define RCC_APB1LRSTR_TIM5RST         (1 << 3)   /* Bit 3:  TIM5 reset */
#define RCC_APB1LRSTR_TIM6RST         (1 << 4)   /* Bit 4:  TIM6 reset */
#define RCC_APB1LRSTR_TIM7RST         (1 << 5)   /* Bit 5:  TIM7 reset */
#define RCC_APB1LRSTR_TIM12RST        (1 << 6)   /* Bit 6:  TIM12 reset */
#define RCC_APB1LRSTR_TIM13RST        (1 << 7)   /* Bit 7:  TIM13 reset */
#define RCC_APB1LRSTR_TIM14RST        (1 << 8)   /* Bit 8:  TIM14 reset */
#define RCC_APB1LRSTR_TIM10RST        (1 << 12)  /* Bit 12: TIM10 reset */
#define RCC_APB1LRSTR_TIM11RST        (1 << 13)  /* Bit 13: TIM11 reset */

/* APB2 peripheral reset register */

#define RCC_APB2RSTR_TIM1RST          (1 << 0)   /* Bit 0:  TIM1 reset */
#define RCC_APB2RSTR_TIM8RST          (1 << 1)   /* Bit 1:  TIM8 reset */
#define RCC_APB2RSTR_TIM18RST         (1 << 15)  /* Bit 15: TIM18 reset */
#define RCC_APB2RSTR_TIM15RST         (1 << 16)  /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST         (1 << 17)  /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST         (1 << 18)  /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_TIM9RST          (1 << 19)  /* Bit 19: TIM9 reset */
#define RCC_APB2RSTR_SPI5RST           (1 << 20)  /* Bit 20: SPI5 reset */
#define RCC_APB2RSTR_SPI4RST           (1 << 13)  /* Bit 13: SPI4 reset */
#define RCC_APB2RSTR_SPI1RST           (1 << 12)  /* Bit 12: SPI1 reset */
#define RCC_APB1LRSTR_SPI3RST          (1 << 15)  /* Bit 15: SPI3 reset */
#define RCC_APB1LRSTR_SPI2RST          (1 << 14)  /* Bit 14: SPI2 reset */
#define RCC_APB4LRSTR_SPI6RST          (1 << 5)   /* Bit 5:  SPI6 reset */
#define RCC_APB4LRSTR_I2C4RST          (1 << 7)   /* Bit 7: I2C4 reset */

/* APB peripheral reset set and clear registers */

#define RCC_APB1LRSTSR_I2C1RSTS        (1 << 21)  /* Bit 21: I2C1 reset set */
#define RCC_APB1LRSTSR_I2C2RSTS        (1 << 22)  /* Bit 22: I2C2 reset set */
#define RCC_APB1LRSTSR_I2C3RSTS        (1 << 23)  /* Bit 23: I2C3 reset set */
#define RCC_APB1LRSTCR_I2C1RSTC        (1 << 21)  /* Bit 21: I2C1 reset clear */
#define RCC_APB1LRSTCR_I2C2RSTC        (1 << 22)  /* Bit 22: I2C2 reset clear */
#define RCC_APB1LRSTCR_I2C3RSTC        (1 << 23)  /* Bit 23: I2C3 reset clear */
#define RCC_APB1LRSTSR_SPI3RSTS        (1 << 15)  /* Bit 15: SPI3 reset set */
#define RCC_APB1LRSTSR_SPI2RSTS        (1 << 14)  /* Bit 14: SPI2 reset set */
#define RCC_APB1LRSTCR_SPI3RSTC        (1 << 15)  /* Bit 15: SPI3 reset clear */
#define RCC_APB1LRSTCR_SPI2RSTC        (1 << 14)  /* Bit 14: SPI2 reset clear */
#define RCC_APB2RSTSR_SPI5RSTS         (1 << 20)  /* Bit 20: SPI5 reset set */
#define RCC_APB2RSTSR_SPI4RSTS         (1 << 13)  /* Bit 13: SPI4 reset set */
#define RCC_APB2RSTSR_SPI1RSTS         (1 << 12)  /* Bit 12: SPI1 reset set */
#define RCC_APB2RSTCR_SPI5RSTC         (1 << 20)  /* Bit 20: SPI5 reset clear */
#define RCC_APB2RSTCR_SPI4RSTC         (1 << 13)  /* Bit 13: SPI4 reset clear */
#define RCC_APB2RSTCR_SPI1RSTC         (1 << 12)  /* Bit 12: SPI1 reset clear */
#define RCC_APB4LRSTSR_SPI6RSTS        (1 << 5)   /* Bit 5:  SPI6 reset set */
#define RCC_APB4LRSTCR_SPI6RSTC        (1 << 5)   /* Bit 5:  SPI6 reset clear */
#define RCC_APB4LRSTSR_I2C4RSTS        (1 << 7)   /* Bit 7: I2C4 reset set */
#define RCC_APB4LRSTCR_I2C4RSTC        (1 << 7)   /* Bit 7: I2C4 reset clear */

/* APB1 peripheral clock enable register 1 */

#define RCC_APB1LENR_I2C1EN           (1 << 21)  /* Bit 21: I2C1 enable */
#define RCC_APB1LENR_I2C2EN           (1 << 22)  /* Bit 22: I2C2 enable */
#define RCC_APB1LENR_I2C3EN           (1 << 23)  /* Bit 23: I2C3 enable */
#define RCC_APB1LENR_TIM2EN           (1 << 0)   /* Bit 0:  TIM2 enable */
#define RCC_APB1LENR_TIM3EN           (1 << 1)   /* Bit 1:  TIM3 enable */
#define RCC_APB1LENR_TIM4EN           (1 << 2)   /* Bit 2:  TIM4 enable */
#define RCC_APB1LENR_TIM5EN           (1 << 3)   /* Bit 3:  TIM5 enable */
#define RCC_APB1LENR_TIM6EN           (1 << 4)   /* Bit 4:  TIM6 enable */
#define RCC_APB1LENR_TIM7EN           (1 << 5)   /* Bit 5:  TIM7 enable */
#define RCC_APB1LENR_TIM12EN          (1 << 6)   /* Bit 6:  TIM12 enable */
#define RCC_APB1LENR_TIM13EN          (1 << 7)   /* Bit 7:  TIM13 enable */
#define RCC_APB1LENR_TIM14EN          (1 << 8)   /* Bit 8:  TIM14 enable */
#define RCC_APB1LENR_TIM10EN          (1 << 12)  /* Bit 12: TIM10 enable */
#define RCC_APB1LENR_TIM11EN          (1 << 13)  /* Bit 13: TIM11 enable */

/* APB2 peripheral clock enable register */

#define RCC_APB2ENR_TIM1EN            (1 << 0)   /* Bit 0:  TIM1 enable */
#define RCC_APB2ENR_TIM8EN            (1 << 1)   /* Bit 1:  TIM8 enable */
#define RCC_APB2ENR_USART1EN          (1 << 4)   /* Bit 4:  USART1 enable */
#define RCC_APB2ENR_TIM18EN           (1 << 15)  /* Bit 15: TIM18 enable */
#define RCC_APB2ENR_TIM15EN           (1 << 16)  /* Bit 16: TIM15 enable */
#define RCC_APB2ENR_TIM16EN           (1 << 17)  /* Bit 17: TIM16 enable */
#define RCC_APB2ENR_TIM17EN           (1 << 18)  /* Bit 18: TIM17 enable */
#define RCC_APB2ENR_TIM9EN            (1 << 19)  /* Bit 19: TIM9 enable */
#define RCC_APB2ENR_SPI5EN            (1 << 20)  /* Bit 20: SPI5 enable */
#define RCC_APB2ENR_SPI4EN            (1 << 13)  /* Bit 13: SPI4 enable */
#define RCC_APB2ENR_SPI1EN            (1 << 12)  /* Bit 12: SPI1 enable */
#define RCC_APB1LENR_SPI3EN           (1 << 15)  /* Bit 15: SPI3 enable */
#define RCC_APB1LENR_SPI2EN           (1 << 14)  /* Bit 14: SPI2 enable */
#define RCC_APB4LENR_SPI6EN           (1 << 5)   /* Bit 5:  SPI6 enable */
#define RCC_APB4LENR_I2C4EN           (1 << 7)   /* Bit 7: I2C4 enable */

/* APB peripheral clock enable set and clear registers */

#define RCC_APB1LENSR_I2C1ENS         (1 << 21)  /* Bit 21: I2C1 enable set */
#define RCC_APB1LENSR_I2C2ENS         (1 << 22)  /* Bit 22: I2C2 enable set */
#define RCC_APB1LENSR_I2C3ENS         (1 << 23)  /* Bit 23: I2C3 enable set */
#define RCC_APB1LENCR_I2C1ENC         (1 << 21)  /* Bit 21: I2C1 enable clear */
#define RCC_APB1LENCR_I2C2ENC         (1 << 22)  /* Bit 22: I2C2 enable clear */
#define RCC_APB1LENCR_I2C3ENC         (1 << 23)  /* Bit 23: I2C3 enable clear */
#define RCC_APB1LENSR_SPI3ENS         (1 << 15)  /* Bit 15: SPI3 enable set */
#define RCC_APB1LENSR_SPI2ENS         (1 << 14)  /* Bit 14: SPI2 enable set */
#define RCC_APB1LENCR_SPI3ENC         (1 << 15)  /* Bit 15: SPI3 enable clear */
#define RCC_APB1LENCR_SPI2ENC         (1 << 14)  /* Bit 14: SPI2 enable clear */
#define RCC_APB2ENSR_SPI5ENS          (1 << 20)  /* Bit 20: SPI5 enable set */
#define RCC_APB2ENSR_SPI4ENS          (1 << 13)  /* Bit 13: SPI4 enable set */
#define RCC_APB2ENSR_SPI1ENS          (1 << 12)  /* Bit 12: SPI1 enable set */
#define RCC_APB2ENCR_SPI5ENC          (1 << 20)  /* Bit 20: SPI5 enable clear */
#define RCC_APB2ENCR_SPI4ENC          (1 << 13)  /* Bit 13: SPI4 enable clear */
#define RCC_APB2ENCR_SPI1ENC          (1 << 12)  /* Bit 12: SPI1 enable clear */
#define RCC_APB4LENSR_SPI6ENS         (1 << 5)   /* Bit 5:  SPI6 enable set */
#define RCC_APB4LENCR_SPI6ENC         (1 << 5)   /* Bit 5:  SPI6 enable clear */
#define RCC_APB4LENSR_I2C4ENS         (1 << 7)   /* Bit 7: I2C4 enable set */
#define RCC_APB4LENCR_I2C4ENC         (1 << 7)   /* Bit 7: I2C4 enable clear */

/* APB4 peripheral clock enable register 2 */

#define RCC_APB4HENR_BSECEN           (1 << 1)   /* Bit 1:  BSEC enable */
#define RCC_APB4HENR_SYSCFGEN         (1 << 0)   /* Bit 0:  SYSCFG enable */

/* AHB1/AHB5 DMA reset and clock controls */

#define RCC_AHB1RSTSR_GPDMA1RSTS      (1 << 4)   /* Bit 4:  GPDMA1 reset set */
#define RCC_AHB5RSTSR_HPDMA1RSTS      (1 << 0)   /* Bit 0:  HPDMA1 reset set */
#define RCC_AHB1RSTCR_GPDMA1RSTC      (1 << 4)   /* Bit 4:  GPDMA1 reset clear */
#define RCC_AHB5RSTCR_HPDMA1RSTC      (1 << 0)   /* Bit 0:  HPDMA1 reset clear */
#define RCC_AHB1ENSR_GPDMA1ENS        (1 << 4)   /* Bit 4:  GPDMA1 clock enable set */
#define RCC_AHB5ENSR_HPDMA1ENS        (1 << 0)   /* Bit 0:  HPDMA1 clock enable set */

/* Embedded bus clocks required by HPDMA1 */

#define RCC_BUSENSR_ACLKNENS          (1 << 0)   /* Bit 0:  ACLKN clock enable set */
#define RCC_BUSENSR_ACLKNCENS         (1 << 1)   /* Bit 1:  ACLKNC clock enable set */

/* Bus clock enable in Sleep mode */

#define RCC_BUSLPENR_ACLKNCLPEN       (1 << 1)   /* Bit 1:  ACLKNC clock enable in CSLEEP */
#define RCC_BUSLPENR_ACLKNLPEN        (1 << 0)   /* Bit 0:  ACLKN clock enable in CSLEEP */

/* SRAM clock enable in Sleep mode */

#define RCC_MEMLPENR_CACHEAXIRAMLPEN  (1 << 10)  /* Bit 10: CACHEAXIRAM enable in CSLEEP */
#define RCC_MEMLPENR_AXISRAM2LPEN     (1 << 8)
#define RCC_MEMLPENR_AXISRAM1LPEN     (1 << 7)
#define RCC_MEMLPENR_AXISRAM6LPEN     (1 << 3)
#define RCC_MEMLPENR_AXISRAM5LPEN     (1 << 2)
#define RCC_MEMLPENR_AXISRAM4LPEN     (1 << 1)
#define RCC_MEMLPENR_AXISRAM3LPEN     (1 << 0)

#define RCC_MEMLPENR_ALLAXISRAM       (RCC_MEMLPENR_AXISRAM1LPEN | RCC_MEMLPENR_AXISRAM2LPEN | \
                                       RCC_MEMLPENR_AXISRAM3LPEN | RCC_MEMLPENR_AXISRAM4LPEN | \
                                       RCC_MEMLPENR_AXISRAM5LPEN | RCC_MEMLPENR_AXISRAM6LPEN)

/* APB1 peripheral clock enable in Sleep mode (register 1) */

#define RCC_APB1LLPENR_TIM2LPEN       (1 << 0)   /* Bit 0:  TIM2 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM3LPEN       (1 << 1)   /* Bit 1:  TIM3 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM4LPEN       (1 << 2)   /* Bit 2:  TIM4 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM5LPEN       (1 << 3)   /* Bit 3:  TIM5 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM6LPEN       (1 << 4)   /* Bit 4:  TIM6 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM7LPEN       (1 << 5)   /* Bit 5:  TIM7 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM12LPEN      (1 << 6)   /* Bit 6:  TIM12 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM13LPEN      (1 << 7)   /* Bit 7:  TIM13 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM14LPEN      (1 << 8)   /* Bit 8:  TIM14 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM10LPEN      (1 << 12)  /* Bit 12: TIM10 enable in CSLEEP */
#define RCC_APB1LLPENR_TIM11LPEN      (1 << 13)  /* Bit 13: TIM11 enable in CSLEEP */

/* APB2 peripheral clock enable in Sleep mode */

#define RCC_APB2LPENR_TIM1LPEN        (1 << 0)   /* Bit 0:  TIM1 enable in CSLEEP */
#define RCC_APB2LPENR_TIM8LPEN        (1 << 1)   /* Bit 1:  TIM8 enable in CSLEEP */
#define RCC_APB2LPENR_USART1LPEN      (1 << 4)   /* Bit 4:  USART1 enable in CSLEEP */
#define RCC_APB2LPENR_TIM18LPEN       (1 << 15)  /* Bit 15: TIM18 enable in CSLEEP */
#define RCC_APB2LPENR_TIM15LPEN       (1 << 16)  /* Bit 16: TIM15 enable in CSLEEP */
#define RCC_APB2LPENR_TIM16LPEN       (1 << 17)  /* Bit 17: TIM16 enable in CSLEEP */
#define RCC_APB2LPENR_TIM17LPEN       (1 << 18)  /* Bit 18: TIM17 enable in CSLEEP */
#define RCC_APB2LPENR_TIM9LPEN        (1 << 19)  /* Bit 19: TIM9 enable in CSLEEP */

/* Peripheral kernel clock select register 13 */

#define RCC_CCIPR13_USART1SEL_SHIFT   (0)
#define RCC_CCIPR13_USART1SEL_MASK    (0x7 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_HSI     (6 << RCC_CCIPR13_USART1SEL_SHIFT)

/* Peripheral kernel clock select register 9 (RM0486 14.10.58) */

#define RCC_CCIPR9_SPI1SEL_SHIFT       (4)
#define RCC_CCIPR9_SPI1SEL_MASK        (0x7 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_PCLK2       (0 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_PER_CK      (1 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_IC8_CK      (2 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_IC9_CK      (3 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_MSI_CK      (4 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_HSI_DIV_CK  (5 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_I2S_CKIN    (6 << RCC_CCIPR9_SPI1SEL_SHIFT)

#define RCC_CCIPR9_SPI2SEL_SHIFT       (8)
#define RCC_CCIPR9_SPI2SEL_MASK        (0x7 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_PCLK1       (0 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_PER_CK      (1 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_IC8_CK      (2 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_IC9_CK      (3 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_MSI_CK      (4 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_HSI_DIV_CK  (5 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_I2S_CKIN    (6 << RCC_CCIPR9_SPI2SEL_SHIFT)

#define RCC_CCIPR9_SPI3SEL_SHIFT       (12)
#define RCC_CCIPR9_SPI3SEL_MASK        (0x7 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_PCLK1       (0 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_PER_CK      (1 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_IC8_CK      (2 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_IC9_CK      (3 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_MSI_CK      (4 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_HSI_DIV_CK  (5 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_I2S_CKIN    (6 << RCC_CCIPR9_SPI3SEL_SHIFT)

#define RCC_CCIPR9_SPI4SEL_SHIFT       (16)
#define RCC_CCIPR9_SPI4SEL_MASK        (0x7 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_PCLK2       (0 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_PER_CK      (1 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_IC9_CK      (2 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_IC14_CK     (3 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_MSI_CK      (4 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_HSI_DIV_CK  (5 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_HSE_CK      (6 << RCC_CCIPR9_SPI4SEL_SHIFT)

#define RCC_CCIPR9_SPI5SEL_SHIFT       (20)
#define RCC_CCIPR9_SPI5SEL_MASK        (0x7 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_PCLK2       (0 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_PER_CK      (1 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_IC9_CK      (2 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_IC14_CK     (3 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_MSI_CK      (4 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_HSI_DIV_CK  (5 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_HSE_CK      (6 << RCC_CCIPR9_SPI5SEL_SHIFT)

#define RCC_CCIPR9_SPI6SEL_SHIFT       (24)
#define RCC_CCIPR9_SPI6SEL_MASK        (0x7 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_PCLK4       (0 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_PER_CK      (1 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_IC8_CK      (2 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_IC9_CK      (3 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_MSI_CK      (4 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_HSI_DIV_CK  (5 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_I2S_CKIN    (6 << RCC_CCIPR9_SPI6SEL_SHIFT)

#endif /* CONFIG_STM32_STM32N6XXXX */
#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H */
