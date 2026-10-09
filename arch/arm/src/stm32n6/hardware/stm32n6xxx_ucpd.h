/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_ucpd.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_UCPD_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_UCPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Static Type-C sink subset, RM0486 75.8. No PD transmitter/receiver. */

#define STM32_UCPD_CFGR1              (STM32_UCPD1_BASE + 0x0000)
#define STM32_UCPD_CFGR2              (STM32_UCPD1_BASE + 0x0004)
#define STM32_UCPD_CR                 (STM32_UCPD1_BASE + 0x000c)
#define STM32_UCPD_IMR                (STM32_UCPD1_BASE + 0x0010)
#define STM32_UCPD_SR                 (STM32_UCPD1_BASE + 0x0014)

#define UCPD_CFGR1_PSC_DIV2           (1u << 17)
#define UCPD_CFGR1_UCPDEN             (1u << 31)
#define UCPD_CR_ANAMODE_SINK          (1u << 9)
#define UCPD_CR_CCENABLE_BOTH         (3u << 10)
#define UCPD_SR_CC1_SHIFT             16
#define UCPD_SR_CC2_SHIFT             18
#define UCPD_SR_CC_MASK               3u

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_UCPD_H */
