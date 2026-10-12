/****************************************************************************
 * arch/arm/src/stm32n6/stm32_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32N6_STM32_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/stm32n6xxx_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables write access to the backup domain (RTC registers, RTC backup
 *   data registers and backup SRAM).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool stm32_pwr_enablebkp(bool writable);

/****************************************************************************
 * Name: stm32_pwr_configvddio
 *
 * Description:
 *   Configure the I/O supply and voltage-range bits in PWR SVMCR3.
 *   Board policy passes separate masks for bits to set and clear so the
 *   voltage-range selection is correct even after a standby wake-up.
 *
 ****************************************************************************/

void stm32_pwr_configvddio(uint32_t setmask, uint32_t clearmask);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32N6_STM32_PWR_H */
