/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_EXTI_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO EXTI register offsets ************************************************/

#define STM32_EXTI_RTSR1_OFFSET    0x000 /* Rising trigger 1 */
#define STM32_EXTI_FTSR1_OFFSET    0x004 /* Falling trigger 1 */
#define STM32_EXTI_RPR1_OFFSET     0x00c /* Rising edge pending register 1 */
#define STM32_EXTI_FPR1_OFFSET     0x010 /* Falling edge pending register 1 */
#define STM32_EXTI_EXTICR1_OFFSET  0x060 /* GPIO source selection 1 */
#define STM32_EXTI_EXTICR2_OFFSET  0x064 /* GPIO source selection 2 */
#define STM32_EXTI_EXTICR3_OFFSET  0x068 /* GPIO source selection 3 */
#define STM32_EXTI_EXTICR4_OFFSET  0x06c /* GPIO source selection 4 */
#define STM32_EXTI_IMR1_OFFSET     0x080 /* Interrupt mask 1 */
#define STM32_EXTI_EMR1_OFFSET     0x084 /* Event mask 1 */

/* GPIO EXTI register addresses *********************************************/

#define STM32_EXTI_RTSR1           (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1           (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_RPR1            (STM32_EXTI_BASE + STM32_EXTI_RPR1_OFFSET)
#define STM32_EXTI_FPR1            (STM32_EXTI_BASE + STM32_EXTI_FPR1_OFFSET)
#define STM32_EXTI_EXTICR1         (STM32_EXTI_BASE + STM32_EXTI_EXTICR1_OFFSET)
#define STM32_EXTI_EXTICR2         (STM32_EXTI_BASE + STM32_EXTI_EXTICR2_OFFSET)
#define STM32_EXTI_EXTICR3         (STM32_EXTI_BASE + STM32_EXTI_EXTICR3_OFFSET)
#define STM32_EXTI_EXTICR4         (STM32_EXTI_BASE + STM32_EXTI_EXTICR4_OFFSET)
#define STM32_EXTI_IMR1            (STM32_EXTI_BASE + STM32_EXTI_IMR1_OFFSET)
#define STM32_EXTI_EMR1            (STM32_EXTI_BASE + STM32_EXTI_EMR1_OFFSET)

/* GPIO EXTI lines 0-15 ******************************************************/

#define STM32_EXTI_GPIO_LINE_MASK(n) (1 << (n))

/* Each EXTICR selector is one byte. Port values match the GPIO port index:
 * A-H are 0x00-0x07 and N-Q are 0x08-0x0b.
 */

#define STM32_EXTI_EXTICR_SHIFT(n)   (((n) & 3) << 3)
#define STM32_EXTI_EXTICR_MASK(n)    (0xff << STM32_EXTI_EXTICR_SHIFT(n))
#define STM32_EXTI_EXTICR_VALUE(n, port) \
  ((port) << STM32_EXTI_EXTICR_SHIFT(n))

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_EXTI_H */
