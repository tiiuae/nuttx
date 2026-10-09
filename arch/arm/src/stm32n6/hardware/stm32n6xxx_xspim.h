/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_xspim.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_XSPIM_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_XSPIM_H

#include <nuttx/config.h>
#include "hardware/stm32n6xxx_memorymap.h"

/* XSPIM register definitions (RM0486 29.6.1 and Table 205). */

#define STM32_XSPIM_CR_OFFSET      0x0000
#define STM32_XSPIM_CR             (STM32_XSPIM_BASE + STM32_XSPIM_CR_OFFSET)
#define XSPIM_CR_MUXEN             (1u << 0)
#define XSPIM_CR_MODE              (1u << 1)
#define XSPIM_CR_CSSEL_OVR_EN      (1u << 4)
#define XSPIM_CR_CSSEL_OVR_O1      (1u << 5)
#define XSPIM_CR_CSSEL_OVR_O2      (1u << 6)
#define XSPIM_CR_REQ2ACK_SHIFT     16
#define XSPIM_CR_REQ2ACK_MASK      (0xffu << XSPIM_CR_REQ2ACK_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_XSPIM_H */
