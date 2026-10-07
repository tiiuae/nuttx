/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_usbphyc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_USBPHYC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_USBPHYC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RM0486 chapter 74. STM32_USBPHYC_BASE selects USB1 or USB2.
 * Registers use 32-bit accesses. Keep recommended trimming at reset.
 */

#define STM32_USBPHYC_CR_OFFSET         0x0000
#define STM32_USBPHYC_TRIM1CR_OFFSET    0x0004
#define STM32_USBPHYC_TRIM2CR_OFFSET    0x0008
#define STM32_USBPHYC_CR                (STM32_USBPHYC_BASE + \
                                        STM32_USBPHYC_CR_OFFSET)
#define STM32_USBPHYC_TRIM1CR           (STM32_USBPHYC_BASE + \
                                        STM32_USBPHYC_TRIM1CR_OFFSET)
#define STM32_USBPHYC_TRIM2CR           (STM32_USBPHYC_BASE + \
                                        STM32_USBPHYC_TRIM2CR_OFFSET)
#define USBPHYC_CR_RESET                0x00010015u
#define USBPHYC_CR_RETENABLEN1          (1u << 0)
#define USBPHYC_CR_AUTORSMENB1          (1u << 1)
#define USBPHYC_CR_CMN                  (1u << 2)
#define USBPHYC_CR_FSEL_SHIFT           4
#define USBPHYC_CR_FSEL_MASK            (7u << 4)
#define USBPHYC_CR_FSEL_19P2MHZ         (0u << 4)
#define USBPHYC_CR_FSEL_20MHZ           (1u << 4)
#define USBPHYC_CR_FSEL_24MHZ           (2u << 4)
#define USBPHYC_CR_OTGDISABLE0          (1u << 16)
#define USBPHYC_CR_DRVVBUS0             (1u << 17)
#define USBPHYC2_CR_SELOTGDBG           (1u << 31) /* Reserved on USB1 */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_USBPHYC_H */
