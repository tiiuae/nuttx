/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_xspi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_XSPI_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_XSPI_H

#include <nuttx/config.h>
#include "hardware/stm32n6xxx_memorymap.h"

/* XSPI register offsets (RM0486 Table 201). */

#define STM32_XSPI_CR_OFFSET       0x000
#define STM32_XSPI_DCR1_OFFSET     0x008
#define STM32_XSPI_DCR2_OFFSET     0x00c
#define STM32_XSPI_DCR3_OFFSET     0x010
#define STM32_XSPI_DCR4_OFFSET     0x014
#define STM32_XSPI_SR_OFFSET       0x020
#define STM32_XSPI_FCR_OFFSET      0x024
#define STM32_XSPI_DLR_OFFSET      0x040
#define STM32_XSPI_AR_OFFSET       0x048
#define STM32_XSPI_DR_OFFSET       0x050
#define STM32_XSPI_CCR_OFFSET      0x100
#define STM32_XSPI_TCR_OFFSET      0x108
#define STM32_XSPI_IR_OFFSET       0x110

/* XSPI2 only is enabled by the initial N6 driver. */

#define STM32_XSPI2_CR             (STM32_XSPI2_BASE + STM32_XSPI_CR_OFFSET)
#define STM32_XSPI2_DCR1           (STM32_XSPI2_BASE + STM32_XSPI_DCR1_OFFSET)
#define STM32_XSPI2_DCR2           (STM32_XSPI2_BASE + STM32_XSPI_DCR2_OFFSET)
#define STM32_XSPI2_DCR3           (STM32_XSPI2_BASE + STM32_XSPI_DCR3_OFFSET)
#define STM32_XSPI2_DCR4           (STM32_XSPI2_BASE + STM32_XSPI_DCR4_OFFSET)
#define STM32_XSPI2_SR             (STM32_XSPI2_BASE + STM32_XSPI_SR_OFFSET)
#define STM32_XSPI2_FCR            (STM32_XSPI2_BASE + STM32_XSPI_FCR_OFFSET)
#define STM32_XSPI2_DLR            (STM32_XSPI2_BASE + STM32_XSPI_DLR_OFFSET)
#define STM32_XSPI2_AR             (STM32_XSPI2_BASE + STM32_XSPI_AR_OFFSET)
#define STM32_XSPI2_DR             (STM32_XSPI2_BASE + STM32_XSPI_DR_OFFSET)
#define STM32_XSPI2_CCR            (STM32_XSPI2_BASE + STM32_XSPI_CCR_OFFSET)
#define STM32_XSPI2_TCR            (STM32_XSPI2_BASE + STM32_XSPI_TCR_OFFSET)
#define STM32_XSPI2_IR             (STM32_XSPI2_BASE + STM32_XSPI_IR_OFFSET)

/* CR */

#define XSPI_CR_EN                 (1u << 0)
#define XSPI_CR_ABORT              (1u << 1)
#define XSPI_CR_FTHRES_SHIFT       8
#define XSPI_CR_FTHRES_MASK        (0x3fu << XSPI_CR_FTHRES_SHIFT)
#define XSPI_CR_TEIE               (1u << 16)
#define XSPI_CR_TCIE               (1u << 17)
#define XSPI_CR_FTIE               (1u << 18)
#define XSPI_CR_SMIE               (1u << 19)
#define XSPI_CR_TOIE               (1u << 20)
#define XSPI_CR_DMM                (1u << 6)
#define XSPI_CR_DMAEN              (1u << 2)
#define XSPI_CR_TCEN               (1u << 3)
#define XSPI_CR_FMODE_SHIFT        28
#define XSPI_CR_FMODE_MASK         (3u << XSPI_CR_FMODE_SHIFT)
#define XSPI_CR_FMODE_INDIRECT_WRITE (0u << XSPI_CR_FMODE_SHIFT)
#define XSPI_CR_FMODE_INDIRECT_READ  (1u << XSPI_CR_FMODE_SHIFT)
#define XSPI_CR_FMODE_AUTOMATIC_POLL (2u << XSPI_CR_FMODE_SHIFT)
#define XSPI_CR_FMODE_MEMORY_MAPPED  (3u << XSPI_CR_FMODE_SHIFT)

/* DCR1 */

#define XSPI_DCR1_CSHT_SHIFT       8
#define XSPI_DCR1_CSHT_MASK        (0x3fu << XSPI_DCR1_CSHT_SHIFT)
#define XSPI_DCR1_DEVSIZE_SHIFT    16
#define XSPI_DCR1_DEVSIZE_MASK     (0x1fu << XSPI_DCR1_DEVSIZE_SHIFT)
#define XSPI_DCR1_MTYP_SHIFT       24
#define XSPI_DCR1_MTYP_MASK        (7u << XSPI_DCR1_MTYP_SHIFT)
#define XSPI_DCR1_CKMODE           (1u << 0) /* Read-only; N6 supports mode 0 */

/* DCR2 */

#define XSPI_DCR2_PRESCALER_MASK   0xffu

/* SR and FCR */

#define XSPI_SR_TEF                (1u << 0)
#define XSPI_SR_TCF                (1u << 1)
#define XSPI_SR_FTF                (1u << 2)
#define XSPI_SR_SMF                (1u << 3)
#define XSPI_SR_TOF                (1u << 4)
#define XSPI_SR_BUSY               (1u << 5)
#define XSPI_SR_FLEVEL_SHIFT       8
#define XSPI_SR_FLEVEL_MASK        (0x7fu << XSPI_SR_FLEVEL_SHIFT)
#define XSPI_FCR_CTEF              XSPI_SR_TEF
#define XSPI_FCR_CTCF              XSPI_SR_TCF
#define XSPI_FCR_CSMF              XSPI_SR_SMF
#define XSPI_FCR_CTOF              XSPI_SR_TOF

/* CCR phase encodings */

#define XSPI_CCR_IMODE_NONE        (0u << 0)
#define XSPI_CCR_IMODE_1LINE       (1u << 0)
#define XSPI_CCR_IMODE_2LINE       (2u << 0)
#define XSPI_CCR_IMODE_4LINE       (3u << 0)
#define XSPI_CCR_IMODE_8LINE       (4u << 0)
#define XSPI_CCR_ADMODE_SHIFT      8
#define XSPI_CCR_ADMODE_NONE       (0u << XSPI_CCR_ADMODE_SHIFT)
#define XSPI_CCR_ADMODE_1LINE      (1u << XSPI_CCR_ADMODE_SHIFT)
#define XSPI_CCR_ADMODE_2LINE      (2u << XSPI_CCR_ADMODE_SHIFT)
#define XSPI_CCR_ADMODE_4LINE      (3u << XSPI_CCR_ADMODE_SHIFT)
#define XSPI_CCR_ADMODE_8LINE      (4u << XSPI_CCR_ADMODE_SHIFT)
#define XSPI_CCR_ADSIZE_SHIFT      12
#define XSPI_CCR_ADSIZE_8BIT       (0u << XSPI_CCR_ADSIZE_SHIFT)
#define XSPI_CCR_ADSIZE_16BIT      (1u << XSPI_CCR_ADSIZE_SHIFT)
#define XSPI_CCR_ADSIZE_24BIT      (2u << XSPI_CCR_ADSIZE_SHIFT)
#define XSPI_CCR_ADSIZE_32BIT      (3u << XSPI_CCR_ADSIZE_SHIFT)
#define XSPI_CCR_ABMODE_SHIFT      16
#define XSPI_CCR_ABMODE_NONE       (0u << XSPI_CCR_ABMODE_SHIFT)
#define XSPI_CCR_ABSIZE_SHIFT      20
#define XSPI_CCR_ABSIZE_8BIT       (0u << XSPI_CCR_ABSIZE_SHIFT)
#define XSPI_CCR_ABSIZE_16BIT      (1u << XSPI_CCR_ABSIZE_SHIFT)
#define XSPI_CCR_ABSIZE_24BIT      (2u << XSPI_CCR_ABSIZE_SHIFT)
#define XSPI_CCR_ABSIZE_32BIT      (3u << XSPI_CCR_ABSIZE_SHIFT)
#define XSPI_CCR_DMODE_SHIFT       24
#define XSPI_CCR_DMODE_NONE        (0u << XSPI_CCR_DMODE_SHIFT)
#define XSPI_CCR_DMODE_1LINE       (1u << XSPI_CCR_DMODE_SHIFT)
#define XSPI_CCR_DMODE_2LINE       (2u << XSPI_CCR_DMODE_SHIFT)
#define XSPI_CCR_DMODE_4LINE       (3u << XSPI_CCR_DMODE_SHIFT)
#define XSPI_CCR_DMODE_8LINE       (4u << XSPI_CCR_DMODE_SHIFT)
#define XSPI_CCR_ISIZE_SHIFT       4
#define XSPI_CCR_ISIZE_8BIT        (0u << XSPI_CCR_ISIZE_SHIFT)
#define XSPI_CCR_ISIZE_16BIT       (1u << XSPI_CCR_ISIZE_SHIFT)
#define XSPI_CCR_ISIZE_24BIT       (2u << XSPI_CCR_ISIZE_SHIFT)
#define XSPI_CCR_ISIZE_32BIT       (3u << XSPI_CCR_ISIZE_SHIFT)

/* TCR */

#define XSPI_TCR_DCYC_SHIFT        0
#define XSPI_TCR_DCYC_MASK         (0x1fu << XSPI_TCR_DCYC_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_XSPI_H */
