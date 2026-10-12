/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMA_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMA_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common GPDMA1/HPDMA1 register offsets (RM0486 sections 18.8 and 19.8). */

#define STM32_DMA_SECCFGR_OFFSET       0x000
#define STM32_DMA_PRIVCFGR_OFFSET      0x004
#define STM32_DMA_RCFGLOCKR_OFFSET     0x008
#define STM32_DMA_MISR_OFFSET          0x00c
#define STM32_DMA_SMISR_OFFSET         0x010

#define STM32_DMA_CXLBAR_OFFSET(ch)    (0x050 + 0x80 * (ch))
#define STM32_DMA_CXFCR_OFFSET(ch)     (0x05c + 0x80 * (ch))
#define STM32_DMA_CXSR_OFFSET(ch)      (0x060 + 0x80 * (ch))
#define STM32_DMA_CXCR_OFFSET(ch)      (0x064 + 0x80 * (ch))
#define STM32_DMA_CXTR1_OFFSET(ch)     (0x090 + 0x80 * (ch))
#define STM32_DMA_CXTR2_OFFSET(ch)     (0x094 + 0x80 * (ch))
#define STM32_DMA_CXBR1_OFFSET(ch)     (0x098 + 0x80 * (ch))
#define STM32_DMA_CXSAR_OFFSET(ch)     (0x09c + 0x80 * (ch))
#define STM32_DMA_CXDAR_OFFSET(ch)     (0x0a0 + 0x80 * (ch))
#define STM32_DMA_CXLLR_OFFSET(ch)     (0x0cc + 0x80 * (ch))

/* TR3 and BR2 are implemented only on channels 12-15. */

#define STM32_DMA_CXTR3_OFFSET(ch)     (0x0a4 + 0x80 * (ch))
#define STM32_DMA_CXBR2_OFFSET(ch)     (0x0a8 + 0x80 * (ch))

/* Channel configuration and event flags. */

#define STM32_DMA_CHAN_MASK(ch)        (1u << (ch))

#define STM32_DMA_FLAG_TOF             (1u << 14)
#define STM32_DMA_FLAG_SUSPF           (1u << 13)
#define STM32_DMA_FLAG_USEF            (1u << 12)
#define STM32_DMA_FLAG_ULEF            (1u << 11)
#define STM32_DMA_FLAG_DTEF            (1u << 10)
#define STM32_DMA_FLAG_HTF             (1u << 9)
#define STM32_DMA_FLAG_TCF             (1u << 8)
#define STM32_DMA_FLAG_IDLEF           (1u << 0)
#define STM32_DMA_FLAG_CLEAR_MASK      (0x7f00u)

#define STM32_DMA_SECCFGR_SEC(ch)      STM32_DMA_CHAN_MASK(ch)
#define STM32_DMA_PRIVCFGR_PRIV(ch)    STM32_DMA_CHAN_MASK(ch)
#define STM32_DMA_RCFGLOCKR_LOCK(ch)   STM32_DMA_CHAN_MASK(ch)

#define STM32_DMA_CR_PRIO_SHIFT        22
#define STM32_DMA_CR_PRIO_MASK         (3u << STM32_DMA_CR_PRIO_SHIFT)
#define STM32_DMA_CR_LAP               (1u << 17)
#define STM32_DMA_CR_LSM               (1u << 16)
#define STM32_DMA_CR_TOIE              (1u << 14)
#define STM32_DMA_CR_SUSPIE            (1u << 13)
#define STM32_DMA_CR_USEIE             (1u << 12)
#define STM32_DMA_CR_ULEIE             (1u << 11)
#define STM32_DMA_CR_DTEIE             (1u << 10)
#define STM32_DMA_CR_HTIE              (1u << 9)
#define STM32_DMA_CR_TCIE              (1u << 8)
#define STM32_DMA_CR_SUSP              (1u << 2)
#define STM32_DMA_CR_RESET             (1u << 1)
#define STM32_DMA_CR_EN                (1u << 0)

/* TR1 source/destination width encodings are log2(bytes): 1, 2, 4, or 8
 * bytes. Controller-specific valid widths are declared in each controller
 * header; a peripheral can impose an additional restriction.
 */

#define STM32_DMA_WIDTH_1BYTE          0
#define STM32_DMA_WIDTH_2BYTES         1
#define STM32_DMA_WIDTH_4BYTES         2
#define STM32_DMA_WIDTH_8BYTES         3
#define STM32_DMA_TR1_DDW_SHIFT        16
#define STM32_DMA_TR1_DDW_MASK         (3u << STM32_DMA_TR1_DDW_SHIFT)
#define STM32_DMA_TR1_DBL_SHIFT        20
#define STM32_DMA_TR1_DBL_MASK         (0x3fu << STM32_DMA_TR1_DBL_SHIFT)
#define STM32_DMA_TR1_DINC             (1u << 19)
#define STM32_DMA_TR1_DAP              (1u << 30)
#define STM32_DMA_TR1_DSEC             (1u << 31)
#define STM32_DMA_TR1_SDW_SHIFT        0
#define STM32_DMA_TR1_SDW_MASK         (3u << STM32_DMA_TR1_SDW_SHIFT)
#define STM32_DMA_TR1_SBL_SHIFT        4
#define STM32_DMA_TR1_SBL_MASK         (0x3fu << STM32_DMA_TR1_SBL_SHIFT)
#define STM32_DMA_TR1_SINC             (1u << 3)
#define STM32_DMA_TR1_SAP              (1u << 14)
#define STM32_DMA_TR1_SSEC             (1u << 15)
#define STM32_DMA_TR1_DHX              (1u << 27)
#define STM32_DMA_TR1_DBX              (1u << 26)
#define STM32_DMA_TR1_SBX              (1u << 13)
#define STM32_DMA_TR1_PAM_SHIFT        11
#define STM32_DMA_TR1_PAM_MASK         (3u << STM32_DMA_TR1_PAM_SHIFT)
#define STM32_DMA_TR1_BURST_MAX        64

/* TR2 request, trigger and event controls. */

#define STM32_DMA_TR2_TCEM_SHIFT       30
#define STM32_DMA_TR2_TCEM_MASK        (3u << STM32_DMA_TR2_TCEM_SHIFT)
#define STM32_DMA_TR2_TCEM_LLI         (2u << STM32_DMA_TR2_TCEM_SHIFT)
#define STM32_DMA_TR2_TRIGPOL_SHIFT    24
#define STM32_DMA_TR2_TRIGPOL_MASK     (3u << STM32_DMA_TR2_TRIGPOL_SHIFT)
#define STM32_DMA_TR2_TRIGSEL_SHIFT    16
#define STM32_DMA_TR2_TRIGSEL_MASK     (0x7fu << STM32_DMA_TR2_TRIGSEL_SHIFT)
#define STM32_DMA_TR2_TRIGM_SHIFT      14
#define STM32_DMA_TR2_TRIGM_MASK       (3u << STM32_DMA_TR2_TRIGM_SHIFT)
#define STM32_DMA_TR2_PFREQ             (1u << 12)
#define STM32_DMA_TR2_BREQ              (1u << 11)
#define STM32_DMA_TR2_DREQ              (1u << 10)
#define STM32_DMA_TR2_SWREQ             (1u << 9)
#define STM32_DMA_TR2_REQSEL_MASK       0xffu

/* Block and linked-list fields. LLI addresses are 32-bit aligned and each
 * linked list, including every complete LLI, must fit in the 64-Kbyte region
 * selected by LBAR (RM0486 sections 18.4.5 and 19.4.5).
 */

#define STM32_DMA_BR1_BNDT_MASK        0xffffu
#define STM32_DMA_SR_FIFOL_SHIFT       16
#define STM32_GPDMA_SR_FIFOL_MASK      (0xffu << STM32_DMA_SR_FIFOL_SHIFT)
#define STM32_HPDMA_SR_FIFOL_MASK      (0x1ffu << STM32_DMA_SR_FIFOL_SHIFT)
#define STM32_DMA_BR1_BRC_SHIFT        16
#define STM32_DMA_BR1_BRC_MASK         (0x7ffu << STM32_DMA_BR1_BRC_SHIFT)
#define STM32_DMA_BR1_SDEC             (1u << 28)
#define STM32_DMA_BR1_DDEC             (1u << 29)
#define STM32_DMA_BR1_BRSDEC           (1u << 30)
#define STM32_DMA_BR1_BRDDEC           (1u << 31)
#define STM32_DMA_TR3_SAO_MASK         0x1fffu
#define STM32_DMA_TR3_DAO_SHIFT        16
#define STM32_DMA_TR3_DAO_MASK         (0x1fffu << STM32_DMA_TR3_DAO_SHIFT)
#define STM32_DMA_BR2_BRSAO_MASK       0xffffu
#define STM32_DMA_BR2_BRDAO_SHIFT      16
#define STM32_DMA_BR2_BRDAO_MASK       (0xffffu << STM32_DMA_BR2_BRDAO_SHIFT)
#define STM32_DMA_BLOCK_MAX            0xffffu
#define STM32_DMA_LBAR_MASK            0xffff0000u
#define STM32_DMA_LBAR_SHIFT           16
#define STM32_DMA_LLR_UT1              (1u << 31)
#define STM32_DMA_LLR_UT2              (1u << 30)
#define STM32_DMA_LLR_UB1              (1u << 29)
#define STM32_DMA_LLR_USA              (1u << 28)
#define STM32_DMA_LLR_UDA              (1u << 27)
/* UT3 and UB2 are valid only on channels 12-15. */
#define STM32_DMA_LLR_UT3              (1u << 26)
#define STM32_DMA_LLR_UB2              (1u << 25)
#define STM32_DMA_LLR_ULL              (1u << 16)
#define STM32_DMA_LLR_LA_MASK          0x0000fffcu
#define STM32_DMA_LLR_ALIGN            4
#define STM32_DMA_LLR_UPDATE_MASK      (STM32_DMA_LLR_UT1 | \
                                        STM32_DMA_LLR_UT2 | \
                                        STM32_DMA_LLR_UB1 | \
                                        STM32_DMA_LLR_USA | \
                                        STM32_DMA_LLR_UDA | \
                                        STM32_DMA_LLR_ULL)
#define STM32_DMA_LLR_UPDATE_MASK_2D   (STM32_DMA_LLR_UPDATE_MASK | \
                                        STM32_DMA_LLR_UT3 | \
                                        STM32_DMA_LLR_UB2)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMA_H */
