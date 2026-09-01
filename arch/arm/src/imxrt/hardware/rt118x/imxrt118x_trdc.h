/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_trdc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_TRDC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_TRDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* i.MX RT1180 has three Trusted Resource Domain Controller instances
 * (IMXRT1180RM Ch. 4).  Base addresses live in
 * hardware/rt118x/imxrt118x_memorymap.h (IMXRT_TRDC1_BASE, IMXRT_TRDC2_BASE,
 * IMXRT_TRDC3_BASE).
 *
 *   TRDC1 -> AONMIX  (Always-On)
 *   TRDC2 -> WAKEUPMIX / MEGAMIX (main peripheral domain, incl. FlexSPI)
 *   TRDC3 -> NETC / secondary peripheral domain
 */

/* Domain identifiers (DID) used in RDC access control words.  These match
 * the ELE core IDs used with ELE_RELEASE_RDC_REQ:
 *   0 = reserved / boot ROM
 *   1 = Cortex-M33
 *   2 = Cortex-M7
 *   3.. = other bus masters (DMA, GPU, etc.)
 */

#define TRDC_DID_CM33                   1
#define TRDC_DID_CM7                    2

/* MBC / MRC control word layout.
 * Nibbles hold R/W/X permissions per (secure, non-secure) x (priv, user):
 *   [15:12] SP  secure-privileged
 *   [11:8]  SU  secure-user
 *   [7:4]   NP  non-secure-privileged
 *   [3:0]   NU  non-secure-user
 * Value 7 = R|W|X (full access).
 */

#define TRDC_GLBAC_FULL_ACCESS          0x00007777u

/* Detect valid MBC/MRC by inspecting the TRDC's HWCFG0 register:
 * bits [19:16] MBC number, bits [28:24] MRC number.
 */

#define IMXRT_TRDC_HWCFG0_OFFSET        0xf0

#define IMXRT_MBC0_MEM_GLBAC(n)         (0x20 + ((n) << 2))
#define IMXRT_MBC_MEM_BLK_CFG_0(m, n)   (0x200 * (m) + 0x40 + ((n) << 2))
#define IMXRT_MRC0_DOM_RGD_W(m, n)      (0x100 * (m) + 0x40 + ((n) << 3))

/* MBC memory-block config-word offsets (per-domain, relative to the MBC
 * instance base).  A TRDC's MBC instance covers up to 4 sub-memories:
 *
 *   MEM0 CFG_W: 16 words at 0x40 (base + 0x40..0x7F)
 *   MEM0 NSE_W: 4 words at 0x140
 *   MEM1 CFG_W: 4 words at 0x180
 *   MEM1 NSE_W: 1 word  at 0x1A0
 *   MEM2 CFG_W: 1 word  at 0x1A8
 *   MEM2 NSE_W: 1 word  at 0x1C8
 *   MEM3 CFG_W: 3 words at 0x1D0
 *   MEM3 NSE_W: 1 word  at 0x1F0
 *
 * Each domain occupies 0x200 bytes so domain n starts at 0x40 + n * 0x200
 * for the CFG_W layout above.  Writing 0 to every CFG_W nibble means the
 * corresponding block uses GLBAC[0] (which we set to full R/W/X).
 * Writing 0 to NSE_W means "secure only", which combined with GLBAC[0]
 * providing full secure and non-secure access is still permissive.
 */

#define IMXRT_MBC_DOM_STRIDE            0x200
#define IMXRT_MBC_DOM_MAX               16
#define IMXRT_MBC_INST_STRIDE           0x2000

#define IMXRT_MBC_MEM0_CFG_W_OFF        0x00
#define IMXRT_MBC_MEM0_CFG_W_COUNT      16
#define IMXRT_MBC_MEM0_NSE_W_OFF        0x100
#define IMXRT_MBC_MEM0_NSE_W_COUNT      4
#define IMXRT_MBC_MEM1_CFG_W_OFF        0x140
#define IMXRT_MBC_MEM1_CFG_W_COUNT      4
#define IMXRT_MBC_MEM1_NSE_W_OFF        0x160
#define IMXRT_MBC_MEM1_NSE_W_COUNT      1
#define IMXRT_MBC_MEM2_CFG_W_OFF        0x168
#define IMXRT_MBC_MEM2_CFG_W_COUNT      1
#define IMXRT_MBC_MEM2_NSE_W_OFF        0x188
#define IMXRT_MBC_MEM2_NSE_W_COUNT      1
#define IMXRT_MBC_MEM3_CFG_W_OFF        0x190
#define IMXRT_MBC_MEM3_CFG_W_COUNT      3
#define IMXRT_MBC_MEM3_NSE_W_OFF        0x1b0
#define IMXRT_MBC_MEM3_NSE_W_COUNT      1

/* MRC region-descriptor words: two words per region, per domain.
 * Domain stride is 0x100, and each domain has up to 16 region descriptors
 * (2 words each) starting at 0x40.  Writing 0 to every region-descriptor
 * word disables the region (invalid), which is what we want when we
 * intend the block-config words to be the only authority.
 */

#define IMXRT_MRC_DOM_STRIDE            0x100
#define IMXRT_MRC_DOM_MAX               16
#define IMXRT_MRC_INST_STRIDE           0x1000
#define IMXRT_MRC_REGION_W_OFF          0x00
#define IMXRT_MRC_REGION_W_COUNT        32   /* 16 regions x 2 words each */

#define MBC_NUM(HWCFG)                  (((HWCFG) >> 16) & 0xf)
#define MRC_NUM(HWCFG)                  (((HWCFG) >> 24) & 0x1f)
#define MBC_BLK_NUM(GLBCFG)             ((GLBCFG) & 0x3ff)

#define GLBAC_SETTING_MASK              0x7777u
#define GLBAC_LOCK_MASK                 (1u << 31)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_TRDC_H */
