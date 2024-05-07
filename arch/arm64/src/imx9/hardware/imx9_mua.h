/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_MUA_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_MUA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_IMX93)
#  include "hardware/imx93/imx93_mua.h"
#else
#  error Unrecognized i.MX9 architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register bit definitions *************************************************/

/* Version ID Register (VERID)  */

#define MUA_VERID_FEATURE_SHIFT      (0)       /* Bits 0-15: Module Identification Number (FEATURE) */
#define MUA_VERID_FEATURE_MASK       (0xffff << MUA_VERID_FEATURE_SHIFT)
#define MUA_VERID_MINOR_SHIFT        (16)      /* Bits 16-23: Minor Version Number (MINOR) */
#define MUA_VERID_MINOR_MASK         (0xff << MUA_VERID_MINOR_SHIFT)
#define MUA_VERID_MAJOR_SHIFT        (24)      /* Bits 24-31: Major Version Number (MAJOR) */
#define MUA_VERID_MAJOR_MASK         (0xff << MUA_VERID_MAJOR_SHIFT)

/* Parameter Register (PARAM) */

#define MUA_PARAM_TR_NUM_SHIFT       (0)       /* Bits 0-7: Transmit Register Number */
#define MUA_PARAM_TR_NUM_MASK        (0xff << MUA_PARAM_TR_NUM_SHIFT)
#define MUA_PARAM_RR_NUM_SHIFT       (8)       /* Bits 8-15: Receive Register Number */
#define MUA_PARAM_RR_NUM_MASK        (0xff << MUA_PARAM_RR_NUM_SHIFT)
#define MUA_PARAM_GIR_NUM_SHIFT      (16)      /* Bits 16-23: General-Purpose Interrupt Request Number */
#define MUA_PARAM_GIR_NUM_MASK       (0xff << MUA_PARAM_GIR_NUM_SHIFT)
#define MUA_PARAM_FLAG_WIDTH_SHIFT   (24)      /* Bits 24-31: Specifies the number of flag bits (3) in the Flag Control (FCR) and Flag Status (FSR) registers.*/
#define MUA_PARAM_FLAG_WIDTH_MASK    (0xff << MUA_PARAM_FLAG_WIDTH_SHIFT)

/* Control Register (CR) */

#define MUA_CR_MUR                   (1 << 0)  /* Bit 0: MU reset */
#define MUA_CR_MURIE                 (1 << 1)  /* Bit 1: MUA Reset Interrupt Enable */
                                               /* Bits 2-31: Reserved */
/* Status Register (SR) */

#define MUA_SR_MURS                   (1 << 0)  /* Bit 0: MUA and MUB Reset State */
#define MUA_SR_MURIP                  (1 << 1)  /* Bit 1: MU Reset Interrupt Pending Flag */
#define MUA_SR_EP                     (1 << 2)  /* Bit 2: MUA Side Event Pending */
#define MUA_SR_FUP                    (1 << 3)  /* Bit 3: MUA Flag Update Pending */
#define MUA_SR_GIRP                   (1 << 4)  /* Bit 4: MUA General-Purpose Interrupt Pending */
#define MUA_SR_TEP                    (1 << 5)  /* Bit 5: MUA Transmit Empty Pending */
#define MUA_SR_RFP                    (1 << 6)  /* Bit 6: MUA Receive Full Pending */
                                                /* Bits 7-31: Reserved */

/* Core Control Register 0 */
#define MUA_CCR0_NMI                   (1 << 0)  /* Bit 0:MUB Nonmaskable Interrupt Request */

/* Core Sticky Status 0 (CSSR0) */
#define MUA_CSSR0_NMIC                  (1 << 0)  /* Bit 0:Processor A Nonmaskable Interrupt Clears */

/* Flag Control Register */
#define MUA_FCR_FLAG(x)                 (1 << (x & 3)) /* Bit 0..2 MUA to MUB Flag */

/* Flag Status Register */
#define MUA_FSR_FLAG(x)                 (1 << (x & 3)) /* Bit 0..2 MUB to MUA Flag */

/* General-Purpose Interrupt Enable (GIER) */

/* General-Purpose Control (GCR) */

/* General-purpose Status (GSR) */

/* Transmit Control (TCR) */

/* Transmit Status (TSR) */

/* Receive Control (RCR) */

/* Receive Status (RSR) */

/* Transmit (TR0 - TR7) */

/* Receive (RR0 - RR3) */








#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_MUA_H */
