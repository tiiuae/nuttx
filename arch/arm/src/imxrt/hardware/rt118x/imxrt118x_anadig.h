/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_anadig.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ANADIG_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ANADIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ANADIG PLL registers (IMXRT1180RM Ch. 41).  Offsets are relative to
 * IMXRT_ANADIG_PLL_BASE (0x4448_0000).  The M7 core clock is fed from
 * ARM_PLL; the CM33 side and most peripherals are fed from SYS_PLL2/3.
 */

#define IMXRT_ANADIG_PLL_ARM_CTRL        (IMXRT_ANADIG_PLL_BASE + 0x4000)
#define IMXRT_ANADIG_PLL_SYS3_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4010)
#define IMXRT_ANADIG_PLL_SYS3_UPDATE     (IMXRT_ANADIG_PLL_BASE + 0x4020)
#define IMXRT_ANADIG_PLL_SYS3_PFD        (IMXRT_ANADIG_PLL_BASE + 0x4030)
#define IMXRT_ANADIG_PLL_SYS2_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4040)
#define IMXRT_ANADIG_PLL_SYS2_UPDATE     (IMXRT_ANADIG_PLL_BASE + 0x4050)
#define IMXRT_ANADIG_PLL_SYS2_SS         (IMXRT_ANADIG_PLL_BASE + 0x4060)
#define IMXRT_ANADIG_PLL_SYS2_PFD        (IMXRT_ANADIG_PLL_BASE + 0x4070)
#define IMXRT_ANADIG_PLL_SYS2_MFN        (IMXRT_ANADIG_PLL_BASE + 0x4080)
#define IMXRT_ANADIG_PLL_SYS2_MFI        (IMXRT_ANADIG_PLL_BASE + 0x4090)
#define IMXRT_ANADIG_PLL_SYS2_MFD        (IMXRT_ANADIG_PLL_BASE + 0x40a0)
#define IMXRT_ANADIG_PLL_SYS1_CTRL       (IMXRT_ANADIG_PLL_BASE + 0x4100)
#define IMXRT_ANADIG_PLL_AUDIO_CTRL      (IMXRT_ANADIG_PLL_BASE + 0x4200)
#define IMXRT_ANADIG_PLL_VIDEO_CTRL      (IMXRT_ANADIG_PLL_BASE + 0x4300)

/* ARM_PLL_CTRL bit fields */

#define ANADIG_PLL_ARM_DIV_SELECT_SHIFT  (0)
#define ANADIG_PLL_ARM_DIV_SELECT_MASK   (0xffu << ANADIG_PLL_ARM_DIV_SELECT_SHIFT)
#define ANADIG_PLL_ARM_DIV_SELECT(x)     (((x) << ANADIG_PLL_ARM_DIV_SELECT_SHIFT) & \
                                          ANADIG_PLL_ARM_DIV_SELECT_MASK)
#define ANADIG_PLL_ARM_HOLD_RING_OFF     (1u << 12)
#define ANADIG_PLL_ARM_POWERUP           (1u << 13)
#define ANADIG_PLL_ARM_ENABLE_CLK        (1u << 14)
#define ANADIG_PLL_ARM_POST_DIV_SEL_SHIFT (19)
#define ANADIG_PLL_ARM_POST_DIV_SEL_MASK (0x7u << ANADIG_PLL_ARM_POST_DIV_SEL_SHIFT)
#define ANADIG_PLL_ARM_STABLE            (1u << 29)
#define ANADIG_PLL_ARM_GATE              (1u << 30)

/* ANADIG OSC registers (IMXRT1180RM Ch. 41) */

#define IMXRT_ANADIG_OSC_RC24M_CTRL      (IMXRT_ANADIG_OSC_BASE + 0x3310)
#define IMXRT_ANADIG_OSC_24M_CTRL        (IMXRT_ANADIG_OSC_BASE + 0x3320)
#define IMXRT_ANADIG_OSC_400M_CTRL1      (IMXRT_ANADIG_OSC_BASE + 0x3350)

/* PHY_LDO registers (IMXRT1180RM Ch. 41.  Physical: 0x4448_4680).  Each
 * has RW/SET/CLR/TOG aliases.
 */

#define IMXRT_PHY_LDO_CTRL0_BASE         (IMXRT_ANADIG_LDO_BASE + 0x680)
#define IMXRT_PHY_LDO_CTRL0_RW           (IMXRT_PHY_LDO_CTRL0_BASE + 0x0)
#define IMXRT_PHY_LDO_CTRL0_SET          (IMXRT_PHY_LDO_CTRL0_BASE + 0x4)
#define IMXRT_PHY_LDO_CTRL0_CLR          (IMXRT_PHY_LDO_CTRL0_BASE + 0x8)
#define IMXRT_PHY_LDO_CTRL0_TOG          (IMXRT_PHY_LDO_CTRL0_BASE + 0xc)

/* PHY_LDO_CTRL0 bit fields */

#define PHY_LDO_CTRL0_LINREG_EN          (1u << 0)   /* Enable linear regulator */
#define PHY_LDO_CTRL0_LINREG_PWRUPLOAD_DIS (1u << 1)
#define PHY_LDO_CTRL0_LINREG_ILIMIT_EN   (1u << 2)   /* Enable current-limit protection */
#define PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_SHIFT (4)    /* Output voltage trim */
#define PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_MASK (0x1fu << PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_SHIFT)
#define PHY_LDO_CTRL0_LINREG_OUTPUT_TRG(x) (((x) << PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_SHIFT) & \
                                            PHY_LDO_CTRL0_LINREG_OUTPUT_TRG_MASK)
#define PHY_LDO_CTRL0_LINREG_PHY_ISO_B   (1u << 15)  /* Release PHY isolation */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ANADIG_H */
