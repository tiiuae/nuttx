/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx93/imx93_gpio.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_MUA_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_MUA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx93_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_MUA_VER_OFFSET          (0x0000) /* Version ID */
#define IMX9_MUA_PAR_OFFSET          (0x0004) /* Parameter */
#define IMX9_MUA_CR_OFFSET           (0x0008) /* Control */
#define IMX9_MUA_SR_OFFSET           (0x000C) /* Status */
#define IMX9_MUA_CCR0_OFFSET         (0x0010) /* Core Control 0 */
#define IMX9_MUA_CIER0_OFFSET        (0x0014) /* Core interrupt Enable 0 */
#define IMX9_MUA_CSSR0_OFFSET        (0x0018) /* Core Sticky Status 0 */
#define IMX9_MUA_FCR_OFFSET          (0x0100) /* Flag Control */
#define IMX9_MUA_FSR_OFFSET          (0x0104) /* Flag Status */
#define IMX9_MUA_GIER_OFFSET         (0x0110) /* General-purpose Interrupt Enable */
#define IMX9_MUA_GCR_OFFSET          (0x0114) /* General-Purpose Control */
#define IMX9_MUA_GSR_OFFSET          (0x0118) /* General-Purpose Status */
#define IMX9_MUA_TCR_OFFSET          (0x0120) /* Transmit Control */
#define IMX9_MUA_TSR_OFFSET          (0x0124) /* Transmit Status */
#define IMX9_MUA_RCR_OFFSET          (0x0128) /* Receive Control */
#define IMX9_MUA_RSR_OFFSET          (0x012C) /* Receive Status */
#define IMX9_MUA_TR_OFFSET           (0x0200) /* Transmit */
#define IMX9_MUA_RR_OFFSET           (0x0280) /* Receive */

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_MUA_H */
