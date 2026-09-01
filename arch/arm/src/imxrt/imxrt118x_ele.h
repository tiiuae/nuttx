/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_ele.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT118X_ELE_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT118X_ELE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/rt118x/imxrt118x_ele.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Embedded ELE firmware AHAB container (see imxrt118x_ele_fw.c;
 * path from CONFIG_IMXRT_ELE_FW_PATH).  Used by imxrt118x_ele_load_fw().
 */

extern const uint8_t imxrt118x_ele_fw[];
extern const uint8_t imxrt118x_ele_fw_end[];

/****************************************************************************
 * Name: imxrt118x_ele_init
 *
 * Description:
 *   Bring the EdgeLock Enclave up:
 *
 *     1. Initialise the S3MUA message-unit control regs (TCR/RCR).
 *     2. Load the embedded ELE firmware (imxrt118x_ele_fw[]) via LOAD_FW.
 *        The RT118x ROM only ships a minimal stub — the other ELE
 *        commands need the full firmware running.
 *     3. Enable Access Permission Control.
 *     4. Take ownership of the AON, MEGA and WAKEUP TRDCs.
 *     5. If CONFIG_IMXRT_TRDC is set, widen every domain's access-control
 *        window on all three TRDCs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt118x_ele_init(void);

/****************************************************************************
 * Name: imxrt118x_ele_load_fw
 *
 * Description:
 *   Ask the ROM ELE stub to load the full EdgeLock Enclave firmware from
 *   the given address.  Required on RT118x before any of the other ELE
 *   commands work — the boot ROM only ships a minimal stub that
 *   understands LOAD_FW; ENABLE_APC / RELEASE_RDC etc. require the loaded
 *   firmware.
 *
 * Input Parameters:
 *   fw_addr - Address (in ELE-visible memory) where the ELE FW AHAB
 *             container is located.  For RT118x with our AHAB layout
 *             this is typically the FlexSPI XIP address of the container
 *             (flash origin + AHAB offset, e.g. 0x28001000).
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_load_fw(uint32_t fw_addr);

/****************************************************************************
 * Name: imxrt118x_ele_release_rdc
 *
 * Description:
 *   Ask ELE to hand ownership of one TRDC (rdc_id) to the given core.
 *   Data word is (rdc_id << 8) | core_id.
 *
 * Input Parameters:
 *   rdc_id  - TRDC identifier (ELE_TRDC_AON_ID / _WAKEUP_ID / _MEGA_ID)
 *   core_id - Requesting core (ELE_CORE_CM33_ID / ELE_CORE_CM7_ID)
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_release_rdc(uint32_t rdc_id, uint32_t core_id);

/****************************************************************************
 * Name: imxrt118x_ele_enable_apc
 *
 * Description:
 *   Ask ELE to enable Access Permission Control for the M7 (RT1180-
 *   specific; used by the CM33 boot stub to allow subsequent per-core
 *   TRDC releases).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated errno value otherwise.
 *
 ****************************************************************************/

int imxrt118x_ele_enable_apc(void);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT118X_ELE_H */
