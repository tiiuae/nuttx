/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_ele_fw.c
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

/****************************************************************************
 * Embeds the NXP EdgeLock Enclave firmware AHAB container (path from
 * CONFIG_IMXRT_ELE_FW_PATH) directly into the driver image so that
 * imxrt118x_ele_boot() can hand its address to the ELE via LOAD_FW at
 * boot time, without depending on where the container sits in the outer
 * AHAB flash layout.
 *
 * The container is proprietary NXP object code; see
 * tools/imxrt1180/fetch_ele_fw.sh for the license note.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imxrt118x_ele.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef IMXRT_ELE_FW_ABS_PATH
#  error "IMXRT_ELE_FW_ABS_PATH must be set via CFLAGS (see arch/arm/src/imxrt/Make.defs)"
#endif

#define STR2(m) #m
#define STR(m) STR2(m)

/****************************************************************************
 * Public Data
 ****************************************************************************/

__asm__ (
    ".section .rodata.imxrt118x_ele_fw, \"a\"\n"
    ".balign  8\n"
    ".globl   imxrt118x_ele_fw\n"
"imxrt118x_ele_fw:\n"
    ".incbin " STR(IMXRT_ELE_FW_ABS_PATH) "\n"
    ".balign  8\n"
    ".globl   imxrt118x_ele_fw_end\n"
"imxrt118x_ele_fw_end:\n"
);
