/*****************************************************************************
 * arch/arm/src/imxrt/imxrt_xrdc2.h
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_XRDC2_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_XRDC2_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/rt117x/imxrt117x_xrdc2.h"

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*****************************************************************************
 * Name: imxrt_xrdc2_fence
 *
 * Description:
 *   Put the Cortex-M7 core in domain 0 and every other bus master in
 *   domain 1 on the M7-side XRDC2 manager, leave all checked memory open
 *   to every domain except [start, end), which only domain 0 may reach,
 *   then lock the configuration until the next reset.  start and end
 *   must be 4 KB aligned.
 *
 * Returned Value:
 *   Zero on success, -EBUSY if an earlier boot already locked XRDC2.
 *
 *****************************************************************************/

int imxrt_xrdc2_fence(uintptr_t start, uintptr_t end);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_XRDC2_H */
