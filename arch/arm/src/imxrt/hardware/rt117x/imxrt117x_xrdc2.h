/*****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_xrdc2.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_XRDC2_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_XRDC2_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define IMXRT_XRDC2_D0_BASE           0x40ce0000

#define IMXRT_XRDC2_NMDAC             18
#define IMXRT_XRDC2_MDAC_RESERVED     13
#define IMXRT_XRDC2_MDAC_SSARC        14
#define IMXRT_XRDC2_NMRC              8
#define IMXRT_XRDC2_MRC_SEMC          7

#define IMXRT_XRDC2_MCR_OFFSET        0x0000
#define IMXRT_XRDC2_MDA_OFFSET(i, j, k) \
  (0x2000 + 0x100 * (i) + 8 * (j) + 4 * (k))
#define IMXRT_XRDC2_MRGD_OFFSET(i, j, k) \
  (0x8000 + 0x400 * (i) + 0x20 * (j) + 4 * (k))

#define XRDC2_MCR_GVLDM               (1 << 0)
#define XRDC2_MCR_GVLDC               (1 << 1)
#define XRDC2_MCR_GCL_MASK            (3 << 4)
#define XRDC2_MCR_GCL_LOCKED          (3 << 4)

#define XRDC2_MDA_W0_MASK_ALL         0x0000ffff
#define XRDC2_MDA_W1_DID(n)           ((uint32_t)(n) << 16)
#define XRDC2_MDA_W1_DL               (1 << 30)
#define XRDC2_MDA_W1_VLD              (1u << 31)

#define XRDC2_MRGD_W6_DL2_LOCKED      (3 << 29)
#define XRDC2_MRGD_W6_VLD             (1u << 31)

#define XRDC2_ACP_ALL                 7
#define XRDC2_DXACP(d, acp)           ((uint32_t)(acp) << (3 * ((d) & 7)))
#define XRDC2_DXACP_ALL               0x00ffffff

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_XRDC2_H */
