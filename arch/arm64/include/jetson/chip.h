/****************************************************************************
 * arch/arm64/include/jetson/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_JETSON_CHIP_H__
#define __ARCH_ARM64_INCLUDE_JETSON_CHIP_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of bytes in @p x kibibytes/mebibytes/gibibytes */

#define KB(x)   ((x) << 10)
#define MB(x)   (KB(x) << 10)
#define GB(x)   (MB(UINT64_C(x)) << 10)

/* TODO !! All these addresses are nonsense */

#if CONFIG_ARM_GIC_VERSION == 3 || CONFIG_ARM_GIC_VERSION == 4

#define CONFIG_GICD_BASE          0x51a00000
#define CONFIG_GICR_BASE          0x51b00000
#define CONFIG_GICR_OFFSET        0x20000

#else

#error CONFIG_ARM_GIC_VERSION should be 3 or 4

#endif /* CONFIG_ARM_GIC_VERSION */

#define CONFIG_RAMBANK1_ADDR      0x80000000
#define CONFIG_DEVICEIO_BASEADDR  0x40000000

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xff)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

.macro  get_cpu_id xreg0
  mrs    \xreg0, mpidr_el1
  ubfx   \xreg0, \xreg0, #0, #8
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_JETSON_CHIP_H */
