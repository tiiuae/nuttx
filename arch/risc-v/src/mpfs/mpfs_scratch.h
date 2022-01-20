/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_scratch.h
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

#ifndef __ARCH_RISC_V_SRC_MPFS_MPFS_SCRATCH_H_
#define __ARCH_RISC_V_SRC_MPFS_MPFS_SCRATCH_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/arch.h>

/****************************************************************************
 * Name: mpfs_scratch_init
 *
 * Description:
 *   Initialize the scratch area structures, should only be done on the boot
 *   hart. Other harts should not access the scratch area before it is
 *   initialized.
 *
 ****************************************************************************/

void mpfs_scratch_init(void);

/****************************************************************************
 * Name: mpfs_scratch_get
 *
 * Description:
 *   Get pointer to scratch area for a specific hart. Use this during system
 *   initialization to obtain pointer to hart specific scratch area. This
 *   pointer can then be stored into the hart's context specific scracth
 *   register.
 *
 * Input Parameters:
 *   hartid - Hart number
 *
 * Returned Value:
 *   Pointer to scratch area. Store this to the m/sscratch register.
 *
 ****************************************************************************/

uintptr_t mpfs_scratch_get_addr(uint64_t hartid);

/****************************************************************************
 * Name: mpfs_scratch_get_hartid
 *
 * Description:
 *   Get harts own hartid by reading it from the scratch area. This is safe
 *   to use from lower privilege modes (than M-mode).
 *
 * Input Parameters:
 *   scratch - Pointer to scratch area
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

uint64_t mpfs_scratch_get_hartid(uintptr_t scratch);

/****************************************************************************
 * Name: mpfs_set_hart_sscratch
 *
 * Description:
 *   Utility function to set a harts own sscratch register with the scratch
 *   memory area base
 *
 * Input Parameters:
 *   scratch_base - Scratch area base address
 *
 ****************************************************************************/

static inline void mpfs_set_hart_sscratch(uint64_t scratch_base)
{
  WRITE_CSR(sscratch, scratch_base);
}

/****************************************************************************
 * Name: mpfs_get_hart_sscratch
 *
 * Description:
 *   Read a harts own sscratch register
 *
 * Returned Value:
 *   sscratch register value
 *
 ****************************************************************************/

static inline uint64_t mpfs_get_hart_sscratch(void)
{
  return READ_CSR(sscratch);
}

#endif /* __ARCH_RISC_V_SRC_MPFS_MPFS_SCRATCH_H_ */
