/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_mreg.c
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
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stdint.h>

#include <nuttx/arch.h>

#include "mpfs_scratch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_mreg_hartid
 *
 * Description:
 *   Context aware way to query hart id
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

uint64_t mpfs_mreg_hartid(void)
{
#ifdef CONFIG_BUILD_KERNEL
  /* Kernel is in S-mode */

  uint64_t sscratch = mpfs_get_hart_sscratch();

  DEBUGASSERT(sscratch != 0);

  return mpfs_scratch_get_hartid(sscratch);
#else
  /* Kernel is in M-mode */

  return READ_CSR(mhartid);
#endif
}
