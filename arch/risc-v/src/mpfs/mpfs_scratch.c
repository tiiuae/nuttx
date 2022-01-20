/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_scratch.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HART_CNT    (5)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mpfs_scratch_s
{
  uint64_t hartid;
};

static struct mpfs_scratch_s g_scratch[HART_CNT];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_scratch_init
 *
 * Description:
 *   Initialize the scratch area structures, should only be done on the boot
 *   hart. Other harts should not access the scratch area before it is
 *   initialized.
 *
 ****************************************************************************/

void mpfs_scratch_init(void)
{
  int i;

  for (i = 0; i < HART_CNT; i++)
    {
      g_scratch[i].hartid = i;
    }
}

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

uintptr_t mpfs_scratch_get_addr(uint64_t hartid)
{
  /* Hart IDs go from 0...4 */

  if (hartid < HART_CNT)
    {
      return (uintptr_t) &g_scratch[hartid];
    }

  return 0;
}

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

uint64_t mpfs_scratch_get_hartid(uintptr_t scratch)
{
  DEBUGASSERT(scratch >= (uintptr_t) &g_scratch &&
              scratch <= (uintptr_t) &g_scratch + sizeof(g_scratch));

  return ((struct mpfs_scratch_s *)scratch)->hartid;
}

