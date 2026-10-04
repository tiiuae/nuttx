/*****************************************************************************
 * arch/arm/src/imxrt/imxrt_xrdc2.c
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

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <arch/barriers.h>

#include "arm_internal.h"
#include "imxrt_xrdc2.h"

#ifdef CONFIG_IMXRT_XRDC2

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

static void imxrt_xrdc2_mda(uintptr_t base, int i, int did)
{
  if (i != IMXRT_XRDC2_MDAC_SSARC)
    {
      putreg32(XRDC2_MDA_W0_MASK_ALL, base + IMXRT_XRDC2_MDA_OFFSET(i, 0, 0));
    }

  putreg32(XRDC2_MDA_W1_VLD | XRDC2_MDA_W1_DL | XRDC2_MDA_W1_DID(did),
           base + IMXRT_XRDC2_MDA_OFFSET(i, 0, 1));
}

static void imxrt_xrdc2_region(uintptr_t base, int mrc, int j,
                               uint32_t first, uint32_t last,
                               uint32_t lo, uint32_t hi)
{
  putreg32(first, base + IMXRT_XRDC2_MRGD_OFFSET(mrc, j, 0));
  putreg32(0, base + IMXRT_XRDC2_MRGD_OFFSET(mrc, j, 1));
  putreg32(last, base + IMXRT_XRDC2_MRGD_OFFSET(mrc, j, 2));
  putreg32(0, base + IMXRT_XRDC2_MRGD_OFFSET(mrc, j, 3));
  putreg32(lo, base + IMXRT_XRDC2_MRGD_OFFSET(mrc, j, 5));
  putreg32(hi | XRDC2_MRGD_W6_DL2_LOCKED | XRDC2_MRGD_W6_VLD,
           base + IMXRT_XRDC2_MRGD_OFFSET(mrc, j, 6));
}

static void imxrt_xrdc2_setup(uintptr_t base, uint32_t first,
                              uint32_t last)
{
  int i;

  for (i = 0; i < IMXRT_XRDC2_NMDAC; i++)
    {
      if (i != IMXRT_XRDC2_MDAC_RESERVED)
        {
          imxrt_xrdc2_mda(base, i, i < 2 ? 0 : 1);
        }
    }

  for (i = 0; i < IMXRT_XRDC2_NMRC; i++)
    {
#ifndef CONFIG_IMXRT_SEMC
      if (i == IMXRT_XRDC2_MRC_SEMC)
        {
          continue;
        }
#endif

      imxrt_xrdc2_region(base, i, 0, 0, 0xffffffff,
                         XRDC2_DXACP_ALL, XRDC2_DXACP_ALL);
      imxrt_xrdc2_region(base, i, 1, first, last,
                         XRDC2_DXACP(0, XRDC2_ACP_ALL), 0);
    }
}

static void imxrt_xrdc2_enable(uintptr_t base)
{
  putreg32(XRDC2_MCR_GVLDM, base + IMXRT_XRDC2_MCR_OFFSET);
  getreg32(base + IMXRT_XRDC2_MCR_OFFSET);
  getreg32(base + IMXRT_XRDC2_MCR_OFFSET);
  UP_DSB();

  putreg32(XRDC2_MCR_GVLDM | XRDC2_MCR_GVLDC | XRDC2_MCR_GCL_LOCKED,
           base + IMXRT_XRDC2_MCR_OFFSET);
  UP_DSB();
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int imxrt_xrdc2_fence(uintptr_t start, uintptr_t end)
{
  if (getreg32(IMXRT_XRDC2_D0_BASE + IMXRT_XRDC2_MCR_OFFSET) &
      XRDC2_MCR_GCL_MASK)
    {
      return -EBUSY;
    }

  imxrt_xrdc2_setup(IMXRT_XRDC2_D0_BASE, start, end - 1);
  imxrt_xrdc2_enable(IMXRT_XRDC2_D0_BASE);
  return 0;
}

#endif /* CONFIG_IMXRT_XRDC2 */
