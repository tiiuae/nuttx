/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_trdc.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "arm_internal.h"
#include "imxrt118x_trdc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-MBC layout inside a TRDC (per RM Ch. 43).  A TRDC contains up to
 * MBC_NUM(HWCFG0) memory-block controllers, each occupying
 * IMXRT_MBC_INST_STRIDE (0x2000) bytes starting at trdc_base + 0x10000.
 * Each MBC is divided into IMXRT_MBC_DOM_MAX (16) domain windows of
 * IMXRT_MBC_DOM_STRIDE (0x200) bytes.  Inside every domain window:
 *
 *   0x00..0x0F  MEM_GLBCFG[0..3]           (only meaningful in domain 0)
 *   0x20..0x3F  MEMn_GLBAC[0..7]           (only meaningful in domain 0)
 *   0x40..0x?   Sub-memory block-config words for that domain
 *   0x140..     Sub-memory block-NSE words for that domain
 *   ...
 *
 * Every block is described in the CFG_W words by a 4-bit nibble:
 *
 *   bit 3: NSE     (NonSecure Enable) — set to allow NS transactions
 *   bit 2..0: MBACSEL (points at one of the 8 GLBAC entries)
 *
 * Each 32-bit CFG_W word holds 8 block nibbles.
 */

/* Small helpers for MBC / MRC base addresses. */

#define TRDC_MBC_BASE(trdc, mbc) \
  ((trdc) + 0x10000 + (uintptr_t)(mbc) * IMXRT_MBC_INST_STRIDE)

#define TRDC_MRC_BASE(trdc, mbc_num, mrc) \
  ((trdc) + 0x10000 + (uintptr_t)(mbc_num) * IMXRT_MBC_INST_STRIDE + \
   (uintptr_t)(mrc) * IMXRT_MRC_INST_STRIDE)

#define TRDC_MBC_DOM(mbc, dom) \
  ((mbc) + (uintptr_t)(dom) * IMXRT_MBC_DOM_STRIDE)

#define TRDC_MRC_DOM(mrc, dom) \
  ((mrc) + (uintptr_t)(dom) * IMXRT_MRC_DOM_STRIDE)

/* MEM_GLBCFG[mem] is at offset 0x00..0x0C inside domain 0 of the MBC.
 * Bits [9:0] hold the block count for that sub-memory.
 */

#define TRDC_MBC_MEM_GLBCFG_OFF(mem)  ((uintptr_t)(mem) * 4u)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Per-sub-memory offsets (bytes) of CFG_W arrays inside one domain window.
 * The layout matches the imx9 struct mbc_mem_dom used by the MCUXpresso
 * SDK / i.MX9 NuttX TRDC driver.
 *
 * MEM0 has up to 512 blocks (64 CFG_W words), MEM1..MEM3 up to 64 (8 words).
 */

struct trdc_mem_layout_s
{
  uint16_t cfg_w_off;      /* Byte offset into the domain window */
  uint16_t cfg_w_words;    /* Max number of 32-bit CFG_W words */
};

static const struct trdc_mem_layout_s g_mem_layout[4] =
{
  { 0x040, 64 },   /* MEM0: 64 CFG_W words * 8 blocks = 512 blocks max */
  { 0x180,  8 },   /* MEM1:  8 CFG_W words * 8 blocks =  64 blocks max */
  { 0x1a8,  8 },   /* MEM2:  8 CFG_W words * 8 blocks =  64 blocks max */
  { 0x1d0,  8 },   /* MEM3:  8 CFG_W words * 8 blocks =  64 blocks max */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trdc_mbc_enabled / trdc_mrc_enabled
 *
 * Description:
 *   Query the TRDC top-level enable bits in the CR (HWCFG0 in some
 *   docs) register at offset 0.
 *
 ****************************************************************************/

static inline bool trdc_mbc_enabled(uintptr_t trdc_base)
{
  return !!(getreg32(trdc_base) & 0x4000u);
}

static inline bool trdc_mrc_enabled(uintptr_t trdc_base)
{
  return !!(getreg32(trdc_base) & 0x8000u);
}

/****************************************************************************
 * Name: trdc_mbc_num / trdc_mrc_num
 *
 * Description:
 *   Return the number of MBC / MRC instances present in the given TRDC.
 *
 ****************************************************************************/

static inline uint32_t trdc_mbc_num(uintptr_t trdc_base)
{
  return MBC_NUM(getreg32(trdc_base + IMXRT_TRDC_HWCFG0_OFFSET));
}

static inline uint32_t trdc_mrc_num(uintptr_t trdc_base)
{
  return MRC_NUM(getreg32(trdc_base + IMXRT_TRDC_HWCFG0_OFFSET));
}

/****************************************************************************
 * Name: trdc_mbc_blk_count
 *
 * Description:
 *   Return the block count of a given MBC sub-memory.  Reads MEM_GLBCFG[mem]
 *   from the MBC's domain-0 window, where the block count lives in the low
 *   10 bits.
 *
 ****************************************************************************/

static uint32_t trdc_mbc_blk_count(uintptr_t trdc_base, uint32_t mbc_id,
                                  uint32_t mem_id)
{
  uintptr_t mbc_base;
  uintptr_t dom0;

  if (mbc_id >= trdc_mbc_num(trdc_base) || mem_id > 3)
    {
      return 0;
    }

  mbc_base = TRDC_MBC_BASE(trdc_base, mbc_id);
  dom0     = TRDC_MBC_DOM(mbc_base, 0);
  return MBC_BLK_NUM(getreg32(dom0 + TRDC_MBC_MEM_GLBCFG_OFF(mem_id)));
}

/****************************************************************************
 * Name: trdc_mbc_set_glbac
 *
 * Description:
 *   Program one MEMn_GLBAC entry of an MBC.  Only the domain-0 window
 *   contains meaningful GLBAC state; the same GLBAC array is used by every
 *   domain.
 *
 ****************************************************************************/

static void trdc_mbc_set_glbac(uintptr_t trdc_base, uint32_t mbc_id,
                              uint32_t glbac_id, uint32_t glbac_val)
{
  uintptr_t mbc_base;

  if (mbc_id >= trdc_mbc_num(trdc_base) || glbac_id >= 8)
    {
      return;
    }

  mbc_base = TRDC_MBC_BASE(trdc_base, mbc_id);
  putreg32(glbac_val & (GLBAC_SETTING_MASK | GLBAC_LOCK_MASK),
           mbc_base + IMXRT_MBC0_MEM_GLBAC(glbac_id));
}

/****************************************************************************
 * Name: trdc_mrc_set_glbac
 *
 * Description:
 *   Program one MEMn_GLBAC entry of an MRC.  Only domain 0 holds it.
 *
 ****************************************************************************/

static void trdc_mrc_set_glbac(uintptr_t trdc_base, uint32_t mrc_id,
                              uint32_t glbac_id, uint32_t glbac_val)
{
  uintptr_t mrc_base;

  if (mrc_id >= trdc_mrc_num(trdc_base) || glbac_id >= 8)
    {
      return;
    }

  mrc_base = TRDC_MRC_BASE(trdc_base, trdc_mbc_num(trdc_base), mrc_id);
  putreg32(glbac_val & (GLBAC_SETTING_MASK | GLBAC_LOCK_MASK),
           mrc_base + IMXRT_MBC0_MEM_GLBAC(glbac_id));
}

/****************************************************************************
 * Name: trdc_mbc_blk_config
 *
 * Description:
 *   Program one block nibble in one (mbc, mem, dom) window's CFG_W array.
 *
 *   nse_bit = 1 sets the NSE bit (allow non-secure transactions).
 *   glbac_id points at one of the 8 GLBAC slots.
 *
 ****************************************************************************/

static void trdc_mbc_blk_config(uintptr_t trdc_base, uint32_t mbc_id,
                               uint32_t dom_id, uint32_t mem_id,
                               uint32_t blk_id, bool nonsecure,
                               uint32_t glbac_id)
{
  uintptr_t mbc_base;
  uintptr_t dom_base;
  uintptr_t cfg_w;
  uint32_t val;
  uint32_t nibble;
  uint32_t offset;

  if (mbc_id >= trdc_mbc_num(trdc_base) || mem_id > 3 ||
      dom_id >= IMXRT_MBC_DOM_MAX || glbac_id >= 8)
    {
      return;
    }

  if (blk_id >= g_mem_layout[mem_id].cfg_w_words * 8u)
    {
      return;
    }

  mbc_base = TRDC_MBC_BASE(trdc_base, mbc_id);
  dom_base = TRDC_MBC_DOM(mbc_base, dom_id);
  cfg_w    = dom_base + g_mem_layout[mem_id].cfg_w_off +
             (blk_id / 8u) * 4u;

  offset   = (blk_id % 8u) * 4u;
  nibble   = (glbac_id & 0x7u) | (nonsecure ? 0x8u : 0u);

  val = getreg32(cfg_w);
  val &= ~(0xfu << offset);
  val |= nibble << offset;
  putreg32(val, cfg_w);
}

/****************************************************************************
 * Name: trdc_mrc_rgn_config
 *
 * Description:
 *   Program one MRC region descriptor (two words) in one (mrc, dom) window.
 *
 ****************************************************************************/

static void trdc_mrc_rgn_config(uintptr_t trdc_base, uint32_t mrc_id,
                               uint32_t dom_id, uint32_t rgn_id,
                               uint32_t addr_start, uint32_t addr_size,
                               bool secure, uint32_t glbac_id)
{
  uintptr_t mrc_base;
  uintptr_t dom_base;
  uintptr_t desc_w;
  uint32_t addr_end;

  if (mrc_id >= trdc_mrc_num(trdc_base) ||
      dom_id >= IMXRT_MRC_DOM_MAX || rgn_id >= 16 || glbac_id >= 8)
    {
      return;
    }

  mrc_base = TRDC_MRC_BASE(trdc_base, trdc_mbc_num(trdc_base), mrc_id);
  dom_base = TRDC_MRC_DOM(mrc_base, dom_id);
  desc_w   = dom_base + 0x40u + (uintptr_t)rgn_id * 8u;

  addr_end = addr_start + addr_size - 1u;
  addr_start &= ~0x3fffu;
  addr_end   &= ~0x3fffu;

  putreg32(addr_start | (glbac_id & 0x7u), desc_w);
  putreg32(addr_end | 0x1u | (secure ? 0u : 0x10u), desc_w + 4u);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_trdc_apply
 *
 * Description:
 *   Apply a table-driven TRDC configuration to one TRDC.  Rows with
 *   wildcard blk_id / dom_id are expanded so board code stays compact.
 *
 ****************************************************************************/

void imxrt118x_trdc_apply(const struct imxrt118x_trdc_config_s *cfg)
{
  unsigned int i;
  unsigned int dom_lo;
  unsigned int dom_hi;
  unsigned int blk_lo;
  unsigned int blk_hi;
  unsigned int dom;
  unsigned int blk;
  uint32_t blk_count;

  if (cfg == NULL || cfg->trdc_base == 0)
    {
      return;
    }

  /* 1. Program every MRC GLBAC first, then every MBC GLBAC.  Order doesn't
   *    matter for correctness but doing GLBACs before CFG_W keeps the
   *    intermediate state legal (any CFG_W nibble already points at a
   *    slot we intend to widen).
   */

  if (trdc_mrc_enabled(cfg->trdc_base) && cfg->mrc_glbac != NULL)
    {
      for (i = 0; i < cfg->n_mrc_glbac; i++)
        {
          trdc_mrc_set_glbac(cfg->trdc_base, cfg->mrc_glbac[i].mbc_mrc_id,
                             cfg->mrc_glbac[i].glbac_id,
                             cfg->mrc_glbac[i].glbac_val);
        }
    }

  if (trdc_mbc_enabled(cfg->trdc_base) && cfg->mbc_glbac != NULL)
    {
      for (i = 0; i < cfg->n_mbc_glbac; i++)
        {
          trdc_mbc_set_glbac(cfg->trdc_base, cfg->mbc_glbac[i].mbc_mrc_id,
                             cfg->mbc_glbac[i].glbac_id,
                             cfg->mbc_glbac[i].glbac_val);
        }
    }

  /* 2. Program per-block permissions, expanding IMXRT_TRDC_DOM_ALL /
   *    IMXRT_TRDC_BLK_ALL wildcards.
   */

  if (trdc_mbc_enabled(cfg->trdc_base) && cfg->mbc_cfg != NULL)
    {
      for (i = 0; i < cfg->n_mbc_cfg; i++)
        {
          const struct imxrt118x_trdc_mbc_config_s *row = &cfg->mbc_cfg[i];

          if (row->dom_id == IMXRT_TRDC_DOM_ALL)
            {
              dom_lo = 0;
              dom_hi = IMXRT_MBC_DOM_MAX;
            }
          else
            {
              dom_lo = row->dom_id;
              dom_hi = row->dom_id + 1;
            }

          if (row->blk_id == IMXRT_TRDC_BLK_ALL)
            {
              blk_count = trdc_mbc_blk_count(cfg->trdc_base, row->mbc_id,
                                            row->mem_id);
              blk_lo = 0;
              blk_hi = blk_count;
            }
          else
            {
              blk_lo = row->blk_id;
              blk_hi = row->blk_id + 1;
            }

          for (dom = dom_lo; dom < dom_hi; dom++)
            {
              for (blk = blk_lo; blk < blk_hi; blk++)
                {
                  trdc_mbc_blk_config(cfg->trdc_base, row->mbc_id, dom,
                                      row->mem_id, blk, row->nonsecure,
                                      row->glbac_id);
                }
            }
        }
    }

  /* 3. Program MRC regions. */

  if (trdc_mrc_enabled(cfg->trdc_base) && cfg->mrc_cfg != NULL)
    {
      for (i = 0; i < cfg->n_mrc_cfg; i++)
        {
          const struct imxrt118x_trdc_mrc_config_s *row = &cfg->mrc_cfg[i];

          if (row->dom_id == IMXRT_TRDC_DOM_ALL)
            {
              dom_lo = 0;
              dom_hi = IMXRT_MRC_DOM_MAX;
            }
          else
            {
              dom_lo = row->dom_id;
              dom_hi = row->dom_id + 1;
            }

          for (dom = dom_lo; dom < dom_hi; dom++)
            {
              trdc_mrc_rgn_config(cfg->trdc_base, row->mrc_id, dom,
                                  row->region_id, row->region_start,
                                  row->region_size, row->secure,
                                  row->glbac_id);
            }
        }
    }
}

/****************************************************************************
 * Name: imxrt118x_trdc_grant_full_access
 *
 * Description:
 *   Widen every MBC + MRC GLBAC[0..7] entry on the given TRDC to R/W/X for
 *   every security combination.  Deliberately does NOT touch the per-block
 *   CFG_W nibbles: on RT1180 the CFG_W register file layout varies between
 *   MBCs (RM §43.8.1 shows MBC0 has W0..W15 for MEM0 while MBC1 has only
 *   W0..W3, and MEM1/2/3 differ similarly) and writes into the reserved
 *   gaps between valid CFG_W words can hard-fault the CM33 stub before it
 *   ever kicks off the CM7.
 *
 *   Board code that needs to change the per-block/per-domain permissions
 *   (for example to grant eDMA3 access to a specific AIPS1 peripheral)
 *   should build a table of imxrt118x_trdc_mbc_config_s rows and call
 *   imxrt118x_trdc_apply(), which programs only the exact
 *   (mbc, mem, dom, blk) tuples the board actually wants to touch.
 *
 *   Must be called by CM33 while it owns the TRDC (after
 *   ELE_RELEASE_RDC has handed ownership to core CM33).
 *
 ****************************************************************************/

void imxrt118x_trdc_grant_full_access(uintptr_t trdc_base)
{
  uint32_t mbc_num;
  uint32_t mrc_num;
  uint32_t i;
  uint32_t g;

  if (trdc_base == 0)
    {
      return;
    }

  mbc_num = trdc_mbc_num(trdc_base);
  mrc_num = trdc_mrc_num(trdc_base);

  if (trdc_mbc_enabled(trdc_base))
    {
      for (i = 0; i < mbc_num; i++)
        {
          for (g = 0; g < 8; g++)
            {
              trdc_mbc_set_glbac(trdc_base, i, g, TRDC_GLBAC_FULL_ACCESS);
            }
        }
    }

  if (trdc_mrc_enabled(trdc_base))
    {
      for (i = 0; i < mrc_num; i++)
        {
          for (g = 0; g < 8; g++)
            {
              trdc_mrc_set_glbac(trdc_base, i, g, TRDC_GLBAC_FULL_ACCESS);
            }
        }
    }
}
