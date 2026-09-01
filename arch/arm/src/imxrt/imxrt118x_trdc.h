/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_trdc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT118X_TRDC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT118X_TRDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/rt118x/imxrt118x_trdc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Wildcard values usable in imxrt118x_trdc_mbc_config_s entries so a single
 * table row can widen many blocks / domains at once.
 */

#define IMXRT_TRDC_BLK_ALL       0xffffu
#define IMXRT_TRDC_DOM_ALL       0xffu

/* Domain IDs (DEFAULT_DID) as documented in RM Tables 299/300/301
 * ("TRDC1/2/3 MDAC Configuration").  Use these names in tables and code so
 * the intent is obvious.
 */

#define IMXRT_TRDC1_DID_CM33         2
#define IMXRT_TRDC1_DID_EDMA3        4

#define IMXRT_TRDC2_DID_CM7          4
#define IMXRT_TRDC2_DID_DAP          9
#define IMXRT_TRDC2_DID_CS_ETR       8
#define IMXRT_TRDC2_DID_EDMA4        7
#define IMXRT_TRDC2_DID_NETC_ACE     10

#define IMXRT_TRDC3_DID_USDHC1       5
#define IMXRT_TRDC3_DID_USDHC2       6
#define IMXRT_TRDC3_DID_USB          11
#define IMXRT_TRDC3_DID_FLEXSPI_FLR  10

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One row = one (glbac_id -> access word) mapping inside one MBC/MRC.
 * glbac_val uses the standard RT1180 nibble encoding:
 *    [15:12] SP  [11:8] SU  [7:4] NP  [3:0] NU  (each nibble = X|W|R).
 * TRDC_GLBAC_FULL_ACCESS (0x7777) opens R/W/X in all four modes.
 */

struct imxrt118x_trdc_glbac_config_s
{
  uint8_t  mbc_mrc_id;    /* MBC or MRC instance number */
  uint8_t  glbac_id;      /* 0..7 */
  uint32_t glbac_val;     /* GLBAC[15:0] permission word */
};

/* One row = one block-config nibble.  Use IMXRT_TRDC_BLK_ALL to program
 * every block of the given (mbc_id, mem_id).  Use IMXRT_TRDC_DOM_ALL to
 * program every domain 0..IMXRT_MBC_DOM_MAX-1.
 *
 * When nonsecure=true the NSE bit is set for the nibble (secure AND
 * nonsecure transactions allowed against the referenced GLBAC).  When
 * nonsecure=false the NSE bit is cleared (secure-only).  MBACSEL points
 * at one of the 8 GLBAC slots widened elsewhere.
 */

struct imxrt118x_trdc_mbc_config_s
{
  uint8_t  mbc_id;        /* MBC instance number */
  uint8_t  mem_id;        /* 0..3 sub-memory */
  uint8_t  dom_id;        /* Domain ID or IMXRT_TRDC_DOM_ALL */
  uint16_t blk_id;        /* Block index or IMXRT_TRDC_BLK_ALL */
  uint8_t  glbac_id;      /* Which GLBAC slot (0..7) the block references */
  bool     nonsecure;     /* true = NSE=1 (allow NS), false = NSE=0 (S-only) */
};

/* One row = one MRC region descriptor (start/size in the same domain).
 * secure=true means only secure transactions match; secure=false means
 * both secure and nonsecure match.
 */

struct imxrt118x_trdc_mrc_config_s
{
  uint8_t  mrc_id;
  uint8_t  dom_id;
  uint8_t  region_id;
  uint8_t  glbac_id;
  bool     secure;
  uint32_t region_start;
  uint32_t region_size;
};

/* Aggregate config for one TRDC instance.  Pass NULL / 0 to skip a stage.
 */

struct imxrt118x_trdc_config_s
{
  uintptr_t trdc_base;

  const struct imxrt118x_trdc_glbac_config_s *mbc_glbac;
  unsigned int n_mbc_glbac;

  const struct imxrt118x_trdc_mbc_config_s   *mbc_cfg;
  unsigned int n_mbc_cfg;

  const struct imxrt118x_trdc_glbac_config_s *mrc_glbac;
  unsigned int n_mrc_glbac;

  const struct imxrt118x_trdc_mrc_config_s   *mrc_cfg;
  unsigned int n_mrc_cfg;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_trdc_grant_full_access
 *
 * Description:
 *   Convenience wrapper: widen every MBC's GLBAC[0] and every MRC's
 *   GLBAC[0] to TRDC_GLBAC_FULL_ACCESS (0x7777), then walk every block of
 *   every sub-memory of every MBC and set the per-domain nibble to
 *   (NSE=1, MBACSEL=0) — i.e. secure and nonsecure privileged and user
 *   R/W/X for every master.  This is the RT1180 equivalent of the
 *   MCUXpresso SDK BOARD_GrantTRDCFullPermissions() applied to one TRDC.
 *
 *   Unlike the earlier implementation this also programs the block-config
 *   words, so non-CPU masters (eDMA3, eDMA4, uSDHC, USB, NETC…) that live
 *   in their own TRDC domains actually pick up the permissive settings.
 *
 *   Must be called by CM33 while it still owns the TRDC.
 *
 * Input Parameters:
 *   trdc_base - IMXRT_TRDC1_BASE / IMXRT_TRDC2_BASE / IMXRT_TRDC3_BASE.
 *
 ****************************************************************************/

void imxrt118x_trdc_grant_full_access(uintptr_t trdc_base);

/****************************************************************************
 * Name: imxrt118x_trdc_apply
 *
 * Description:
 *   Apply a table-driven TRDC configuration.  Both GLBAC widening and
 *   per-block/per-region assignments are honoured.  Rows with wildcard
 *   dom_id / blk_id are expanded.  Use this instead of the "grant full"
 *   helper when a board needs to add new blocks or lock some down later.
 *
 ****************************************************************************/

void imxrt118x_trdc_apply(const struct imxrt118x_trdc_config_s *cfg);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT118X_TRDC_H */
