/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_bootloader.c
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

/* Minimal Cortex-M33 bootloader application for the MIMXRT1180-EVK
 * ("bl" configuration).
 *
 * This is the NuttX-on-M33 equivalent of the legacy standalone M33 boot
 * stub (src/m33_stub/stub.c): it does nothing but release the Cortex-M7
 * so that the M7 NuttX image XIP'd at 0x2808_0000 starts running.  The
 * ELE (EdgeLock Enclave) has already been brought up by imxrt_start()
 * before this entry point runs, so the only remaining work is:
 *
 *   1. Bring the ARM PLL up and route it to the M7 core clock, otherwise
 *      the M7 has no clock and never fetches an instruction.
 *   2. Set BLK_CTRL_S_AONMIX.M7_CFG.INITVTOR to 0 (the M7 ITCM).
 *   3. Set SRC.SCR.BT_RELEASE_M7.
 *   4. Scrub the ECC-protected M7 TCM aliases and install the two-word
 *      kick-off vector at M7 ITCM address 0.
 *   5. Ask the ELE to release the M7 (Enable APC request), then gate the
 *      M7 clock off, clear M7_CFG.WAIT and gate the clock back on.  The
 *      rising clock edge is what kicks the core off.
 *
 * The sequence follows IMXRT1180RM 12.10 "Cortex-M7 kick-off procedure".
 *
 * Nothing is written to LPUART1 after the release: the M7 re-initialises
 * the console for itself.
 *
 * Step 4 is mandatory on this part.  The M7 does not boot directly from
 * XIP flash: INITVTOR is set to 0 so the core takes its reset vector from
 * ITCM address 0, and the kick-off vector staged there (initial SP plus a
 * reset PC pointing into the XIP image at 0x2808_0000) is what transfers
 * control to the real image.  This mirrors NXP MCUXpresso SDK
 * Prepare_CM7(), which every RT1180 example calls with
 * CORE1_KICKOFF_ADDRESS == 0x0.
 *
 * Register layout: IMXRT1180RM Ch. 15 (BLK_CTRL_S_AONMIX), Ch. 27 (SRC),
 * Ch. 30 (eDMA4).
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "imxrt1180-evk.h"
#include "imxrt118x_ele.h"
#include "hardware/rt118x/imxrt118x_anadig.h"
#include "hardware/imxrt_lpuart.h"
#include "imxrt_clockconfig.h"
#include "hardware/imxrt_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG32(a)                    (*(volatile uint32_t *)(uintptr_t)(a))
#define REG16(a)                    (*(volatile uint16_t *)(uintptr_t)(a))

#define BLK_CTRL_S_AONMIX_BASE      0x444f0000u
#define AON_M7_CFG                  (BLK_CTRL_S_AONMIX_BASE + 0x80)
#define AON_M7_CFG_WAIT             (1u << 4)
#define AON_M7_CFG_CORECLK_FORCE_ON (1u << 5)
#define AON_M7_CFG_HCLK_FORCE_ON    (1u << 6)
#define AON_M7_CFG_INITVTOR_MASK    0xffffff80u
#define AON_M7_CFG_INITVTOR_SHIFT   7u

/* The INITVTOR field holds the vector-table address shifted right by 7
 * (it is 128-byte aligned), placed at bit 7.  See NXP MCUXpresso SDK
 * BLK_CTRL_S_AONMIX_M7_CFG_INITVTOR(m7_vtor >> 7).
 */

#define AON_M7_CFG_INITVTOR(a)      ((((a) >> 7) << AON_M7_CFG_INITVTOR_SHIFT) \
                                     & AON_M7_CFG_INITVTOR_MASK)

#define SRC_BASE                    0x44460000u
#define SRC_SCR                     (SRC_BASE + 0x10)
#define SRC_SCR_BT_RELEASE_M7       (1u << 0)
#define SRC_AON_SLICE_BASE          (SRC_BASE + 0x800)
#define SRC_WKUP_SLICE_BASE         (SRC_BASE + 0xc00)

/* Base of the Cortex-M7 XIP image.  Must match the 'flash' region ORIGIN
 * in boards/arm/imxrt/imxrt1180-evk/scripts/flash.ld: the first 512 KB of
 * FlexSPI1 NOR is reserved for this (M33) image.
 */

#define M7_ENTRY                    0x28080000u

/* The M7 boots from its ITCM at address 0, never directly from XIP flash.
 * NXP MCUXpresso SDK passes CORE1_KICKOFF_ADDRESS == 0x0 to Prepare_CM7()
 * in every RT1180 example: INITVTOR selects the ITCM vector table, and the
 * two-word kick-off vector installed there (SP + reset PC pointing into
 * the XIP image) is what actually starts the M7.
 */

#define M7_INITVTOR                 0x00000000u

/* Analog / PMIC registers required to bring up the M7 core clock, ported
 * from NXP MCUXpresso SDK Prepare_CM7()
 * (devices/RT/RT1180/MIMXRT1187/system_MIMXRT1187_cm33.c).
 */

#define PHY_LDO_CTRL0_VAL           (PHY_LDO_CTRL0_LINREG_OUTPUT_TRG(0x10) | \
                                     PHY_LDO_CTRL0_LINREG_ILIMIT_EN | \
                                     PHY_LDO_CTRL0_LINREG_EN)

/* ARM PLL: 24 MHz * 132 / 4 = 792 MHz, close to the 800 MHz that
 * imxrt_clockconfig_ver3.c assumes for ARM_PLL_OUT.
 */

#define ARM_PLL_DIV_SELECT          132u

/* eDMA4 channel-0 TCD used to write the M7 TCM aliases from the M33 side.
 * The M7 ITCM/DTCM are ECC-protected: reading a location that has never
 * been written raises an ECC fault, so the TCMs are scrubbed before the
 * M7 is released.  M7 TCMs appear as 4x128 KB aliases in the SoC address
 * map (IMXRT1180RM Ch. 3 Table 4).
 */

#define DMA4_BASE                   0x42000000u
#define DMA4_CH0_CH_CSR             (DMA4_BASE + 0x10000)   /* 32b */
#define DMA4_CH0_SADDR              (DMA4_BASE + 0x10020)   /* 32b */
#define DMA4_CH0_SOFF               (DMA4_BASE + 0x10024)   /* 16b */
#define DMA4_CH0_ATTR               (DMA4_BASE + 0x10026)   /* 16b */
#define DMA4_CH0_NBYTES_MLOFFNO     (DMA4_BASE + 0x10028)   /* 32b */
#define DMA4_CH0_DADDR              (DMA4_BASE + 0x10030)   /* 32b */
#define DMA4_CH0_DOFF               (DMA4_BASE + 0x10034)   /* 16b */
#define DMA4_CH0_CITER_ELINKNO      (DMA4_BASE + 0x10036)   /* 16b */
#define DMA4_CH0_CSR                (DMA4_BASE + 0x1003c)   /* 16b */
#define DMA4_CH0_BITER_ELINKNO      (DMA4_BASE + 0x1003e)   /* 16b */

#define DMA4_CH_CSR_DONE            (1u << 30)

#define M7_ITCM_ALIAS               0x303c0000u

/* eDMA4 source scratch.  Lives in the OCRAM1 area that the M7 linker
 * script (scripts/flash.ld) excludes from its 'ocram' region, so the M7
 * image never overlaps it.
 */

#define DMA_SCRATCH                 0x204fc000u

/* CCM M7 CLKROOT (root index 0, control @ CCM_BASE + 0x0).  MUX field is
 * bits [9:8]; mux 2 selects ARM PLL OUT.  Without a valid mux the M7 core
 * has no clock and never fetches a single instruction.
 */

#define CCM_M7_CLK_ROOT_CONTROL     0x44450000u
#define CCM_M7_MUX_ARM_PLL_OUT      (0x2u << 8)
#define CCM_M7_ROOT_CTRL_VAL        CCM_M7_MUX_ARM_PLL_OUT

#define CCM_LPCG0_DIRECT            (0x44450000u + 0x8000u)
#define CCM_LPCG_DIRECT_ON          (1u << 0)

/* DCDC REG3.  The EVK has no external PMIC, so the SoC buck regulator
 * supplies the core domain.  NXP MCUXpresso SDK Prepare_CM7() raises the
 * regulator feedback tap and disables idle/pulse skipping before the M7
 * runs; without it the M7 domain is supplied marginally and the core may
 * fail to start (or start only intermittently).
 */

#define DCDC_BASE                   0x44520000u
#define DCDC_REG3                   (DCDC_BASE + 0x0c)
#define DCDC_REG3_REG_FBK_SEL(x)    (((x) << 22) & 0x00c00000u)
#define DCDC_REG3_DISABLE_PULSE_SKIP (1u << 19)
#define DCDC_REG3_DISABLE_IDLE_SKIP (1u << 20)
#define DCDC_REG3_VAL               (DCDC_REG3_REG_FBK_SEL(2) | \
                                     DCDC_REG3_DISABLE_IDLE_SKIP | \
                                     DCDC_REG3_DISABLE_PULSE_SKIP)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prepare_arm_pll
 ****************************************************************************/

static int prepare_arm_pll(void)
{
  /* Enable the PHY LDO that powers the analog PLL IP, bring the ARM PLL
   * up and wait for lock, then route it to the M7 core via CCM
   * CLOCK_ROOT[M7] mux 2 (ArmPllOut).  The M7 receives its core clock
   * from the ARM PLL and cannot program it itself.
   */

  volatile uint32_t timeout;

  REG32(IMXRT_PHY_LDO_CTRL0_RW) = PHY_LDO_CTRL0_VAL;

  /* Power the PLL up with its output still gated, then wait for it to
   * report stable.  Switching the M7 root mux to an unlocked PLL clocks
   * the core from a slewing, far-too-slow reference: the M7 still runs,
   * but at a tiny fraction of its intended speed.
   */

  REG32(IMXRT_ANADIG_PLL_ARM_CTRL) = ANADIG_PLL_ARM_GATE |
                                     ANADIG_PLL_ARM_POWERUP |
                                     ANADIG_PLL_ARM_DIV_SELECT(
                                       ARM_PLL_DIV_SELECT);

  timeout = 1000000;
  while (timeout-- &&
         (REG32(IMXRT_ANADIG_PLL_ARM_CTRL) & ANADIG_PLL_ARM_STABLE) == 0);

  if (timeout == 0)
    {
      syslog(LOG_ERR, "bootloader: ARM PLL did not lock (CTRL=0x%08" PRIx32
             ")\n", REG32(IMXRT_ANADIG_PLL_ARM_CTRL));
      return -ETIMEDOUT;
    }

  /* PLL is locked: ungate its output and enable the clock. */

  REG32(IMXRT_ANADIG_PLL_ARM_CTRL) = ANADIG_PLL_ARM_ENABLE_CLK |
                                     ANADIG_PLL_ARM_POWERUP |
                                     ANADIG_PLL_ARM_DIV_SELECT(
                                       ARM_PLL_DIV_SELECT);

  REG32(CCM_M7_CLK_ROOT_CONTROL) = CCM_M7_ROOT_CTRL_VAL;

  /* Set up the core supply for the M7 domain (no external PMIC on the
   * EVK).  Must happen before the M7 starts executing.
   */

  REG32(DCDC_REG3) = DCDC_REG3_VAL;

  return 0;
}

/****************************************************************************
 * Name: dma_fill
 *
 * Description:
 *   Program eDMA4 channel-0 to hammer the 8 bytes at DMA_SCRATCH across a
 *   128 KB destination window.  SOFF is zero so the same source word pair
 *   is re-read for every 8-byte beat.
 *
 ****************************************************************************/

static int dma_fill(uint32_t target)
{
  volatile uint32_t timeout = 100000;

  REG32(DMA4_CH0_SADDR)          = DMA_SCRATCH;
  REG32(DMA4_CH0_DADDR)          = target;
  REG32(DMA4_CH0_NBYTES_MLOFFNO) = 0x20000;
  REG16(DMA4_CH0_CITER_ELINKNO)  = 0x1;
  REG16(DMA4_CH0_BITER_ELINKNO)  = 0x1;
  REG16(DMA4_CH0_ATTR)           = 0x303;
  REG16(DMA4_CH0_SOFF)           = 0;
  REG16(DMA4_CH0_DOFF)           = 0x8;
  REG32(DMA4_CH0_CH_CSR)         = 0x7;
  REG16(DMA4_CH0_CSR)            = 0x8;
  REG16(DMA4_CH0_CSR)            = 0x9;
  REG32(DMA4_CH0_CH_CSR)         = 0x40000006;

  while (timeout-- && (REG32(DMA4_CH0_CH_CSR) & DMA4_CH_CSR_DONE) == 0);

  if (timeout == 0)
    {
      syslog(LOG_ERR, "bootloader: eDMA4 fill of 0x%08" PRIx32 " timed out "
             "(CH_CSR=0x%08" PRIx32 ")\n", target,
             REG32(DMA4_CH0_CH_CSR));
      return -ETIMEDOUT;
    }

  REG32(DMA4_CH0_CH_CSR) = DMA4_CH_CSR_DONE;
  return 0;
}

/****************************************************************************
 * Name: prepare_edma4
 *
 * Description:
 *   Clock eDMA4.  The bootloader does not build the NuttX eDMA driver, so
 *   nothing else programs wakeup_axi_clk_root or the eDMA4 LPCG.  Without
 *   this the TCD writes below land on an unclocked peripheral and are
 *   silently lost.
 *
 ****************************************************************************/

static void prepare_edma4(void)
{
  imxrt_ccm_configure_root_clock(CCM_CR_WAKEUP_AXI, SYS_PLL3_OUT, 2);
  imxrt_ccm_gate_on(CCM_LPCG_EDMA4, true);
}

/****************************************************************************
 * Name: init_cm7_tcm
 *
 * Description:
 *   Zero all four 128 KB M7 TCM aliases to initialise their ECC syndromes.
 *
 ****************************************************************************/

static int init_cm7_tcm(void)
{
  REG32(DMA_SCRATCH)     = 0;
  REG32(DMA_SCRATCH + 4) = 0;

  return dma_fill(0x303c0000u) | dma_fill(0x303e0000u) |
         dma_fill(0x30400000u) | dma_fill(0x30420000u);
}

/****************************************************************************
 * Name: install_cm7_kickoff_vectors
 *
 * Description:
 *   Place the SP / Reset_Handler pair at M7 ITCM address 0, where the boot
 *   ROM fetches the M7 reset vector from.  The M7 loads MSP and PC from
 *   here, then jumps to the real Reset_Handler in FlexSPI XIP which points
 *   VTOR at the real vector table.
 *
 *   A plain CPU store is attempted first.  The legacy boot stub asserted
 *   that M33 stores to this alias are dropped and only DMA masters may
 *   write it; if the readback below succeeds, that is not true and the
 *   eDMA path is unnecessary.  The result is logged either way.
 *
 ****************************************************************************/

static int install_cm7_kickoff_vectors(uint32_t sp, uint32_t reset_handler)
{
  REG32(M7_ITCM_ALIAS + 0) = sp;
  REG32(M7_ITCM_ALIAS + 4) = reset_handler;
  __asm__ volatile ("dsb 0xf" : : : "memory");

  if (REG32(M7_ITCM_ALIAS + 0) == sp &&
      REG32(M7_ITCM_ALIAS + 4) == reset_handler)
    {
      syslog(LOG_INFO, "bootloader: ITCM kick-off vector written by CPU\n");
      return 0;
    }

  syslog(LOG_INFO, "bootloader: ITCM CPU store ineffective, using eDMA4\n");

  REG32(DMA_SCRATCH)     = sp;
  REG32(DMA_SCRATCH + 4) = reset_handler;

  return dma_fill(M7_ITCM_ALIAS);
}

/****************************************************************************
 * Name: dump_release_state
 *
 * Description:
 *   Log every register that participates in the M7 release, so the state
 *   this bootloader leaves behind can be diffed against the state a J-Link
 *   connect produces (its InitTarget() releases the M7 successfully).
 *
 ****************************************************************************/

static void dump_release_state(const char *tag)
{
  syslog(LOG_INFO,
         "bl[%s]: M7_CFG=%08" PRIx32 " SRC_SCR=%08" PRIx32
         " ARM_PLL=%08" PRIx32 "\n",
         tag, REG32(AON_M7_CFG), REG32(SRC_SCR),
         REG32(IMXRT_ANADIG_PLL_ARM_CTRL));

  syslog(LOG_INFO,
         "bl[%s]: M7ROOT=%08" PRIx32 " AON_SLICE=%08" PRIx32
         " WKUP_SLICE=%08" PRIx32 "\n",
         tag, REG32(CCM_M7_CLK_ROOT_CONTROL),
         REG32(SRC_AON_SLICE_BASE), REG32(SRC_WKUP_SLICE_BASE));

  syslog(LOG_INFO,
         "bl[%s]: ITCM[0]=%08" PRIx32 " ITCM[1]=%08" PRIx32 "\n",
         tag, REG32(M7_ITCM_ALIAS + 0), REG32(M7_ITCM_ALIAS + 4));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bootloader_main
 *
 * Description:
 *   Entry point of the imxrt1180-evk "bl" configuration.  Releases the
 *   Cortex-M7 and then exits; the M33 idle loop takes over.
 *
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  uint32_t cfg;
  int ret;

  syslog(LOG_INFO, "imxrt1180-evk bootloader: releasing M7 @ 0x%08" PRIx32
         "\n", (uint32_t)M7_ENTRY);

  /* Bring the ARM PLL up and route it to the M7 core clock root.  This is
   * the one clock the M7 cannot set up itself: without a core clock it
   * never executes the instruction that would program it.  Everything
   * else the M7 needs (bus roots, peripheral LPCGs) it configures for
   * itself in imxrt_clockconfig(); the ELE has already opened TRDC access
   * so those writes succeed.
   */

  if (prepare_arm_pll() < 0)
    {
      syslog(LOG_ERR, "bootloader: not releasing M7\n");
      return EXIT_FAILURE;
    }

  /* Program the M7 initial VTOR (bits [31:7] of the target address).
   * INITVTOR is a byte-address field aligned to 128 bytes.  Also force
   * HCLK / CORECLK on so a debugger can reach the M7 CoreSight regardless
   * of what the CPU is doing.
   */

  cfg = REG32(AON_M7_CFG);
  cfg = (cfg & ~AON_M7_CFG_INITVTOR_MASK) |
        AON_M7_CFG_INITVTOR(M7_INITVTOR) |
        AON_M7_CFG_HCLK_FORCE_ON |
        AON_M7_CFG_CORECLK_FORCE_ON;
  REG32(AON_M7_CFG) = cfg;

  /* Clock eDMA4 before any TCM access: the scrub below needs it, and a
   * read of never-written ECC memory faults.
   */

  prepare_edma4();

  /* Release the M7 from reset (write-once). */

  REG32(SRC_SCR) = REG32(SRC_SCR) | SRC_SCR_BT_RELEASE_M7;

  /* Initialise the M7 TCM ECC, then stage the reset vector the ROM will
   * actually fetch.  Both must happen before WAIT is cleared.
   */

  if (init_cm7_tcm() < 0)
    {
      syslog(LOG_ERR, "bootloader: TCM scrub failed, not releasing M7\n");
      return EXIT_FAILURE;
    }

  if (install_cm7_kickoff_vectors(REG32(M7_ENTRY + 0),
                                  REG32(M7_ENTRY + 4)) < 0)
    {
      syslog(LOG_ERR, "bootloader: kick-off vector install failed, "
             "not releasing M7\n");
      return EXIT_FAILURE;
    }

  /* IMXRT1180RM 12.10 step 9: ask the ELE to release the M7 ("Enable APC
   * request").  Until this is done the enclave keeps the core held, no
   * matter what SRC.SCR and M7_CFG say.
   */

  ret = imxrt118x_ele_enable_apc();
  if (ret < 0)
    {
      syslog(LOG_ERR, "bootloader: ELE_ENABLE_APC failed (%d)\n", ret);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO, "bootloader: ELE APC enabled\n");

  /* Dump the full release state, then perform the kick-off itself:
   * IMXRT1180RM 12.10 steps 10-12 - gate the M7 clock off, clear CPUWAIT,
   * and gate the clock back on.  The rising clock is what actually starts
   * the core; simply clearing WAIT is not enough.
   */

  dump_release_state("pre-kick");

  REG32(CCM_LPCG0_DIRECT) = 0;

  REG32(AON_M7_CFG) = REG32(AON_M7_CFG) & ~AON_M7_CFG_WAIT;

  REG32(CCM_LPCG0_DIRECT) = CCM_LPCG_DIRECT_ON;

  up_udelay(20000);

  dump_release_state("post-kick");

  syslog(LOG_INFO, "imxrt1180-evk bootloader: M7 released\n");
  return 0;
}
