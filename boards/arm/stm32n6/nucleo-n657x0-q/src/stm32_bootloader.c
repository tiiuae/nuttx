/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_bootloader.c
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

#include <stdint.h>

#include <arch/barriers.h>
#include <arch/irq.h>
#include <debug.h>

#include "arm_internal.h"
#include "nvic.h"
#include "hardware/stm32n6xxx_memorymap.h"
#include "hardware/stm32n6xxx_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The application is signed with a 0x400-byte STM32 v2.3 header at
 * 0x70100000.  Its vector table is the first item in the payload.
 */

#define NSH_XSPI_VECTOR_BASE           0x70100400ul

/* Kernel clock mux registers.  Only CCIPR13 (USART1) is currently declared
 * in hardware/stm32n6xxx_rcc.h, but the XSPI kernel clock selection lives
 * in this same contiguous block, so dump all of them.  CCIPRn is at
 * 0x0144 + 4 * (n - 1); CCIPR13 at 0x0174 matches the arch header.
 */

#define STM32_RCC_CCIPR1               (STM32_RCC_BASE + 0x0144)
#define STM32_RCC_CCIPR_COUNT          14

/* Peripheral clock enable registers.  XSPI1/2/3 and the XSPIM I/O manager
 * sit on AHB5, not in MEMENR (which only covers the SRAM/cache RAMs), so
 * AHB5ENR is the register that says whether XSPI2 is clocked at all.  The
 * arch header does not declare it; the offset follows from the regular
 * RM0486 layout that the declared offsets already match:
 *
 *   0x0250 AHB1ENR   0x0254 AHB2ENR   0x0258 AHB3ENR
 *   0x025c AHB4ENR   0x0260 AHB5ENR   0x0264 APB1ENR1  ...
 *
 * Dump the whole 0x0240..0x027c enable block so the surrounding registers
 * (DIVENR, BUSENR, MISCENR, MEMENR) can be cross-checked in one shot.
 */

#define STM32_RCC_AHB5ENR              (STM32_RCC_BASE + 0x0260)
#define STM32_RCC_AHB5ENSR             (STM32_RCC_BASE + 0x0a60)
#define STM32_RCC_ENR_DUMP_FIRST       0x0240
#define STM32_RCC_ENR_DUMP_LAST        0x027c

/* Sleep-mode clock enables.  The LPENR block mirrors the ENR block at a
 * +0x40 offset (MEMENR 0x24c / MEMLPENR 0x28c, APB2ENR 0x26c / APB2LPENR
 * 0x2ac, ...), so AHB5LPENR follows AHB5ENR at 0x2a0.  Set registers are a
 * further +0x800.
 */

#define STM32_RCC_AHB5LPENR            (STM32_RCC_BASE + 0x02a0)
#define STM32_RCC_AHB5LPENSR           (STM32_RCC_BASE + 0x0aa0)

#define RCC_AHB5LPENR_XSPI2LPEN        (1 << 12)
#define RCC_AHB5LPENR_XSPIMLPEN        (1 << 13)

/* RCC_AHB5ENR bits of interest (RM0486 / CMSIS stm32n657xx.h) */

#define RCC_AHB5ENR_XSPI1EN            (1 << 5)
#define RCC_AHB5ENR_XSPI2EN            (1 << 12)
#define RCC_AHB5ENR_XSPIMEN            (1 << 13)

/* RCC_CCIPR6 XSPI2 kernel clock mux: 0=HCLK 1=CLKP 2=IC3 3=IC4 */

#define STM32_RCC_CCIPR6               (STM32_RCC_CCIPR1 + 4 * 5)
#define RCC_CCIPR6_XSPI2SEL_SHIFT      (4)
#define RCC_CCIPR6_XSPI2SEL_MASK       (3 << RCC_CCIPR6_XSPI2SEL_SHIFT)
#define RCC_CCIPR6_XSPI2SEL_CLKP       (1 << RCC_CCIPR6_XSPI2SEL_SHIFT)

/* XSPI1 and the XSPIM I/O manager.  AHB5ENR shows both are clocked by the
 * boot ROM (XSPI1EN and XSPIMEN set, XSPI2EN clear), so unlike XSPI2 these
 * registers can be read without stalling the bus.
 *
 * The question they answer: did the ROM boot this board through the XSPI1
 * controller routed to port 2 by the XSPIM multiplexer?  If so the flash is
 * memory mapped at XSPI1's window (0x90000000), not at 0x70000000.
 */

#define STM32_XSPI1_BASE               0x58025000
#define STM32_XSPIM_CR                 (STM32_XSPIM_BASE + 0x000)

#define STM32_XSPI_CR_OFFSET           0x000
#define STM32_XSPI_DCR1_OFFSET         0x008
#define STM32_XSPI_DCR2_OFFSET         0x00c
#define STM32_XSPI_DCR3_OFFSET         0x010
#define STM32_XSPI_DCR4_OFFSET         0x014
#define STM32_XSPI_SR_OFFSET           0x020
#define STM32_XSPI_CCR_OFFSET          0x100
#define STM32_XSPI_TCR_OFFSET          0x108
#define STM32_XSPI_IR_OFFSET           0x110

#define XSPI_CR_EN                     (1 << 0)
#define XSPI_CR_FMODE_SHIFT            (28)
#define XSPI_CR_FMODE_MASK             (3 << XSPI_CR_FMODE_SHIFT)
#define XSPI_CR_FMODE_MEMMAPPED        (3)

#define XSPI_SR_BUSY                   (1 << 5)

#define XSPIM_CR_MUXEN                 (1 << 0)
#define XSPIM_CR_MODE                  (1 << 1)

/* XSPI1 memory-mapped window.  Not used for booting -- the application is
 * linked for XSPI2's window -- but retained for the diagnostic probe in
 * stm32_dump_xspi1(), and as the fallback if XSPI2 bring-up ever proves
 * problematic: the ROM's own XSPI1 configuration reaches the same flash
 * here with nothing more than a CR.FMODE change.
 */

#define XSPI1_MEM_BASE                 0x90000000ul

#define XSPI_BUSY_TIMEOUT              1000000

/* Transaction configuration the boot ROM negotiated with the flash and
 * used to load this image: 0x0b fast read, single-line instruction,
 * address and data, 24-bit address, 8 dummy cycles, prescaler /2.  These
 * values are replayed verbatim onto XSPI2 rather than invented, so the
 * memory-mapped window runs at timings already proven on this board.
 *
 * DEVSIZE is left at the ROM's 0x1f.  Note that the 24-bit address limits
 * the window to the first 16 MB of the 64 MB device; the payload at
 * 0x00100400 is well inside that, but larger images will need 4-byte
 * addressing (and a matching read opcode).
 */

#define XSPI_ROM_DCR1                  0x001f0000ul
#define XSPI_ROM_DCR2                  0x00000001ul
#define XSPI_ROM_CCR                   0x01002101ul
#define XSPI_ROM_TCR                   0x00000008ul
#define XSPI_ROM_IR                    0x0000000bul

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dump_xspi_clocks
 *
 * Description:
 *   Dump the RCC state that governs whether XSPI2 is usable at all.  This
 *   only touches the RCC block, which is always clocked, so it is safe to
 *   call before any XSPI2 register or memory-mapped access -- both of which
 *   stall the bus if the XSPI2 peripheral clock is gated off.
 *
 *   What to look for:
 *     AHB5ENR - the XSPI2 / XSPIM enable bits (see RM0486 RCC_AHB5ENR).
 *               If they are clear, the boot ROM did not leave XSPI2
 *               clocked and it must be re-initialized here before either
 *               its registers or 0x70000000 can be touched.
 *     CCIPRn  - the XSPI kernel clock source mux.
 *     IC3CFGR / DIVENR - the IC3 divider feeding the XSPI2 kernel clock.
 *     MEMENR  - SRAM/cache RAM clocks only; shown for completeness.
 *
 ****************************************************************************/

static void stm32_dump_xspi_clocks(void)
{
  uint32_t ahb5enr = getreg32(STM32_RCC_AHB5ENR);
  uint32_t ccipr6  = getreg32(STM32_RCC_CCIPR6);
  uint32_t offset;
  int i;

  _alert("RCC_AHB5ENR: %08lx", (unsigned long)ahb5enr);
  _alert("RCC_AHB5LPEN:%08lx",
         (unsigned long)getreg32(STM32_RCC_AHB5LPENR));
  _alert("  XSPI1EN=%d XSPI2EN=%d XSPIMEN=%d XSPI2SEL=%d (0=HCLK 1=CLKP "
         "2=IC3 3=IC4)",
         (ahb5enr & RCC_AHB5ENR_XSPI1EN) != 0,
         (ahb5enr & RCC_AHB5ENR_XSPI2EN) != 0,
         (ahb5enr & RCC_AHB5ENR_XSPIMEN) != 0,
         (int)((ccipr6 & RCC_CCIPR6_XSPI2SEL_MASK) >>
               RCC_CCIPR6_XSPI2SEL_SHIFT));
  _alert("RCC_MEMENR : %08lx",
         (unsigned long)getreg32(STM32_RCC_MEMENR));
  _alert("RCC_DIVENR : %08lx",
         (unsigned long)getreg32(STM32_RCC_DIVENR));
  _alert("RCC_IC3CFGR: %08lx",
         (unsigned long)getreg32(STM32_RCC_IC3CFGR));
  _alert("RCC_CFGR1  : %08lx",
         (unsigned long)getreg32(STM32_RCC_CFGR1));
  _alert("RCC_CFGR2  : %08lx",
         (unsigned long)getreg32(STM32_RCC_CFGR2));

  for (i = 0; i < STM32_RCC_CCIPR_COUNT; i++)
    {
      _alert("RCC_CCIPR%-2d: %08lx", i + 1,
             (unsigned long)getreg32(STM32_RCC_CCIPR1 + 4 * i));
    }

  /* Raw dump of the peripheral clock enable block, so the AHB5ENR offset
   * inferred above can be confirmed against its neighbours.
   */

  for (offset = STM32_RCC_ENR_DUMP_FIRST;
       offset <= STM32_RCC_ENR_DUMP_LAST;
       offset += 4)
    {
      _alert("RCC+0x%03lx  : %08lx", (unsigned long)offset,
             (unsigned long)getreg32(STM32_RCC_BASE + offset));
    }
}

/****************************************************************************
 * Name: stm32_dump_xspi1
 *
 * Description:
 *   Dump the XSPIM multiplexer and the XSPI1 controller state left behind
 *   by the boot ROM.  Both blocks are clocked (AHB5ENR.XSPIMEN and
 *   AHB5ENR.XSPI1EN are set), so these reads are safe -- in contrast to
 *   XSPI2, whose clock is gated and whose registers stall the bus.
 *
 *   Decision this drives:
 *     CR.EN = 1 and CR.FMODE = 3 on XSPI1 means the ROM left a live
 *     memory-mapped window at 0x90000000, and the application should be
 *     reached there rather than through an XSPI2 bring-up.  CR.EN = 0
 *     means the ROM tore its configuration down and a full controller +
 *     flash initialization is unavoidable.
 *
 ****************************************************************************/

static void stm32_dump_xspi1(void)
{
  uint32_t cr = getreg32(STM32_XSPI1_BASE + STM32_XSPI_CR_OFFSET);
  uint32_t xspim_cr = getreg32(STM32_XSPIM_CR);

  _alert("XSPIM_CR   : %08lx (MUXEN=%d MODE=%d)",
         (unsigned long)xspim_cr,
         (xspim_cr & XSPIM_CR_MUXEN) != 0,
         (xspim_cr & XSPIM_CR_MODE) != 0);

  _alert("XSPI1_CR   : %08lx (EN=%d FMODE=%d, 3=memory-mapped)",
         (unsigned long)cr, (cr & XSPI_CR_EN) != 0,
         (int)((cr & XSPI_CR_FMODE_MASK) >> XSPI_CR_FMODE_SHIFT));

  _alert("XSPI1_DCR1 : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_DCR1_OFFSET));
  _alert("XSPI1_DCR2 : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_DCR2_OFFSET));
  _alert("XSPI1_DCR3 : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_DCR3_OFFSET));
  _alert("XSPI1_DCR4 : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_DCR4_OFFSET));
  _alert("XSPI1_SR   : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_SR_OFFSET));
  _alert("XSPI1_CCR  : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_CCR_OFFSET));
  _alert("XSPI1_TCR  : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_TCR_OFFSET));
  _alert("XSPI1_IR   : %08lx",
         (unsigned long)getreg32(STM32_XSPI1_BASE + STM32_XSPI_IR_OFFSET));

  /* If, and only if, XSPI1 reports a live memory-mapped window, read the
   * first words of its region.  This is the one access here that can stall
   * the bus, so it is gated on FMODE and bracketed by prints: if the trace
   * stops after "XSPI1 probe 0x90000000", the window is not actually
   * readable despite the controller state.
   */

  if ((cr & XSPI_CR_EN) != 0 &&
      ((cr & XSPI_CR_FMODE_MASK) >> XSPI_CR_FMODE_SHIFT) ==
       XSPI_CR_FMODE_MEMMAPPED)
    {
      const uint32_t *mem = (const uint32_t *)XSPI1_MEM_BASE;

      _alert("XSPI1 probe %08lx ...", (unsigned long)XSPI1_MEM_BASE);
      _alert("XSPI1 [0..1]: %08lx %08lx",
             (unsigned long)mem[0], (unsigned long)mem[1]);
    }
  else
    {
      _alert("XSPI1 not memory-mapped; skipping window probe");
    }
}

/****************************************************************************
 * Name: stm32_xspi2_memorymap
 *
 * Description:
 *   Bring up XSPI2 as the memory-mapped XIP window at 0x70000000.
 *
 *   The boot ROM loads this image over the XSPI1 controller, which it
 *   routes to the port-2 pins by putting XSPIM into swapped mode
 *   (XSPIM_CR.MODE=1), and it leaves XSPI1 in indirect-read mode.  XSPI2
 *   itself is left clock-gated, which is why any access to its registers
 *   or to 0x70000000 stalls the bus until this function has run.
 *
 *   Since the application is linked for XSPI2's window, hand the flash
 *   over to XSPI2 rather than relocating the image to XSPI1's window at
 *   0x90000000:
 *
 *     1. quiesce and disable XSPI1 so it releases the bus
 *     2. enable the XSPI2 bus clock
 *     3. point XSPI2's kernel clock at CLKP, the same source XSPI1 used,
 *        so the ROM's prescaler yields the same serial clock
 *     4. switch XSPIM to direct mode, giving port 2 to XSPI2
 *     5. replay the ROM's transaction configuration
 *     6. arm memory-mapped mode
 *
 *   The port-2 pins need no attention: the ROM configured them as AF9
 *   (XSPIM_P2), which selects the I/O manager port rather than a specific
 *   controller, so they stay correct across the mux change.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value if XSPI1 never goes idle.
 *
 ****************************************************************************/

static int stm32_xspi2_memorymap(void)
{
  uint32_t cr;
  int timeout;

  /* 1. Wait for the ROM's last XSPI1 transfer to retire, then disable the
   *    controller.  XSPI1 is clocked, so these accesses are safe.
   */

  for (timeout = XSPI_BUSY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_XSPI1_BASE + STM32_XSPI_SR_OFFSET) &
           XSPI_SR_BUSY) == 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      _alert("ERROR: XSPI1 stuck busy");
      return -EBUSY;
    }

  cr = getreg32(STM32_XSPI1_BASE + STM32_XSPI_CR_OFFSET);
  putreg32(cr & ~XSPI_CR_EN, STM32_XSPI1_BASE + STM32_XSPI_CR_OFFSET);
  UP_DSB();

  /* 2. Enable the XSPI2 bus clock.  Until this write retires, every XSPI2
   *    access below would stall.
   */

  putreg32(RCC_AHB5ENR_XSPI2EN, STM32_RCC_AHB5ENSR);
  UP_DSB();

  /* The application executes in place from this window and idles in WFI.
   * Without the sleep-mode enables, CSLEEP gates the XSPI2 and XSPIM
   * clocks, the memory-mapped window disappears, and the first instruction
   * fetch after WFI takes a hard fault in the idle task.
   *
   * This is set here, in the code that owns the XSPI2 configuration, and
   * survives into the application: arch/arm/src/stm32n6/stm32_start.c only
   * ever writes the atomic LPENSR set registers, so it cannot clear these
   * bits.
   */

  putreg32(RCC_AHB5LPENR_XSPI2LPEN | RCC_AHB5LPENR_XSPIMLPEN,
           STM32_RCC_AHB5LPENSR);
  UP_DSB();

  /* 3. XSPI2 comes out of reset muxed to HCLK, which is far faster than
   *    the ROM's prescaler assumes.  Select CLKP to match XSPI1 exactly.
   */

  modifyreg32(STM32_RCC_CCIPR6, RCC_CCIPR6_XSPI2SEL_MASK,
              RCC_CCIPR6_XSPI2SEL_CLKP);
  UP_DSB();

  /* 4. Direct mode: XSPI1 -> port 1, XSPI2 -> port 2. */

  putreg32(0, STM32_XSPIM_CR);
  UP_DSB();

  /* 5. Replay the ROM's proven transaction configuration while the
   *    controller is disabled.
   */

  putreg32(0, STM32_XSPI2_BASE + STM32_XSPI_CR_OFFSET);
  putreg32(XSPI_ROM_DCR1, STM32_XSPI2_BASE + STM32_XSPI_DCR1_OFFSET);
  putreg32(XSPI_ROM_DCR2, STM32_XSPI2_BASE + STM32_XSPI_DCR2_OFFSET);
  putreg32(0, STM32_XSPI2_BASE + STM32_XSPI_DCR3_OFFSET);
  putreg32(0, STM32_XSPI2_BASE + STM32_XSPI_DCR4_OFFSET);
  putreg32(XSPI_ROM_CCR, STM32_XSPI2_BASE + STM32_XSPI_CCR_OFFSET);
  putreg32(XSPI_ROM_TCR, STM32_XSPI2_BASE + STM32_XSPI_TCR_OFFSET);
  putreg32(XSPI_ROM_IR, STM32_XSPI2_BASE + STM32_XSPI_IR_OFFSET);

  /* 6. Arm memory-mapped mode and enable. */

  cr = XSPI_CR_FMODE_MEMMAPPED << XSPI_CR_FMODE_SHIFT;
  putreg32(cr, STM32_XSPI2_BASE + STM32_XSPI_CR_OFFSET);
  putreg32(cr | XSPI_CR_EN, STM32_XSPI2_BASE + STM32_XSPI_CR_OFFSET);

  UP_DSB();
  UP_ISB();

  /*
  _alert("XSPI2_CR   : %08lx (after)",
         (unsigned long)getreg32(STM32_XSPI2_BASE + STM32_XSPI_CR_OFFSET));
  _alert("RCC_AHB5LPEN:%08lx (after)",
         (unsigned long)getreg32(STM32_RCC_AHB5LPENR));
  */

  return OK;
}

/****************************************************************************
 * Name: stm32_boot_nsh_xspi
 *
 * Description:
 *   Bring up the XIP window, then install the application's vector table
 *   and branch to its reset handler.  This has the same architectural
 *   effect as a reset into the application.
 *
 ****************************************************************************/

static void __attribute__((noreturn)) stm32_boot_nsh_xspi(void)
{
  const uint32_t *vectors = (const uint32_t *)NSH_XSPI_VECTOR_BASE;
  uint32_t msp;
  uint32_t reset;

  _alert("memory-map XSPI2");
  _alert("");

  /* Report the state the boot ROM left behind before touching XSPI2.  Only
   * RCC, XSPIM and XSPI1 are read here; all three are clocked, unlike XSPI2.
   */

  //stm32_dump_xspi_clocks();
  //stm32_dump_xspi1();

  up_irq_disable();
  putreg32(0, NVIC_SYSTICK_CTRL);

  /* Do not dereference the application address until this has succeeded.
   * The pointer assignment above is local-only; vectors[0] below causes
   * the first memory-mapped XSPI2 transaction.
   */

  if (stm32_xspi2_memorymap() < 0)
    {
      _alert("ERROR: XSPI2 memory-map failed");
      for (; ; );
    }

  msp   = vectors[0];
  reset = vectors[1];

  putreg32(NSH_XSPI_VECTOR_BASE, NVIC_VECTAB);
  UP_DSB();
  UP_ISB();

  /* bootloader_main() runs in NuttX thread mode, where PSP may be the
   * active stack.  A reset starts with MSP selected, so recreate that state
   * before entering the application's reset handler.
   */

  __asm__ volatile
    (
      "movs r2, #0\n\t"
      "msr control, r2\n\t"
      "isb\n\t"
      "msr msp, %0\n\t"
      "msr psp, r2\n\t"
      "bx %1\n\t"
      :
      : "r" (msp), "r" (reset)
      : "r2", "memory"
    );

  __builtin_unreachable();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bootloader_main
 *
 * Description:
 *   Initial NuttX application for the SRAM2-resident bootloader image.
 *
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  UNUSED(argc);
  UNUSED(argv);
  _alert("start");
  stm32_boot_nsh_xspi();
}
