/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/scripts/flash.ld
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

/* Layout for CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI.
 *
 * The STM32N657X0 has no internal flash.  This script builds an image that
 * is *stored* in the external Octo-SPI NOR flash attached to XSPI2 (memory
 * mapped at 0x70000000) and that *runs* from AXISRAM.
 *
 * Boot flow:
 *
 *   1. The boot ROM (BOOT pins selecting boot from external flash) puts
 *      XSPI2 into memory-mapped read mode, validates the STM32 header
 *      stored at 0x70000000 and, because that header declares a load
 *      address of 0xffffffff ("execute in place, do not relocate"),
 *      branches straight to the entry point it carries, i.e. to
 *      _flash_start at 0x70000400.
 *   2. _flash_start (see src/stm32_xipboot.S) executes in place from the
 *      memory-mapped flash.  It copies the whole loadable image (.text,
 *      .rodata, .init_section, the ARM unwind tables and .data) from its
 *      load address in flash to its link address in SRAM and then branches
 *      to the reset handler in SRAM.
 *   3. From that point on, everything runs out of SRAM exactly as it does
 *      in the SRAM-only DEV boot mode.
 *
 * The header is *not* emitted by the linker; it is prepended by
 * STM32_SigningTool_CLI when the image is packaged (see tools/mkimage.sh,
 * which runs automatically as a POSTBUILD step).  That is why the flash
 * region starts at 0x70000400 and not at 0x70000000: the linker only lays
 * out the payload that follows the header.  The 0x400 offset is what the
 * signing tool produces for header v2.3 with the "-align" option; do not
 * change one without the other.
 *
 * The first 1 KiB of SRAM (0x34000000..0x340003ff) stays reserved for the
 * boot ROM header/context, hence the SRAM origin at 0x34000400.
 *
 * See scripts/sram.ld for the DEV-boot / SRAM-only alternative.
 */

__stm32_header_size = 0x400;
__xspi2_base        = 0x70000000;

MEMORY
{
    /* External Octo-SPI NOR on XSPI2 (MX25UM51245G, 512 Mbit = 64 MiB on the
     * NUCLEO-N657X0-Q), minus the STM32 boot header.  Adjust LENGTH if the
     * board is populated with a different sized device.
     */

    flash (rx) : ORIGIN = 0x70000400, LENGTH = 64M - 0x400

    /* AXI SRAM2, minus the 1 KiB boot ROM header area */
    sram (rwx) : ORIGIN = 0x34180400, LENGTH = 511k
}

OUTPUT_ARCH(arm)

/* Entry is the in-flash loader stub, not _stext */

ENTRY(_flash_start)
EXTERN(_flash_start)

SECTIONS
{
    /* The XIP loader stub.  This is the only code that executes from the
     * memory-mapped flash, so it is linked in flash as well.
     */

    .xipboot : {
        _sxipboot = ABSOLUTE(.);
        KEEP(*(.xipboot))
        . = ALIGN(16);
        _exipboot = ABSOLUTE(.);
    } > flash

    /* Everything below is linked to run from SRAM but is loaded into
     * flash (VMA in sram, LMA in flash).
     *
     * Each of these output sections is padded to a 16-byte boundary so that
     * the gap between two consecutive sections is identical in flash and in
     * SRAM.  That keeps the whole image contiguous in both views and lets
     * the loader stub relocate it with a single copy loop.  The ASSERT at
     * the end of this script verifies the invariant.
     */

    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
        . = ALIGN(16);
    } > sram AT > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
        . = ALIGN(16);
    } > sram AT > flash

    .ARM.extab : {
        *(.ARM.extab*)
        . = ALIGN(16);
    } > sram AT > flash

    .ARM.exidx : {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
        . = ALIGN(16);
    } > sram AT > flash

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
        . = ALIGN(16);
    } > sram AT > flash

    /* Load address of .data, used by __start_c() to re-initialise .data
     * straight from flash.
     */

    _eronly = LOADADDR(.data);

    .bss (NOLOAD) : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    /* Symbols consumed by the loader stub in src/stm32_xipboot.S */

    _sflashimg = LOADADDR(.text);           /* Start of the image in flash */
    _sramimg   = ADDR(.text);               /* Where it has to be copied   */
    _eramimg   = ADDR(.data) + SIZEOF(.data);

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}

/* The loader stub relocates .text .. .data with one copy loop.  That is only
 * correct as long as the flash image and the SRAM image use the same layout,
 * i.e. as long as the LMA/VMA delta is the same for the first and the last
 * relocated section.
 */

ASSERT(LOADADDR(.data) - ADDR(.data) == LOADADDR(.text) - ADDR(.text),
       "flash.ld: flash image is not contiguous; LMA/VMA delta mismatch")
