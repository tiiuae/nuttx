/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/scripts/bl_flash.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* Layout for CONFIG_NUCLEO_N657X0_Q_BOOTLOADER.
 *
 * The STM32N6 boot ROM reads the signed payload from XSPI2 flash at
 * 0x70000000, copies it to AXI SRAM2 at 0x34180000, and branches to the
 * payload entry point.  The first 1 KiB is reserved for the ROM context, so
 * the NuttX image is linked to begin at 0x34180400.
 *
 * The STM32 header is prepended after linking by tools/mkimage.sh.  It is not
 * part of this ELF or raw binary: the payload is a contiguous SRAM image.
 */

MEMORY
{
    sram2 (rwx) : ORIGIN = 0x34180400, LENGTH = 511K
}

OUTPUT_ARCH(arm)
ENTRY(_stext)

SECTIONS
{
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
    } > sram2

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > sram2

    .ARM.extab : {
        *(.ARM.extab*)
    } > sram2

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > sram2
    __exidx_end = ABSOLUTE(.);

    .data : {
        /* The image is already copied to SRAM2 by the boot ROM. */
        _eronly = ABSOLUTE(.);
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram2

    .bss (NOLOAD) : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram2

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
