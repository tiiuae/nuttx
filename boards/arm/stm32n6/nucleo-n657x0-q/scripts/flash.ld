/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/scripts/flash.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* Layout for CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI.
 *
 * This is the second-stage NuttX application.  A signed SRAM2-resident
 * bootloader occupies the first MiB of the XSPI2 NOR flash at 0x70000000.
 * This image, including its STM32 boot header, is programmed at 0x70100000.
 *
 * The bootloader configures XSPI2 for 200 MHz memory-mapped reads and enters
 * the reset handler from this image's vector table.  Consequently .text and
 * read-only data execute directly from flash.  Only writable data is loaded
 * into AXI SRAM; stm32_start.c copies it from its flash load address before
 * clearing .bss.
 *
 * STM32_SigningTool_CLI prepends the 0x400-byte v2.3 boot header during the
 * POSTBUILD step.  The linker lays out the payload after that header, hence
 * the flash origin at 0x70100400.
 */

__stm32_header_size = 0x400;
__xspi2_app_base    = 0x70100000;

MEMORY
{
    /* MX25UM51245G on XSPI2: 64 MiB total.  The first MiB is reserved for
     * the bootloader image and its STM32 header.
     */

    flash (rx)  : ORIGIN = 0x70100400, LENGTH = 63M - 0x400

    /* AXI SRAM, excluding the 1 KiB boot-ROM context area. */

    sram (rwx)  : ORIGIN = 0x34180400, LENGTH = 4095K
}

OUTPUT_ARCH(arm)
ENTRY(_stext)

SECTIONS
{
    /* All executable and read-only content remains in the XSPI2 memory
     * mapped window and is therefore executed in place.
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
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
        . = ALIGN(16);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
        . = ALIGN(16);
    } > flash

    .ARM.exidx : {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
        . = ALIGN(16);
    } > flash

    /* Writable initial values follow the read-only flash content in the raw
     * payload, but their virtual address is AXI SRAM.
     */

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    _eronly = LOADADDR(.data);

    .bss (NOLOAD) : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

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
