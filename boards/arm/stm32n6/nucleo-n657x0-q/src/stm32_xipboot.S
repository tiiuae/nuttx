/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_xipboot.S
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARMv8-M cache maintenance: invalidate the entire I-cache.  Writing this
 * register is harmless when the I-cache is disabled.
 */

#define NVIC_ICIALLU    0xe000ef50

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.syntax	unified
	.thumb
	.file	"stm32_xipboot.S"

/****************************************************************************
 * Name: _flash_start
 *
 * Description:
 *   Entry point of the image when it is stored in, and booted from, the
 *   external Octo-SPI NOR flash mapped at 0x70000000 (XSPI2).
 *
 *   This is the *only* code that runs in place from the memory-mapped
 *   flash.  It is entered by the boot ROM (or by a first stage boot loader)
 *   with XSPI2 already in memory-mapped read mode.  It copies the complete
 *   loadable image -- .text, .rodata, .init_section, the ARM unwind tables
 *   and .data -- from its load address in flash to its link address in
 *   AXISRAM and then branches to the reset handler, which from then on
 *   executes entirely out of SRAM.
 *
 *   The region boundaries are supplied by scripts/flash.ld:
 *
 *     _sflashimg - load address of the image (in flash)
 *     _sramimg   - link address of the image (in SRAM)
 *     _eramimg   - end of the image (in SRAM)
 *
 *   The copy is a plain word loop; flash.ld guarantees that both the source
 *   and the destination are word aligned and that the image is contiguous.
 *
 ****************************************************************************/

	.section	.xipboot, "ax", %progbits
	.align	2
	.globl	_flash_start
	.type	_flash_start, function

_flash_start:

	/* The boot ROM leaves MSPLIM/PSPLIM set to values that make the very
	 * first push from this stub fault.  Clear them before doing anything
	 * else.
	 */

	mov		r0, #0
	msr		msplim, r0
	msr		psplim, r0

	/* Copy the image from external flash to SRAM */

	ldr		r0, .Lsflashimg		/* r0: source (flash)      */
	ldr		r1, .Lsramimg		/* r1: destination (SRAM)  */
	ldr		r2, .Leramimg		/* r2: end of destination  */

.Lcopy:
	cmp		r1, r2
	bhs		.Lcopydone
	ldr		r3, [r0], #4
	str		r3, [r1], #4
	b		.Lcopy

.Lcopydone:

	/* Make the freshly written instructions visible to the I-side before
	 * jumping into them.
	 */

	dsb
	ldr		r0, .Liciallu
	mov		r1, #0
	str		r1, [r0]
	dsb
	isb

	/* Switch to the SRAM stack advertised by the relocated vector table
	 * and branch to its reset handler (__start).
	 */

	ldr		r0, .Lvectors
	ldr		r1, [r0]		/* Vector 0: initial MSP   */
	msr		msp, r1
	ldr		r2, [r0, #4]		/* Vector 1: reset handler */
	bx		r2

	/* Literal pool.  It lives in .xipboot, i.e. in flash, so the
	 * PC-relative loads above are valid before the copy has run.
	 */

	.align	2
.Lsflashimg:
	.word	_sflashimg
.Lsramimg:
	.word	_sramimg
.Leramimg:
	.word	_eramimg
.Lvectors:
	.word	_vectors
.Liciallu:
	.word	NVIC_ICIALLU

	.size	_flash_start, . - _flash_start
	.end
