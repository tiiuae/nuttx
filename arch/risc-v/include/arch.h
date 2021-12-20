/****************************************************************************
 * arch/risc-v/include/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_RISCV_INCLUDE_ARCH_H
#define __ARCH_RISCV_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

#include <arch/csr.h>

#ifdef CONFIG_ARCH_RV32IM
#  include <arch/rv32im/arch.h>
#endif

#ifdef CONFIG_ARCH_RV64GC
#  include <arch/rv64gc/arch.h>
#endif

#ifdef CONFIG_ARCH_ADDRENV
#  include "../src/rv64gc/riscv_mmu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to get the core and vendor ID, HART, arch and ISA codes, etc.
 */

#ifdef CONFIG_RV32IM_SYSTEM_CSRRS_SUPPORT

uint32_t up_getmisa(void);
uint32_t up_getarchid(void);
uint32_t up_getimpid(void);
uint32_t up_getvendorid(void);
uint32_t up_gethartid(void);

#else

#define up_getmisa() 0
#define up_getarchid() 0
#define up_getimpid() 0
#define up_getvendorid() 0
#define up_gethartid() 0

#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV

/* A task group must have its L1 table in memory always, and the rest can
 * be dynamically committed to memory (and even swapped).
 *
 * In this implementation every level tables besides the final level N are
 * kept in memory always, while the level N tables are dynamically allocated.
 *
 * The motivation ?
 *
 * This does consume more memory statically compared to dynamic level N
 * allocation, but it makes context switches fast, especially when the
 * risc-v kernel runs in machine mode as the address translation tables do
 * not need to be switched nor the TLB flushed when running kernel code.
 *
 * The implications ? They depend on the MMU type.
 *
 * For Sv39 this means that:
 * - A task can not have more than 1GB of memory allocated. This should be
 *   plenty enough...
 * - The minimum amount of memory needed for page tables per task is 12K,
 *   which gives access to 2MB of memory. This is plenty for many tasks.
 */

struct group_addrenv_s
{
  /* Pointers to LN-1 tables here, one of each are allocated for the task
   * when it is created.
   */

  uint64_t *spgtables[RV_MMU_PT_LEVELS - 1];

  /* For convenience store the data base here */

  uint64_t datavbase;

  /* For convenience store the heap base and initial size here */

  uint64_t heapvbase;
  uint64_t heapsize;

  /* For convenience store the satp value here */

  uint64_t satp;
};

typedef struct group_addrenv_s group_addrenv_t;

/* If an address environment needs to be saved, saving the satp register
 * will suffice. The register width is architecture dependent
 */

typedef uint64_t save_addrenv_t;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RV32IM_HW_MULDIV
uint32_t up_hard_mul(uint32_t a, uint32_t b);
uint32_t up_hard_mulh(uint32_t a, uint32_t b);
uint32_t up_hard_mulhsu(uint32_t a, uint32_t b);
uint32_t up_hard_mulhu(uint32_t a, uint32_t b);
uint32_t up_hard_div(uint32_t a, uint32_t b);
uint32_t up_hard_rem(uint32_t a, uint32_t b);
uint32_t up_hard_divu(uint32_t a, uint32_t b);
uint32_t up_hard_remu(uint32_t a, uint32_t b);
uint32_t time_hard_mul(uint32_t a, uint32_t b, uint32_t *t);
#endif

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ARCH_H */
