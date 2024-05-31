/****************************************************************************
 * arch/risc-v/src/common/supervisor/riscv_syscall.S
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

.file "riscv_syscall.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/mode.h>

#include "chip.h"

#include "riscv_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .globl  sys_call0
    .globl  sys_call1
    .globl  sys_call2
    .globl  sys_call3
    .globl  sys_call4
    .globl  sys_call5
    .globl  sys_call6

/****************************************************************************
 * Name: sys_callx
 *
 * Description:
 *   Dispatch syscall from kernel
 *
 * C Function Prototype:
 *   uintptr_t sys_callx(unsigned int nbr, ...);
 *
 * Input Parameters:
 *   Assumes the context to return is already set up
 *
 * Returned Value:
 *   Return value of system call is returned into contex
 *
 * Assumptions:
 *   Task is running in privileged mode
 *
 ****************************************************************************/

.type sys_call0, function
.type sys_call1, function
.type sys_call2, function
.type sys_call3, function
.type sys_call4, function
.type sys_call5, function
.type sys_call6, function

sys_call0:
sys_call1:
sys_call2:
sys_call3:
sys_call4:
sys_call5:
sys_call6:

  addi       sp, sp, -XCPTCONTEXT_SIZE /* make room */
  save_ctx   sp                        /* save current context */

  /* Mask interrupts and store the status register to context */

  li         s1, STATUS_IE             /* move IE -> PIE */
  csrrc      s0, CSR_STATUS, s1
  and        s1, s0, s1                /* if (STATUS & IE) */
  beqz       s1, 1f
  li         s1, ~STATUS_IE            /* clear IE */
  and        s0, s0, s1
  li         s1, STATUS_PIE            /* set PIE */
  or         s0, s0, s1

 1:
  /* Set previous privilege, we are in privileged mode now */

  li         s1, STATUS_PPP            /* set previous privilege */
  or         s0, s0, s1
  REGSTORE   s0, REG_INT_CTX(sp)       /* store status to context */

  REGSTORE   x1, REG_EPC(sp)           /* save ra to epc */

  addi       s0, sp, XCPTCONTEXT_SIZE
  REGSTORE   s0, REG_SP(sp)            /* original SP */

  mv         a0, sp                    /* a0 = context */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Switch to interrupt stack */

  setintstack t0, t1
#endif

  /* Run the handler */

  la         x1, return_from_exception
  tail       riscv_perform_syscall

  .size sys_call0, .-sys_call0
  .size sys_call1, .-sys_call1
  .size sys_call2, .-sys_call2
  .size sys_call3, .-sys_call3
  .size sys_call4, .-sys_call4
  .size sys_call5, .-sys_call5
  .size sys_call6, .-sys_call6

  #.end

.globl switch_context

switch_context:

  /* Create a stack frame */

  addi        sp, sp, -XCPTCONTEXT_SIZE
  REGSTORE    sp, 0(a0)           /* set tcb->xcp.regs = context */
  mv          a0, sp
  mv          tp, a2

  /* Save the CPU status */

  csrr        a2, CSR_STATUS
  li          a3, STATUS_PPP      /* set previous privilege */
  or          a2, a2, a3
  REGSTORE    a2, REG_INT_CTX(a0)

  REGSTORE    x1, REG_EPC(a0)     /* save ra to epc */

  /* Save the callee registers */

  REGSTORE    s0,  REG_S0(a0)
  REGSTORE    s1,  REG_S1(a0)
  REGSTORE    s2,  REG_S2(a0)
  REGSTORE    s3,  REG_S3(a0)
  REGSTORE    s4,  REG_S4(a0)
  REGSTORE    s5,  REG_S5(a0)
  REGSTORE    s6,  REG_S6(a0)
  REGSTORE    s7,  REG_S7(a0)
  REGSTORE    s8,  REG_S8(a0)
  REGSTORE    s9,  REG_S9(a0)
  REGSTORE    s10, REG_S10(a0)
  REGSTORE    s11, REG_S11(a0)

  addi        a2, sp, XCPTCONTEXT_SIZE
  REGSTORE    a2, REG_SP(a0)
  REGSTORE    x1, REG_RA(a0)

  /* Load the callee registers */

  REGLOAD     s0,  REG_S0(a1)
  REGLOAD     s1,  REG_S1(a1)
  REGLOAD     s2,  REG_S2(a1)
  REGLOAD     s3,  REG_S3(a1)
  REGLOAD     s4,  REG_S4(a1)
  REGLOAD     s5,  REG_S5(a1)
  REGLOAD     s6,  REG_S6(a1)
  REGLOAD     s7,  REG_S7(a1)
  REGLOAD     s8,  REG_S8(a1)
  REGLOAD     s9,  REG_S9(a1)
  REGLOAD     s10, REG_S10(a1)
  REGLOAD     s11, REG_S11(a1)

  REGLOAD     sp,  REG_SP(a1)
  REGLOAD     x1,  REG_RA(a1)

  /* Load the CPU status */

  REGLOAD     a2, REG_INT_CTX(a1)
  csrw        CSR_STATUS, a2

  /* Return to (new) context */

  ret
