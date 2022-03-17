/****************************************************************************
 * arch/risc-v/src/common/supervisor/riscv_context.S
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

.file "riscv_context.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/mode.h>

#include "riscv_exception_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .globl  riscv_saveusercontext
    .globl  riscv_fullcontextrestore
    .globl  riscv_switchcontext
    .globl  riscv_syscall_return

/****************************************************************************
 * Name: riscv_saveusercontext
 *
 * Description:
 *   Save user context, partially destroys the caller's context
 *
 * C Function Prototype:
 *   int riscv_saveusercontext(uintptr_t *saveregs);
 *
 * Input Parameters:
 *   saveregs - Context to save
 *
 * Returned Value:
 *   0 on context switch
 *   1 on no context switch
 *
 * Assumptions:
 *   Global interrupts disabled by the caller.
 *
 ****************************************************************************/

.type riscv_saveusercontext, function

riscv_saveusercontext:

  save_ctx   a0                        /* save context */

  csrr       s0, CSR_STATUS
  REGSTORE   s0, REG_INT_CTX(a0)       /* save status */

  REGSTORE   x1, REG_EPC(a0)           /* save ra to epc */
  REGSTORE   sp, REG_SP(a0)            /* original SP */

#ifdef CONFIG_ARCH_FPU
  jal        x1, riscv_savefpu         /* FP registers */
  REGLOAD    x1, REG_X1(a0)            /* restore ra */
#endif

  li         s0, 1                     /* return 1 (in context) */
  REGSTORE   s0, REG_A0(a0)
  REGLOAD    s0, REG_S0(a0)            /* restore s0 */

  li         a0, 0                     /* return 0 (back to caller) */
  ret

/****************************************************************************
 * Name: riscv_fullcontextrestore
 *
 * Description:
 *   Restore context
 *
 * C Function Prototype:
 *   void riscv_fullcontextrestore(uintptr_t *restoreregs) noreturn_function;
 *
 * Input Parameters:
 *   restoreregs - Context to restore
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts disabled by the caller.
 *
 ****************************************************************************/

.type riscv_fullcontextrestore, function

riscv_fullcontextrestore:

#ifdef CONFIG_ARCH_FPU
  jal        x1, riscv_restorefpu      /* FP registers */
#endif

  mv         sp, a0                    /* use sp, as a0 gets wiped */

  REGLOAD    s0, REG_EPC(sp)           /* restore epc */
  csrw       CSR_EPC, s0

  /* Restore status register, but don't enable interrupts yet */

  REGLOAD    s0, REG_INT_CTX(sp)       /* restore status */
  li         s1, STATUS_IE             /* move IE -> PIE */
  and        s1, s0, s1                /* if (STATUS & IE) */
  beqz       s1, 1f
  li         s1, ~STATUS_IE            /* clear IE */
  and        s0, s0, s1
  li         s1, STATUS_PIE            /* set PIE */
  or         s0, s0, s1

1:
  csrw       CSR_STATUS, s0

  load_ctx   sp

  REGLOAD    sp, REG_SP(sp)            /* restore original sp */

  /* return from exception, which updates the status register */

  sret

/****************************************************************************
 * Name: riscv_switchcontext
 *
 * Description:
 *   Restore user context
 *
 * C Function Prototype:
 *   void riscv_switchcontext(uintptr_t *saveregs, uintptr_t *restoreregs);
 *
 * Input Parameters:
 *   saveregs    - Context to save
 *   restoreregs - Context to restore
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts disabled by the caller.
 *
 ****************************************************************************/

.type riscv_switchcontext, function

riscv_switchcontext:

  /* Save old context to arg[0] */

  save_ctx   a0                        /* save context */

  REGSTORE   x1, REG_EPC(a0)           /* save ra to epc */
  REGSTORE   sp, REG_SP(a0)            /* original SP */

  /* Set previous privilege, we are in privileged mode now */

  csrr       s0, CSR_STATUS            /* read status register */
  li         s1, STATUS_PPP            /* set previous privilege */
  or         s0, s0, s1
  li         s1, ~STATUS_PIE           /* clear previous interrupt enable */
  and        s0, s0, s1
  REGSTORE   s0, REG_INT_CTX(a0)       /* store status to context */

#ifdef CONFIG_ARCH_FPU
  jal        x1, riscv_savefpu         /* FP registers */
#endif

  /* Load new context from arg[1] */

  mv         a0, a1                    /* load from a1 */
  j          riscv_fullcontextrestore  /* restore context */

/****************************************************************************
 * Name: riscv_syscall_return
 *
 * Description:
 *   Return from system call to user task with user mode privileges
 *
 * C Function Prototype:
 *   void riscv_syscall_return(void);
 *
 * Input Parameters:
 *   Assumes the return value of the system call is in a0
 *
 * Returned Value:
 *   Return value of system call is returned into contex
 *
 * Assumptions:
 *   User task is running system call in privileged mode, ready to resume
 *   the user task in unprivileged mode
 *
 ****************************************************************************/

.type riscv_syscall_return, function

riscv_syscall_return:

  addi       sp, sp, -XCPTCONTEXT_SIZE    /* make room */
  save_ctx   sp                           /* save current context */

  /* Mask interrupts here, they will be re-enabled later */

  li         s0, STATUS_IE
  csrc       CSR_STATUS, s0

  addi       s0, sp, XCPTCONTEXT_SIZE
  REGSTORE   s0, REG_SP(sp)               /* original SP */

  mv         a0, sp                       /* a0 = context */
  jal        x1, riscv_handle_syscall     /* run the exit function */

  j          riscv_fullcontextrestore     /* resume the user task */
