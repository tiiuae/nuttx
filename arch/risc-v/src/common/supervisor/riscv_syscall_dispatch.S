/****************************************************************************
 * arch/risc-v/src/common/supervisor/riscv_syscall_dispatch.S
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

.file "riscv_syscall_dispatch.S"

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

    .globl  riscv_syscall_dispatch

/****************************************************************************
 * Name: riscv_syscall_dispatch
 *
 * Description:
 *   Dispatch syscall from kernel, return to user mode
 *
 * C Function Prototype:
 *   void riscv_syscall_dispatch(void);
 *
 * Input Parameters:
 *   Assumes the context to return is already set up
 *
 * Returned Value:
 *   Return value of system call is returned into contex
 *
 * Assumptions:
 *   User task is running in privileged mode with interrupts masked,
 *   ready to resume the user task in unprivileged mode with interrupts
 *   restored
 *
 ****************************************************************************/

.type riscv_syscall_dispatch, function

riscv_syscall_dispatch:

  addi       sp, sp, -XCPTCONTEXT_SIZE /* make room */
  save_ctx   sp                        /* save current context */

  REGSTORE   x1, REG_EPC(sp)           /* save ra to epc */

  addi       s0, sp, XCPTCONTEXT_SIZE
  REGSTORE   s0, REG_SP(sp)            /* original SP */

  /* Set previous privilege, we are in privileged mode now */

  csrr       s0, CSR_STATUS            /* read status register */
  li         s1, STATUS_PPP            /* set previous privilege */
  or         s0, s0, s1
  li         s1, ~STATUS_PIE           /* clear previous interrupt enable */
  and        s0, s0, s1
  REGSTORE   s0, REG_INT_CTX(sp)       /* store status to context */

#ifdef CONFIG_ARCH_FPU
  mv         a0, sp
  jal        x1, riscv_savefpu         /* FP registers */
#endif

  mv         a0, sp                    /* a0 = context */
  jal        x1, riscv_handle_syscall  /* run the syscall */

  j          riscv_fullcontextrestore  /* resume the user task */
