/****************************************************************************
 * arch/risc-v/src/rv64gc/addrenv.h
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

#ifndef __ARCH_RISC_V_SRC_RV64GC_ADDRENV_H_
#define __ARCH_RISC_V_SRC_RV64GC_ADDRENV_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Aligned size of the kernel stack */

#ifdef CONFIG_ARCH_KERNEL_STACK
#  define ARCH_KERNEL_STACKSIZE ((CONFIG_ARCH_KERNEL_STACKSIZE + 7) & ~7)
#endif

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __ARCH_RISC_V_SRC_RV64GC_ADDRENV_H_ */
