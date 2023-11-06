/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_wdog.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_WDOG_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_WDOG_H

/*****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>
#include "hardware/mpfs_wdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_WDOG_NUM_INSTANCES (5)
#ifndef __ASSEMBLY__


#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
EXTERN int mpfs_wdog_start(unsigned int instance);
EXTERN int mpfs_wdog_getcounter(unsigned int instance);
EXTERN int mpfs_wdog_settrigger(unsigned int instance, unsigned int value);
EXTERN int mpfs_wdog_setmsvp(unsigned int instance, unsigned int value);
EXTERN int mpfs_wdog_settimeout(unsigned int instance, unsigned int value);
EXTERN int mpfs_wdog_keepalive(unsigned int instance);
EXTERN void mpfs_wdog_immediate_reset(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_WDOG_H */
