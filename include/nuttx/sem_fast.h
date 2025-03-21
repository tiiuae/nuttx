/****************************************************************************
 * include/nuttx/sem_fast.h
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

#ifndef __INCLUDE_NUTTX_SEM_FAST_H
#define __INCLUDE_NUTTX_SEM_FAST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/atomic.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Mutex fast path using atomics is not relevant when using
 * CONFIG_LIBC_ARCH_ATOMIC, since libc atomic implementation uses spinlocks.
 * Spinlocks should not be called from libc, and also in the kernel it is
 * faster to take the slow path directly.
 */

#ifndef CONFIG_LIBC_ARCH_ATOMIC

/****************************************************************************
 * Name: nxsem_trywait_fast
 *
 * Description:
 *   Try to wait on mutex using fast atomic exchange
 *
 * Input Parameters:
 *   sem - The semaphore to wait on
 *
 * Returned Value:
 *   OK      - Mutex taken
 *   -EAGAIN - Mutex not taken (already locked)
 *   -EPERM  - Parameter "sem" is not a mutex, or it enables features
 *             which prevent fast locking sucn as priority adjust
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

static inline_function int nxsem_trywait_fast(sem_t *sem)
{
  int ret = -EPERM;
  if ((sem->flags & SEM_TYPE_MUTEX)
#if defined(CONFIG_PRIORITY_PROTECT) || defined(CONFIG_PRIORITY_INHERITANCE)
      && (sem->flags & (SEM_TYPE_MUTEX | SEM_PRIO_MASK)) ==
      (SEM_TYPE_MUTEX | SEM_PRIO_NONE)
#endif
      )
    {
      int32_t old = 1;
      ret = atomic_try_cmpxchg_acquire(NXSEM_COUNT(sem), &old, 0) ?
        OK : -EAGAIN;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsem_post_fast
 *
 * Description:
 *   Try to release a mutex using fast atomic exchange
 *
 * Input Parameters:
 *   sem - The semaphore to post
 *
 * Returned Value:
 *   OK      - Mutex released
 *   -EPERM  - Parameter "sem" is not a mutex, or it enables features
 *             which prevent fast release sucn as priority adjust
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

static inline_function int nxsem_post_fast(sem_t *sem)
{
  int ret = -EPERM;
  if ((sem->flags & SEM_TYPE_MUTEX)
#if defined(CONFIG_PRIORITY_PROTECT) || defined(CONFIG_PRIORITY_INHERITANCE)
      && (sem->flags & (SEM_TYPE_MUTEX | SEM_PRIO_MASK)) ==
      (SEM_TYPE_MUTEX | SEM_PRIO_NONE)
#endif
      )
    {
      int32_t old = 0;
      if (atomic_try_cmpxchg_release(NXSEM_COUNT(sem), &old, 1))
        {
          ret = OK;
        }
    }

  return ret;
}

#else /* ifndef CONFIG_LIBC_ARCH_ATOMIC */

#define nxsem_trywait_fast(sem) -EPERM
#define nxsem_post_fast(sem) -EPERM

#endif /* ifndef CONFIG_LIBC_ARCH_ATOMIC */

#endif /* __INCLUDE_NUTTX_SEM_FAST_H */
