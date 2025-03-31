/****************************************************************************
 * sched/semaphore/semaphore.h
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

#ifndef __SCHED_SEMAPHORE_SEMAPHORE_H
#define __SCHED_SEMAPHORE_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/atomic.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Common semaphore logic */

#ifdef CONFIG_PRIORITY_INHERITANCE
void nxsem_initialize(void);
#else
#  define nxsem_initialize()
#endif

/* Wake up a thread that is waiting on semaphore */

void nxsem_wait_irq(FAR struct tcb_s *wtcb, int errcode);

/* Handle semaphore timer expiration */

void nxsem_timeout(wdparm_t arg);

/* Recover semaphore resources with a task or thread is destroyed */

void nxsem_recover(FAR struct tcb_s *tcb);

/* Special logic needed only by priority inheritance to manage collections of
 * holders of semaphores.
 */

#ifdef CONFIG_PRIORITY_INHERITANCE
void nxsem_initialize_holders(void);
void nxsem_destroyholder(FAR sem_t *sem);
void nxsem_add_holder(FAR sem_t *sem);
void nxsem_add_holder_tcb(FAR struct tcb_s *htcb, FAR sem_t *sem);
void nxsem_boost_priority(FAR sem_t *sem);
void nxsem_release_holder(FAR sem_t *sem);
void nxsem_restore_baseprio(FAR struct tcb_s *stcb, FAR sem_t *sem);
void nxsem_canceled(FAR struct tcb_s *stcb, FAR sem_t *sem);
void nxsem_release_all(FAR struct tcb_s *stcb);
#else
#  define nxsem_initialize_holders()
#  define nxsem_destroyholder(sem)
#  define nxsem_add_holder(sem)
#  define nxsem_add_holder_tcb(htcb,sem)
#  define nxsem_boost_priority(sem)
#  define nxsem_release_holder(sem)
#  define nxsem_restore_baseprio(stcb,sem)
#  define nxsem_canceled(stcb,sem)
#  define nxsem_release_all(stcb)
#endif

/* Special logic needed only by priority protect */

#ifdef CONFIG_PRIORITY_PROTECT
int nxsem_protect_wait(FAR sem_t *sem);
void nxsem_protect_post(FAR sem_t *sem);
#else
#  define nxsem_protect_wait(sem)  0
#  define nxsem_protect_post(sem)
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_get_mholder_reserve
 *
 * Description:
 *   Reads the mholder value and temporarily locks it by setting the
 *   blocking bit. Note that blocking bit must be reset to correct value
 *   when releasing the atomic.
 *
 * Input Parameters:
 *   sem  - Semaphore descriptor
 *
 * Returned Value:
 *   mutex holder value before the blocking bit is set
 *
 * Assumptions:
 *   The semaphore structure is locked
 *
 ****************************************************************************/

static inline uint32_t nxsem_get_mholder_reserve(FAR sem_t *sem)
{
  uint32_t mholder = atomic_read(NXSEM_MHOLDER(sem));
  do
    {
    }
  while (!atomic_try_cmpxchg_acquire(NXSEM_MHOLDER(sem), &mholder,
                                     mholder | NXSEM_MBLOCKS_BIT));

  return mholder;
}

/****************************************************************************
 * Name: nxsem_set_mholder
 *
 * Description:
 *   Constructs a mutex holder word with the given tcb's TID and sets the
 *   blocking bit according to semaphore wait queue.
 *
 * Input Parameters:
 *   htcb - Holder task's tcb
 *   sem  - Semaphore descriptor
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The semaphore structure is locked
 *
 ****************************************************************************/

static inline void nxsem_set_mholder(FAR struct tcb_s *htcb, FAR sem_t *sem)
{
  uint32_t mholder;

  if (htcb)
    {
      uint32_t blocks = dq_peek(SEM_WAITLIST(sem)) == NULL ? 0 :
        NXSEM_MBLOCKS_BIT;

      mholder = ((uint32_t)htcb->pid) | blocks;
    }
  else
    {
      mholder = NXSEM_NO_MHOLDER;
    }

  atomic_set_release(NXSEM_MHOLDER(sem), mholder);
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_SEMAPHORE_SEMAPHORE_H */
