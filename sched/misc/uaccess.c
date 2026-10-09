/****************************************************************************
 * sched/misc/uaccess.c
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

#include <debug.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#ifdef CONFIG_BUILD_PROTECTED
#  include <nuttx/mm/mm.h>
#  include <nuttx/userspace.h>
#endif

#include "sched/sched.h"

#if defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_BUILD_PROTECTED)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
static bool uaccess_within(uintptr_t start, uintptr_t end,
                           uintptr_t low, uintptr_t high)
{
  return start >= low && end <= high;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uaccess_ok
 *
 * Description:
 *   Check that a range lies wholly in the memory a user process may reach.
 *
 ****************************************************************************/

bool uaccess_ok(FAR const void *ptr, size_t len)
{
#ifdef CONFIG_BUILD_KERNEL
  uintptr_t start = (uintptr_t)ptr;
  uintptr_t end = start + len - 1;

  return up_addrenv_user_vaddr(start) &&
         (len == 0 || (end >= start && up_addrenv_user_vaddr(end)));
#else
  uintptr_t start = (uintptr_t)ptr;
  uintptr_t end = start + (len > 0 ? len : 1);

  /* A length that wraps names memory the caller never owns. */

  if (end < start)
    {
      return false;
    }

  /* One address space, so the regions are fixed: user text and rodata,
   * user data and bss, and the user heap the stacks come from. The kernel
   * heap sits between user bss and the user heap, hence the whole range
   * has to fall inside one of them.
   */

  return uaccess_within(start, end, USERSPACE->us_textstart,
                        USERSPACE->us_textend) ||
         uaccess_within(start, end, USERSPACE->us_datastart,
                        USERSPACE->us_bssend) ||
         umm_heaprange(ptr, len > 0 ? len : 1);
#endif
}

/****************************************************************************
 * Name: uaccess_nested
 *
 * Description:
 *   Check a pointer read out of a user structure. A kernel caller passes a
 *   kernel parent, and its children need no check.
 *
 ****************************************************************************/

bool uaccess_nested(FAR const void *parent, FAR const void *ptr)
{
#ifdef CONFIG_BUILD_KERNEL
  return !up_addrenv_user_vaddr((uintptr_t)parent) || uaccess_ok(ptr, 1);
#else
  return !uaccess_ok(parent, 1) || uaccess_ok(ptr, 1);
#endif
}

/****************************************************************************
 * Name: uaccess_check
 *
 * Description:
 *   End the calling process unless the range is its own memory.
 *
 ****************************************************************************/

void uaccess_check(FAR const void *ptr, size_t len)
{
  if (!uaccess_ok(ptr, len))
    {
      uaccess_fault(ptr);
    }
}

/****************************************************************************
 * Name: uaccess_fault
 *
 * Description:
 *   Report the address and end the process that named it.
 *
 ****************************************************************************/

void uaccess_fault(FAR const void *ptr)
{
  _alert("%s: %p is not user memory\n", get_task_name(this_task()), ptr);
  _exit(SIGSEGV);
}

/****************************************************************************
 * Name: uaccess_privileged
 *
 * Description:
 *   True if the task runs with kernel privileges, so the addresses it
 *   passes are its own.
 *
 ****************************************************************************/

bool uaccess_privileged(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_BUILD_KERNEL
  return tcb->addrenv_own == NULL;
#else
  return tcb->group == NULL ||
         (tcb->group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0;
#endif
}

#endif /* CONFIG_BUILD_KERNEL || CONFIG_BUILD_PROTECTED */
