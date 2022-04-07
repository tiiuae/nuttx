/****************************************************************************
 * mm/vm_map/vm_map.c
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
#include <stdbool.h>
#include <stddef.h>
#include <queue.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/vm_map.h>
#include <nuttx/kmalloc.h>
#include <sys/mman.h>
#include <assert.h>

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool vaddr_in_area(FAR const void *addr, FAR const void *start,
                          size_t length)
{
  uintptr_t u_addr = (uintptr_t)addr;
  uintptr_t u_start = (uintptr_t)start;
  return (u_addr >= u_start) && (u_addr < u_start + length);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vm_map_initialize
 *
 * Description:
 *   Allocates a task group specific vm_map stucture. Called when the group
 *   is initialized
 *
 ****************************************************************************/

void vm_map_initialize(FAR struct vm_map_s *mm)
{
  memset(mm, 0, sizeof(struct vm_map_s));
  sq_init(&mm->vm_map_sq);
  nxsem_init(&mm->vm_map_sem, 0, 1);
}

/****************************************************************************
 * Name: vm_map_destroy
 *
 * Description:
 *   De-allocates a task group specific vm_map stucture and the vm_map_sem
 *
 ****************************************************************************/

void vm_map_destroy(FAR struct vm_map_s *mm)
{
  FAR struct vm_map_entry_s *map =
    (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);
  while (map)
    {
      munmap((void *)map->vaddr, map->length);
      map = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);
    }

  nxsem_destroy(&mm->vm_map_sem);
}

/****************************************************************************
 * Name: vm_map_add
 *
 * Description:
 *   Add a mapping to task group's vm_map list
 *
 ****************************************************************************/

int vm_map_add(enum vm_map_type type, union vm_map_id_u id,
               FAR const void *vaddr, size_t length)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *map = kmm_zalloc(sizeof(struct vm_map_entry_s));
  int ret;

  if (!map)
    {
      return -ENOMEM;
    }

  map->type = type;
  map->id = id;
  map->vaddr = vaddr;
  map->length = length;

  ret = nxsem_wait(&mm->vm_map_sem);
  if (ret < 0)
    {
      kmm_free(map);
      return ret;
    }

  sq_addfirst((sq_entry_t *)map, &mm->vm_map_sq);

  nxsem_post(&mm->vm_map_sem);

  return OK;
}

/****************************************************************************
 * Name: vm_map_next
 *
 * Description:
 *   Returns the next mapping in the list.
 *
 ****************************************************************************/

FAR const struct vm_map_entry_s *vm_map_next(
                                 FAR const struct vm_map_entry_s *entry)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *map = NULL;

  if (nxsem_wait(&mm->vm_map_sem) == OK)
    {
      if (entry == NULL)
        {
          map = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);
        }
      else
        {
          map = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)entry));
        }

      nxsem_post(&mm->vm_map_sem);
    }

  return map;
}

/****************************************************************************
 * Name: vm_map_find_contains
 *
 * Description:
 *   Find the first mapping containing an address from the task group's list
 *
 ****************************************************************************/

FAR const struct vm_map_entry_s *vm_map_find_contains(FAR const void *vaddr)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *map = NULL;

  if (nxsem_wait(&mm->vm_map_sem) == OK)
    {
      map = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);

      while (map && !vaddr_in_area(vaddr, map->vaddr, map->length))
        {
          map = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)map));
        }

      nxsem_post(&mm->vm_map_sem);
    }

  return map;
}

/****************************************************************************
 * Name: vm_map_find
 *
 * Description:
 *   Find the first mapping matching address and length
 *
 ****************************************************************************/

FAR const struct vm_map_entry_s *vm_map_find(FAR const void *vaddr,
                                             size_t length)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *map = NULL;

  if (nxsem_wait(&mm->vm_map_sem) == OK)
    {
      map = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);

      while (map && vaddr != map->vaddr && length != map->length)
        {
          map = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)map));
        }

      nxsem_post(&mm->vm_map_sem);
    }

  return map;
}

/****************************************************************************
 * Name: vm_map_rm
 *
 * Description:
 *   Remove a mapping from the task  group's list
 *
 ****************************************************************************/

int vm_map_rm(FAR const struct vm_map_entry_s **map)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *prev;
  FAR struct vm_map_entry_s *r = NULL;

  int ret = nxsem_wait(&mm->vm_map_sem);
  if (ret < 0)
    {
      return ret;
    }

  prev = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);

  /* Check if the list was empty */

  if (!prev)
    {
      nxsem_post(&mm->vm_map_sem);
      return -ENOENT;
    }

  /* Check if removing the first item */

  if (*map == prev)
    {
      sq_remfirst(&mm->vm_map_sq);
      *map = NULL;
      r = prev;
    }
  else
    {
      /* Loop through the remaining items to find the one to be removed */

      while ((r = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)prev))))
        {
          if (*map == r)
            {
              sq_remafter((sq_entry_t *)prev, &mm->vm_map_sq);
              *map = prev;
              break;
            }

          prev = r;
        }
    }

  nxsem_post(&mm->vm_map_sem);

  /* If the item was removed, also delete the entry struct */

  if (r)
    {
      kmm_free(r);
      return OK;
    }

  return -ENOENT;
}

#endif /* defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__) */
