/****************************************************************************
 * arch/risc-v/src/rv64gc/riscv_pgalloc.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include "riscv_mmu.h"

#ifdef CONFIG_BUILD_KERNEL

#define PGT_LAST            RV_MMU_PT_LEVELS
#define MMU_UDATA_FLAGS     (PTE_R | PTE_W | PTE_U | PTE_G)

static int get_pgtable(FAR group_addrenv_t *addrenv, uintptr_t vaddr)
{
  uintptr_t     paddr;
  uintptr_t     ptentry;
  uint64_t      spgidx;

  /* Get the current level n-1 entry corresponding to this vaddr */

  spgidx = (vaddr >> (RV_MMU_VPN_WIDTH + RV_MMU_PAGE_SHIFT) & 0x1ff);

  ptentry = (uintptr_t)addrenv->spgtables[spgidx];
  if (ptentry == 0)
    {
      /* No page table has been allocated... allocate one now */

      paddr = mm_pgalloc(1);
      if (paddr != 0)
        {
          memset((void *)paddr, 0, MM_PGSIZE);
          ptentry = paddr | MMU_UDATA_FLAGS;
          addrenv->spgtables[spgidx] = (FAR uint64_t *)ptentry;
        }
    }

  return ((ptentry & RV_MMU_PTE_PPN_MASK) << RV_MMU_PTE_PPN_SHIFT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pgalloc
 *
 * Description:
 *   If there is a page allocator in the configuration and if and MMU is
 *   available to map physical addresses to virtual address, then function
 *   must be provided by the platform-specific code.  This is part of the
 *   implementation of sbrk().  This function will allocate the requested
 *   number of pages using the page allocator and map them into consecutive
 *   virtual addresses beginning with 'brkaddr'
 *
 *   NOTE:  This function does not use the up_ naming standard because it
 *   is indirectly callable from user-space code via a system trap.
 *   Therefore, it is a system interface and follows a different naming
 *   convention.
 *
 * Input Parameters:
 *   brkaddr - The heap break address.  The next page will be allocated and
 *     mapped to this address.  Must be page aligned.  If the memory manager
 *     has not yet been initialized and this is the first block requested for
 *     the heap, then brkaddr should be zero.  pgalloc will then assigned the
 *     well-known virtual address of the beginning of the heap.
 *   npages - The number of pages to allocate and map.  Mapping of pages
 *     will be contiguous beginning beginning at 'brkaddr'
 *
 * Returned Value:
 *   The (virtual) base address of the mapped page will returned on success.
 *   Normally this will be the same as the 'brkaddr' input. However, if
 *   the 'brkaddr' input was zero, this will be the virtual address of the
 *   beginning of the heap.  Zero is returned on any failure.
 *
 ****************************************************************************/

uintptr_t pgalloc(uintptr_t brkaddr, unsigned int npages)
{
  FAR struct tcb_s          *tcb = nxsched_self();
  FAR struct task_group_s   *group;
  uintptr_t                 lntable;
  uintptr_t                 paddr;

  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;

  /* The current implementation only supports extending the user heap
   * region as part of the implementation of user sbrk().  This function
   * needs to be expanded to also handle (1) extending the user stack
   * space and (2) extending the kernel memory regions as well.
   */

  DEBUGASSERT((group->tg_flags & GROUP_FLAG_ADDRENV) != 0);

  /* brkaddr = 0 means that no heap has yet been allocated */

  if (brkaddr == 0)
    {
      brkaddr = group->tg_addrenv.heapvbase;
    }

  /* Sanity checks */

  DEBUGASSERT(brkaddr >= group->tg_addrenv.heapvbase);
  DEBUGASSERT(brkaddr < group->tg_addrenv.heapvbase +
                        group->tg_addrenv.heapsize);
  DEBUGASSERT(MM_ISALIGNED(brkaddr));

  for (; npages > 0; npages--)
    {
      /* Get the physical address of the level n page table */

      lntable = get_pgtable(&group->tg_addrenv, brkaddr);
      if (lntable == 0)
        {
          return 0;
        }

      /* Allocate physical memory for the new heap */

      paddr = mm_pgalloc(1);
      if (paddr == 0)
        {
          return 0;
        }

      /* Jawohl */

      mmu_ln_setentry(PGT_LAST, lntable, paddr, brkaddr, MMU_UDATA_FLAGS);
      brkaddr += MM_PGSIZE;
    }

  return brkaddr;
}

#endif /* CONFIG_BUILD_KERNEL */
