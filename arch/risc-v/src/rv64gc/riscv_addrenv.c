/****************************************************************************
 * arch/risc-v/src/rv64gc/riscv_addrenv.c
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
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on type group_addrenv_t which
 * is an abstract representation of a task group's address environment and
 * must be defined in arch/arch.h if CONFIG_ARCH_ADDRENV is defined.
 *
 *   up_addrenv_create   - Create an address environment
 *   up_addrenv_destroy  - Destroy an address environment.
 *   up_addrenv_vtext    - Returns the virtual base address of the .text
 *                         address environment
 *   up_addrenv_vdata    - Returns the virtual base address of the .bss/.data
 *                         address environment
 *   up_addrenv_heapsize - Returns the size of the initial heap allocation.
 *   up_addrenv_select   - Instantiate an address environment
 *   up_addrenv_restore  - Restore an address environment
 *   up_addrenv_clone    - Copy an address environment from one location to
 *                        another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach   - Clone the address environment assigned to one TCB
 *                         to another.  This operation is done when a pthread
 *                         is created that share's the same address
 *                         environment.
 *   up_addrenv_detach   - Release the threads reference to an address
 *                         environment when a task/thread exits.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>

#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only CONFIG_BUILD_KERNEL is supported (i.e. tested) */

#ifndef CONFIG_BUILD_KERNEL
#  error "This module is intended to be used with CONFIG_BUILD_KERNEL"
#endif

/* Amount of static pages allocated for an address environment */

#define STATIC_PAGES        (RV_MMU_PT_LEVELS - 1)

/* Entries per PGT */

#define ENTRIES_PER_PGT     (RV_MMU_PAGE_SIZE / sizeof(uint64_t))

/* Common virtual address base to use */

#define ARCH_VBASE          (CONFIG_ARCH_TEXT_VBASE)
#define ARCH_TEXT_VBASE     (CONFIG_ARCH_TEXT_VBASE)

/* Flags for user FLASH (RX) and user RAM (RW) */

#define MMU_UTEXT_FLAGS     (PTE_R | PTE_X | PTE_U | PTE_G)
#define MMU_UDATA_FLAGS     (PTE_R | PTE_W | PTE_U | PTE_G)

/* Memory barriers */
#define __DMB()             __asm__ __volatile__ ("fence"   ::: "memory")
#define __ISB()             __asm__ __volatile__ ("fence.i" ::: "memory")

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: connect_spgtables
 *
 * Description:
 *   Connect two static page tables together. Please note that index 0 is
 *   connected to the satp register, so it is skipped here.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment
 *
 ****************************************************************************/

static void connect_spgtables(FAR group_addrenv_t *addrenv, int idx)
{
  if (idx > 0)
    {
      uintptr_t prev = (uintptr_t)addrenv->spgtables[idx - 1];
      uintptr_t next = (uintptr_t)addrenv->spgtables[idx];
      mmu_ln_setentry(idx + 1, prev, next, next, PTE_G);
    }
}

/****************************************************************************
 * Name: create_spgtables
 *
 * Description:
 *   Create the static L1/LN-1 page tables. Allocate memory for them and
 *   connect them together.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment
 *
 * Returned value:
 *   Amount of pages created on success; a negated errno value on failure
 *
 ****************************************************************************/

static int create_spgtables(FAR group_addrenv_t *addrenv)
{
  int       npages;
  uintptr_t alloc;

  for (npages = 0; npages < STATIC_PAGES; npages++)
    {
      alloc = mm_pgalloc(1);
      if (!alloc)
        {
          return -ENOMEM;
        }

      /* Wipe the memory and assign it */

      memset((void *)alloc, 0, MM_PGSIZE);
      addrenv->spgtables[npages] = (uint64_t *)alloc;

      /* Connect the page tables */

      connect_spgtables(addrenv, npages);
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return npages;
}

/****************************************************************************
 * Name: create_spgtables
 *
 * Description:
 *   Create the static L1/LN-1 page tables. Allocate memory for them and
 *   connect them together. Note: the implementation maps page tables with
 *   vaddr=paddr mapping.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment
 *   vaddr - Base virtual address for the mapping
 *   size - Size of the region in bytes
 *   mmuflags - MMU flags to use
 *
 * Returned value:
 *   Amount of pages created on success; a negated errno value on failure
 *
 ****************************************************************************/

static int create_region(FAR group_addrenv_t *addrenv, uintptr_t vaddr,
                         size_t size, uint32_t mmuflags)
{
  uintptr_t lnvaddr;
  uintptr_t ptprev;
  uintptr_t paddr;
  uint32_t  ptlevel;
  int       npages;
  int       nmapped;
  int       i;
  int       j;

  nmapped   = 0;
  npages    = MM_NPAGES(size);
  ptprev    = (uintptr_t)addrenv->spgtables[STATIC_PAGES - 1];
  ptlevel   = STATIC_PAGES;

  /* Begin allocating memory the LN page tables */

  for (i = 0; i < npages; i += ENTRIES_PER_PGT)
    {
      /* Allocate one page for LN page table */

      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          return -ENOMEM;
        }

      /* Map the page to the prior level */

      mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, PTE_G);

      /* This is then used to map the final level */

      lnvaddr = paddr;
      memset((void *)lnvaddr, 0, MM_PGSIZE);

      /* Then allocate memory for the region data */

      for (j = 0; j < ENTRIES_PER_PGT && nmapped < size; j++)
        {
          paddr = mm_pgalloc(1);
          if (!paddr)
            {
              return -ENOMEM;
            }

          /* Wipe the physical page memory */

          memset((void *)paddr, 0, MM_PGSIZE);

          /* Then map the virtual address to the physical address */

          mmu_ln_setentry(ptlevel + 1, lnvaddr, paddr, vaddr, mmuflags);
          nmapped   += MM_PGSIZE;
          vaddr     += MM_PGSIZE;
        }
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return npages;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called when a new task is created in order to
 *   instantiate an address environment for the new task group.
 *   up_addrenv_create() is essentially the allocator of the physical
 *   memory for the new task.
 *
 * Input Parameters:
 *   textsize - The size (in bytes) of the .text address environment needed
 *     by the task.  This region may be read/execute only.
 *   datasize - The size (in bytes) of the .data/.bss address environment
 *     needed by the task.  This region may be read/write only.  NOTE: The
 *     actual size of the data region that is allocated will include a
 *     OS private reserved region at the beginning.  The size of the
 *     private, reserved region is give by ARCH_DATA_RESERVE_SIZE.
 *   heapsize - The initial size (in bytes) of the heap address environment
 *     needed by the task.  This region may be read/write only.
 *   addrenv - The location to return the representation of the task address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_create(size_t textsize, size_t datasize, size_t heapsize,
                      FAR group_addrenv_t *addrenv)
{
  int       ret;
  uintptr_t textbase;
  uintptr_t database;
  uintptr_t heapbase;

  DEBUGASSERT(addrenv);
  DEBUGASSERT(MM_ISALIGNED(ARCH_VBASE));
  DEBUGASSERT(MM_ISALIGNED(ARCH_TEXT_VBASE));

  /* Make sure the address environment is wiped before doing anything */

  memset(addrenv, 0, sizeof(group_addrenv_t));

  /* Create the static page tables */

  if (create_spgtables(addrenv) < 0)
    {
      serr("ERROR: Failed to create static page tables\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Calculate the base addresses for convenience */

  textbase = ARCH_TEXT_VBASE;
  database = textbase + MM_PGALIGNUP(textsize);
  datasize = datasize + ARCH_DATA_RESERVE_SIZE;
  heapbase = database + MM_PGALIGNUP(datasize);

  /* Map each region in turn */

  ret = create_region(addrenv, textbase, textsize, MMU_UTEXT_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create .text region: %d\n", ret);
      goto errout;
    }

  ret = create_region(addrenv, database, datasize, MMU_UDATA_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create .bss/.data region: %d\n", ret);
      goto errout;
    }

  ret = create_region(addrenv, heapbase, heapsize, MMU_UDATA_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create heap region: %d\n", ret);
      goto errout;
    }

  /* Save the heap base and initial size allocated. These will be needed when
   * the heap data structures are initialized.
   */

  addrenv->heapvbase = heapbase;
  addrenv->heapsize = (size_t)ret << MM_PGSHIFT;

  /* Save the data base */

  addrenv->datavbase = database;

  /* Provide the satp value for context switch */

  addrenv->satp = satp_reg((uint64_t)addrenv->spgtables[0], 0);

  /* When all is set and done, flush the data caches */

  __ISB();
  __DMB();

  return OK;

errout:
  up_addrenv_destroy(addrenv);
  return ret;
}

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   This function is called when a final thread leaves the task group and
 *   the task group is destroyed.  This function then destroys the defunct
 *   address environment, releasing the underlying physical memory.
 *
 * Input Parameters:
 *   addrenv - The address environment to be destroyed.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_destroy(FAR group_addrenv_t *addrenv)
{
  /* Recursively destroy it all, need to table walk */

  uint64_t  *pgtable;
  uintptr_t pte;
  int       i;

  DEBUGASSERT(addrenv);

  /* First destroy the leaf entries */

  pgtable = addrenv->spgtables[STATIC_PAGES - 1];
  if (pgtable)
    {
      for (i = 0; i < ENTRIES_PER_PGT; i++)
        {
          pte = (uintptr_t)pgtable[i];
          if (pte)
            {
              mm_pgfree(pte, 1);
            }
        }
    }

  /* Then destroy the static tables */

  for (i = 0; i < STATIC_PAGES; i++)
    {
      pgtable = addrenv->spgtables[i];
      if (pgtable)
        {
          mm_pgfree((uintptr_t)pgtable, 1);
        }
    }

  /* When all is set and done, flush the data caches */

  __ISB();
  __DMB();

  memset(addrenv, 0, sizeof(group_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vtext
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vtext - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vtext(FAR group_addrenv_t *addrenv, FAR void **vtext)
{
  DEBUGASSERT(addrenv && vtext);
  *vtext = (FAR void *)ARCH_TEXT_VBASE;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vdata
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   textsize - For some implementations, the text and data will be saved
 *      in the same memory region (read/write/execute) and, in this case,
 *      the virtual address of the data just lies at this offset into the
 *      common region.
 *   vdata - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vdata(FAR group_addrenv_t *addrenv, uintptr_t textsize,
                     FAR void **vdata)
{
  DEBUGASSERT(addrenv && vdata);
  *vdata = (FAR void *)(addrenv->datavbase + ARCH_DATA_RESERVE_SIZE);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_heapsize
 *
 * Description:
 *   Return the initial heap allocation size.  That is the amount of memory
 *   allocated by up_addrenv_create() when the heap memory region was first
 *   created.  This may or may not differ from the heapsize parameter that
 *   was passed to up_addrenv_create()
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   The initial heap size allocated is returned on success; a negated
 *   errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
ssize_t up_addrenv_heapsize(FAR const group_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);
  return (ssize_t)addrenv->heapsize;
}
#endif

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task (via
 *   up_addrenv_create()), this function may be called to instantiate
 *   that address environment in the virtual address space.  This might be
 *   necessary, for example, to load the code for the task from a file or
 *   to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   oldenv
 *     The address environment that was in place before up_addrenv_select().
 *     This may be used with up_addrenv_restore() to restore the original
 *     address environment that was in place before up_addrenv_select() was
 *     called.  Note that this may be a task agnostic, hardware
 *     representation that is different from group_addrenv_t.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(FAR const group_addrenv_t *addrenv,
                      FAR save_addrenv_t *oldenv)
{
  DEBUGASSERT(addrenv);
  if (oldenv)
    {
      /* Save the old environment */

      uint64_t satp_reg = mmu_read_satp();
      *oldenv = (save_addrenv_t) satp_reg;
    }

  mmu_write_satp(addrenv->satp);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_restore
 *
 * Description:
 *   After an address environment has been temporarily instantiated by
 *   up_addrenv_select, this function may be called to restore the
 *   original address environment.
 *
 * Input Parameters:
 *   oldenv - The hardware representation of the address environment
 *     previously returned by up_addrenv_select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_restore(FAR const save_addrenv_t *oldenv)
{
  DEBUGASSERT(oldenv);
  mmu_write_satp((uint64_t)*oldenv);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_coherent
 *
 * Description:
 *   Flush D-Cache and invalidate I-Cache in preparation for a change in
 *   address environments.  This should immediately precede a call to
 *   up_addrenv_select();
 *
 * Input Parameters:
 *   addrenv - Describes the address environment to be made coherent.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_coherent(FAR const group_addrenv_t *addrenv)
{
  /* Flush the instruction and data caches */

  __ISB();
  __DMB();
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_clone
 *
 * Description:
 *   Duplicate an address environment.  This does not copy the underlying
 *   memory, only the representation that can be used to instantiate that
 *   memory as an address environment.
 *
 * Input Parameters:
 *   src - The address environment to be copied.
 *   dest - The location to receive the copied address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_clone(FAR const group_addrenv_t *src,
                     FAR group_addrenv_t *dest)
{
  DEBUGASSERT(src && dest);
  memcpy(dest, src, sizeof(group_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_attach
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address environment of its task
 *   group.
 *
 * Input Parameters:
 *   group - The task group to which the new thread belongs.
 *   tcb   - The tcb of the thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_attach(FAR struct task_group_s *group, FAR struct tcb_s *tcb)
{
  /* There is nothing that needs to be done */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_detach
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  The address environment,
 *   however, should persist until up_addrenv_destroy() is called when the
 *   task group is itself destroyed.  Any resources unique to this thread
 *   may be destroyed now.
 *
 *   NOTE: In some platforms, nothing will need to be done in this case.
 *   Simply being a member of the group that has the address environment
 *   may be sufficient.
 *
 * Input Parameters:
 *   group - The group to which the thread belonged.
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_detach(FAR struct task_group_s *group, FAR struct tcb_s *tcb)
{
  /* There is nothing that needs to be done */

  return OK;
}
