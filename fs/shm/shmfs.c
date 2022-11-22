/****************************************************************************
 * fs/shm/shmfs.c
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

#include <assert.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/shmfs.h>

#if defined (CONFIG_BUILD_KERNEL)
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>
#include <nuttx/mm/vm_map.h>
#endif

#include "shmfs_private.h"
#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int shmfs_close(FAR struct file *filep);
static ssize_t shmfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t shmfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int shmfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int shmfs_truncate(FAR struct file *filep, off_t length);

/* Helper functions for mmap / unmap */

static int shmfs_mmap(FAR uintptr_t *pages, size_t length,
                      FAR uintptr_t *vaddr);
static int shmfs_unmap(uintptr_t vaddr, size_t length);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct file_operations shmfs_operations =
{
  NULL,             /* open */
  shmfs_close,      /* close */
  shmfs_read,       /* read */
  shmfs_write,      /* write */
  NULL,             /* seek */
  shmfs_ioctl,      /* ioctl */
  NULL,             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,             /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmfs_close
 ****************************************************************************/

static int shmfs_close(FAR struct file *filep)
{
  /* If the file has been unlinked previously, delete the contents.
   * The inode is released after this call, hence checking if i_crefs <= 1.
   */

  if (filep->f_inode->i_parent == NULL &&
      filep->f_inode->i_crefs <= 1)
    {
      delete_shm_object(filep->f_inode->i_private);
      filep->f_inode->i_private = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: shmfs_read
 ****************************************************************************/

static ssize_t shmfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct shmfs_object_s *object =
    (FAR struct shmfs_object_s *)filep->f_inode->i_private;
  ssize_t len = 0;

  if (object && object->paddr[0])
    {
      len = buflen > object->length ? object->length : buflen;
#ifdef CONFIG_BUILD_KERNEL
      FAR uintptr_t *pages = (FAR uintptr_t *)&object->paddr[0];
      ssize_t remaining = len;
      ssize_t slice;
      uintptr_t paddr;

      /* Must copy page by page */

      while (remaining > 0)
        {
          slice = min(len, MM_PGSIZE);
          paddr = *pages++;
          remaining -= slice;
          memcpy(buffer, up_addrenv_pa_to_va(paddr) , slice);
          buffer += slice;
        }
#else
      /* Can copy as a bundle */

      memcpy(buffer, object->paddr[0], len);
#endif
    }

  return len;
}

/****************************************************************************
 * Name: shmfs_write
 ****************************************************************************/

static ssize_t shmfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct shmfs_object_s *object =
    (FAR struct shmfs_object_s *)filep->f_inode->i_private;
  ssize_t len = 0;

  if (object && object->paddr[0])
    {
      len = buflen > object->length ? object->length : buflen;
#ifdef CONFIG_BUILD_KERNEL
      FAR uintptr_t *pages = (FAR uintptr_t *)&object->paddr[0];
      ssize_t remaining = len;
      ssize_t slice;
      uintptr_t paddr;

      /* Must copy page by page */

      while (remaining > 0)
        {
          slice = min(len, MM_PGSIZE);
          paddr = *pages++;
          remaining -= slice;
          memcpy(up_addrenv_pa_to_va(paddr), buffer, slice);
          buffer += slice;
        }
#else
      /* Can copy as a bundle */

      memcpy(object->paddr[0], buffer, len);
#endif
    }

  return len;
}

/****************************************************************************
 * Name: shmfs_ioctl
 ****************************************************************************/

static int shmfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;
  switch (cmd)
    {
    case FIOC_MMAP:
      {
        FAR struct shmfs_object_s *object =
          (FAR struct shmfs_object_s *)filep->f_inode->i_private;

        /* object may be null if the shm has not been ftruncated */

        if (object)
          {
            FAR uintptr_t *pages = (FAR uintptr_t *)&object->paddr[0];
            uintptr_t      vaddr;

            ret = shmfs_mmap(pages, object->length, &vaddr);
            if (ret >= 0)
              {
                *(uintptr_t *)arg = vaddr;
              }

            /* Keep the inode when mmapped, increase refcount */

            filep->f_inode->i_crefs++;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;
    case FIOC_MUNMAP:
      {
        const FAR struct vm_map_entry_s *map;

        /* Remove mapping regardless */

        map = (const FAR struct vm_map_entry_s *)arg;
        if (map)
          {
            shmfs_unmap((uintptr_t)map->vaddr, map->length);
          }

        /* Delete the object if it has been unlinked
         * and it is no longer referenced
         */

        if (filep->f_inode->i_parent == NULL &&
            filep->f_inode->i_crefs <= 1)
          {
            delete_shm_object(filep->f_inode->i_private);
            filep->f_inode->i_private = NULL;
          }

        /* And release the inode */

        inode_release(filep->f_inode);
      }
      break;
    case FIOC_TRUNCATE:
      if (shmfs_truncate(filep, arg) != OK)
        {
          ret = -EINVAL;
        }
      break;
    default:
      ret = -ENOSYS;
  }

  return ret;
}

/****************************************************************************
 * Name: shmfs_truncate
 ****************************************************************************/

static int shmfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct shmfs_object_s *object = filep->f_inode->i_private;

  if (!object)
    {
      filep->f_inode->i_private = alloc_shm_object(length);
    }
  else if (object->length != length)
    {
      /* This doesn't support resize */

      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: shmfs_mmap
 ****************************************************************************/

static int shmfs_mmap(FAR uintptr_t *pages, size_t length,
                      FAR uintptr_t *vaddr)
{
#ifdef CONFIG_BUILD_KERNEL
  FAR struct tcb_s        *tcb   = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  uintptr_t                mapaddr;
  unsigned int             npages;
  int                      ret;

  /* Find a free vaddr space that satisfies length */

  mapaddr = (uintptr_t)shm_alloc(group, 0, length);

  /* Convert the region size to pages */

  npages = MM_NPAGES(length);

  /* Map the memory to user virtual address space */

  ret = up_shmat(pages, npages, mapaddr);
  if (ret < 0)
    {
      shm_free(group, (FAR void *)mapaddr, length);
    }

  *vaddr = mapaddr;
  return ret;
#else
  /* In flat mode just return the physical address */

  *vaddr = (uintptr_t)pages;
  return OK;
#endif
}

/****************************************************************************
 * Name: shmfs_unmap
 ****************************************************************************/

static int shmfs_unmap(uintptr_t vaddr, size_t length)
{
#ifdef CONFIG_BUILD_KERNEL
  FAR struct tcb_s        *tcb   = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  unsigned int             npages;
  int                      ret;

  /* Convert the region size to pages */

  npages = MM_NPAGES(length);

  /* Unmap the memory from user virtual address space */

  ret = up_shmdt(vaddr, npages);

  /* Add the virtual memory back to the shared memory pool */

  shm_free(group, (FAR void *)vaddr, length);

  return ret;
#else
  /* No work to do */

  return OK;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmfs_initialize
 ****************************************************************************/

void shmfs_initialize(void)
{
  register_driver(CONFIG_FS_SHM_VFS_PATH, &shmfs_operations, 0666, NULL);
}
