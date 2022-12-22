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

#include <sys/mman.h>

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
static int shmfs_truncate(FAR struct file *filep, off_t length);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int shmfs_unlink(FAR struct inode *inode);
#endif

static FAR void *shmfs_mmap(FAR struct file *filep, off_t start,
                            size_t length);

static int shmfs_munmap(FAR struct task_group_s *group,
                        FAR struct inode *inode, void *vaddr, size_t length);

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
  NULL,             /* ioctl */
  shmfs_truncate,   /* truncate */
  shmfs_mmap,       /* mmap */
  shmfs_munmap,     /* munmap */
  NULL              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , shmfs_unlink    /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmfs_close
 ****************************************************************************/

static int _shmfs_close(FAR struct inode *inode)
{
  /* If the file has been unlinked previously, delete the contents.
   * The inode is released after this call, hence checking if i_crefs <= 1.
   */

  int ret = inode_lock();
  if (ret >= 0)
    {
      if (inode->i_parent == NULL &&
          inode->i_crefs <= 1)
        {
          delete_shm_object(inode->i_private);
          inode->i_private = NULL;
        }

      inode_unlock();
    }

  return ret;
}

static int shmfs_close(FAR struct file *filep)
{
  return _shmfs_close(filep->f_inode);
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
 * Name: shmfs_truncate
 ****************************************************************************/

static int shmfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct shmfs_object_s *object;
  int ret;

  ret = inode_lock();
  if (ret >= 0)
    {
      object = filep->f_inode->i_private;
      if (!object)
        {
          filep->f_inode->i_private = alloc_shm_object(length);
        }
      else if (object->length != length)
        {
          /* This doesn't support resize */

          ret = ERROR;
        }

      inode_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: shmfs_unlink
 ****************************************************************************/
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int shmfs_unlink(FAR struct inode *inode)
{
  if (inode->i_crefs <= 1)
    {
      delete_shm_object(inode->i_private);
      inode->i_private = NULL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: shmfs_mmap
 ****************************************************************************/

FAR void *shmfs_mmap(FAR struct file *filep, off_t start, size_t length)
{
  FAR struct shmfs_object_s *object;
  void *vaddr = MAP_FAILED;
  int ret;

  /* Keep the inode when mmapped, increase refcount */

  ret = inode_addref(filep->f_inode);
  if (ret >= 0)
    {
      object = (FAR struct shmfs_object_s *)filep->f_inode->i_private;

      /* object may be null if the shm has not been ftruncated */

      if (object)
        {
          FAR uintptr_t *pages = (FAR uintptr_t *)&object->paddr[0];

#ifdef CONFIG_BUILD_KERNEL
          FAR struct tcb_s        *tcb   = nxsched_self();
          FAR struct task_group_s *group = tcb->group;
          uintptr_t                mapaddr;
          unsigned int             npages;

          /* Find a free vaddr space that satisfies length */

          mapaddr = (uintptr_t)shm_alloc(group, 0, object->length);

          /* Convert the region size to pages */

          npages = MM_NPAGES(object->length);

          /* Map the memory to user virtual address space */

          ret = up_shmat(pages, npages, mapaddr);
          if (ret < 0)
            {
              shm_free(group, (FAR void *)mapaddr, object->length);
            }
          else
            {
              vaddr = mapaddr;
            }
#else
          /* In flat mode just use the physical address */

          vaddr = pages;
#endif

          if (ret < 0)
            {
              /* Oops, the file has not been truncated yet */

              inode_release(filep->f_inode);
              vaddr = MAP_FAILED;
            }
        }
    }

  return vaddr;
}

/****************************************************************************
 * Name: shmfs_munmap
 ****************************************************************************/

static int shmfs_munmap(FAR struct task_group_s *group,
                        FAR struct inode *inode, void *vaddr, size_t length)
{
  int                      ret = OK;
#ifdef CONFIG_BUILD_KERNEL
  unsigned int             npages;

  /* Convert the region size to pages */

  npages = MM_NPAGES(length);

  /* Unmap the memory from user virtual address space */

  ret = up_shmdt(vaddr, npages);

  /* Add the virtual memory back to the shared memory pool */

  shm_free(group, (FAR void *)vaddr, length);
#endif

  /* Delete the object if it has been unlinked
   * and it is no longer referenced, let _shmfs_close() do the work
   */

  return _shmfs_close(inode) == OK ? ret : -EFAULT;
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
