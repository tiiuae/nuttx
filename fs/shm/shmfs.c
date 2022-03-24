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

#include <nuttx/fs/ioctl.h>
#include <assert.h>
#include <nuttx/fs/shmfs.h>
#include "shmfs_private.h"
#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#ifdef CONFIG_BUILD_KERNEL
#error Not implemented
#else
  if (object && object->paddr[0])
    {
      len = buflen > object->length ? object->length : buflen;
      memcpy(buffer, object->paddr[0], len);
    }

#endif
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

#ifdef CONFIG_BUILD_KERNEL
#error Not implemented
#else
  if (object && object->paddr[0])
    {
      len = buflen > object->length ? object->length : buflen;
      memcpy(object->paddr[0], buffer, len);
    }

#endif
  return len;
}

/****************************************************************************
 * Name: shmfs_ioctl
 ****************************************************************************/

static int  shmfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;
  switch (cmd)
    {
    case FIOC_MMAP:
      {
        FAR struct shmfs_object_s *object =
          (FAR struct shmfs_object_s *)filep->f_inode->i_private;

        /* object may be null if the shm has not bee ftruncated */

        if (object)
          {
#ifdef CONFIG_BUILD_KERNEL
#error Not implemented
#else
            *(uintptr_t *)arg = (uintptr_t)object->paddr[0];
#endif
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmfs_initialize
 ****************************************************************************/

void shmfs_initialize(void)
{
  register_driver(CONFIG_FS_SHM_VFS_PATH, &shmfs_operations, 0666, NULL);
}
