/****************************************************************************
 * fs/shm/shm_open.c
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
#include <sys/stat.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include "inode/inode.h"

/* Todo */

extern const struct file_operations shmfs_operations;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int file_shm_vopen(FAR struct file *shm, FAR const char *shm_name,
                   int oflags, mode_t mode)
{
  FAR struct inode *inode;
  struct inode_search_s desc;
  char fullpath[CONFIG_FS_SHM_VFS_MAXNAMLEN];
  int ret;

  /* Make sure that a non-NULL name is supplied */

  if (!shm || !shm_name || *shm_name == '/' || *shm_name == '\0')
    {
      return -EINVAL;
    }

  /* Get the full path to the shm object */

  snprintf(fullpath, CONFIG_FS_SHM_VFS_MAXNAMLEN,
           CONFIG_FS_SHM_VFS_PATH "/%s", shm_name);

  /* Get the inode for this shm object */

  SETUP_SEARCH(&desc, fullpath, false);

  ret = inode_lock();
  if (ret < 0)
    {
      goto errout_with_search;
    }

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at this path.  Get the search results */

      inode = desc.node;

      /* Verify that the inode is an shm object */

      if (!INODE_IS_SHM(inode))
        {
          ret = -EINVAL;
          inode_release(inode);
          goto errout_with_sem;
        }

      /* It exists and is an shm object.  Check if the caller wanted to
       * create a new object with this name.
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          ret = -EEXIST;
          inode_release(inode);
          goto errout_with_sem;
        }
    }
  else
    {
      /* The shm does not exist. Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* The shm does not exist and O_CREAT is not set */

          ret = -ENOENT;
          goto errout_with_sem;
        }

      /* Create an inode in the pseudo-filesystem at this path */

      ret = inode_reserve(fullpath, mode, &inode);

      if (ret < 0)
        {
          goto errout_with_sem;
        }

      INODE_SET_SHM(inode);
      inode->u.i_ops    = &shmfs_operations;
      inode->i_private  = NULL;
      inode->i_crefs    = 1;
    }

  /* Associate the inode with a file structure */

  shm->f_oflags  = oflags;
  shm->f_pos     = 0;
  shm->f_inode   = inode;
  shm->f_priv    = NULL;
  ret = OK;

errout_with_sem:
  inode_unlock();
errout_with_search:
  RELEASE_SEARCH(&desc);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int shm_open(FAR const char *name, int oflag, mode_t mode)
{
  struct file shm;
  int ret;

  ret = file_shm_vopen(&shm, name, oflag, mode);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_allocate(shm.f_inode, shm.f_oflags, shm.f_pos, shm.f_priv, 0,
                      false);
  if (ret < 0)
    {
      file_close(&shm);
    }

  return ret;
}
