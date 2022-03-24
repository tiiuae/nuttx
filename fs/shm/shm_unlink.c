/****************************************************************************
 * fs/shm/shm_unlink.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/fs/shmfs.h>
#include "shmfs_private.h"
#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int file_shm_unlink(FAR const char *shm_name)
{
  FAR struct inode *inode;
  struct inode_search_s desc;
  char fullpath[CONFIG_FS_SHM_VFS_MAXNAMLEN];
  int ret;

  /* Get the full path to the message queue */

  snprintf(fullpath, CONFIG_FS_SHM_VFS_MAXNAMLEN,
           CONFIG_FS_SHM_VFS_PATH "/%s", shm_name);

  /* Get the inode for this message queue. */

  SETUP_SEARCH(&desc, fullpath, false);

  ret = inode_semtake();
  if (ret < 0)
    {
      goto errout_with_search;
    }

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      goto errout_with_sem;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that what we found is, indeed, a message queue */

  if (!INODE_IS_SHM(inode))
    {
      ret = -ENOENT;
      inode_release(inode);
      goto errout_with_sem;
    }

  /* Delete the object if all instances have been unmapped
   */

  if (inode->i_crefs <= 1)
    {
      delete_shm_object(inode->i_private);
      inode->i_private = NULL;
    }

  /* Remove the old inode from the tree. If we hold a reference count
   * on the inode, it will not be deleted now.  This will set the
   * FSNODEFLAG_DELETED bit in the inode flags.
   */

  inode_release(inode);
  ret = inode_remove(fullpath);

  /* inode_remove() should always fail with -EBUSY because we have a
   * reference on the inode.  -EBUSY means that the inode was, indeed,
   * unlinked but it could not be freed because there are references.
   */

  if (ret == -EBUSY)
    {
      ret = OK;
    }

  DEBUGASSERT(ret == OK);

errout_with_sem:
  inode_semgive();

errout_with_search:
  RELEASE_SEARCH(&desc);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int shm_unlink(FAR const char *shm_name)
{
  int ret = file_shm_unlink(shm_name);

  if (ret < 0)
    {
      set_errno(ENOENT);
    }

  return ret;
}
