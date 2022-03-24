/****************************************************************************
 * fs/shm/shmfs_private.h
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

#ifndef __FS_SHM_SHMFS_PRIVATE_H
#define __FS_SHM_SHMFS_PRIVATE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <nuttx/fs/shmfs.h>
#include "inode/inode.h"

struct shmfs_object_s
{
  size_t length;
  void *paddr[];
};

FAR struct shmfs_object_s *alloc_shm_object(size_t length);

void delete_shm_object(FAR struct shmfs_object_s *object);

#endif
