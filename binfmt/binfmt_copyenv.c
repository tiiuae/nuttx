/****************************************************************************
 * binfmt/binfmt_copyenv.c
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
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_copyenv
 *
 * Description:
 *   In the kernel build, the environment exists in the parent's address
 *   environment and, hence, will be inaccessible when we switch to the
 *   address environment of the new process. So we do not have any real
 *   option other than to copy the parents envp list into an intermediate
 *   buffer that resides in neutral kernel memory.
 *
 * Input Parameters:
 *   envp     - Allocated environment strings
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

FAR char * const *binfmt_copyenv(FAR char * const *envp)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* In this case the current task must be the one we are copying */

  FAR struct tcb_s *ptcb = nxsched_self();
  FAR char *envcp = (FAR char *) envp;
  size_t envlen;

  /* Check that the parent has a group and an environment */

  if (ptcb->group != NULL && ptcb->group->tg_envp != NULL)
    {
      envlen = ptcb->group->tg_envsize;

      if (envlen > 0)
        {
          envcp = (FAR char *)kmm_malloc(envlen);
          if (!envcp)
            {
              berr("ERROR: Failed to allocate the environment buffer\n");
              return NULL;
            }

          /* Copy envp to the kernel allocated buffer */

          memcpy(envcp, ptcb->group->tg_envp, envlen);
        }
    }

  return (FAR char * const *)envcp;
#else
  /* Just return the original */

  return envp;
#endif
}

/****************************************************************************
 * Name: binfmt_freeenv
 *
 * Description:
 *   Release the copied envp[] list.
 *
 * Input Parameters:
 *   envp     - Allocated environment strings
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
void binfmt_freeenv(FAR char * const *envp)
{
  if (envp)
    {
      kmm_free((FAR char *)envp);
    }
}
#endif
#endif /* CONFIG_BINFMT_DISABLE */
