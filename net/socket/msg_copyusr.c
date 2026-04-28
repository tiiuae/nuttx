/****************************************************************************
 * net/socket/msg_copyusr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_ARCH_ADDRENV)

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>

#include <nuttx/kmalloc.h>

#ifdef CONFIG_MM_IOB
#include <nuttx/mm/iob.h>
#endif

#include "socket/socket.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct msg_kbuf_s
{
  struct msghdr       kmsg;        /* Kernel copy of user msghdr */
#ifdef CONFIG_MM_IOB
  FAR struct iob_s   *iob;         /* IOB containing this structure */
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msg_alloc_kbuf
 *
 * Description:
 *   In CONFIG_BUILD_KERNEL + CONFIG_ARCH_ADDRENV builds each user process
 *   has its own MMU page table.  When a blocking socket call yields, the
 *   scheduler may switch to a different user process, changing the active
 *   address environment (SATP on RISC-V, TTBR on ARM).  Network callbacks
 *   that fire in that context and copy data to/from the original user-space
 *   iov_base / msg_name / msg_control pointers would access the wrong
 *   physical memory.
 *
 *   This function creates a kernel-owned copy of the relevant msghdr
 *   fields (iov payload, msg_name, msg_control) so that all internal
 *   network stack accesses go through kernel virtual addresses, which are
 *   mapped identically in every address environment.
 *
 *   The full iovec array is preserved so protocol code that supports
 *   scatter / gather I/O can continue to iterate over msg_iovlen entries.
 *   The helper allocates one kernel-owned backing store for the copied
 *   msghdr state and payload, using a single IOB when everything fits or
 *   falling back to a single heap allocation otherwise.
 *
 * Input Parameters:
 *   src - Original user-space msghdr (read-only after this call).
 *   msg - Out parameter. Set to allocated kernel msghdr on success.
 *
 * Returned Value:
 *   0 on success; negated errno on failure:
 *   -ENOMEM    Allocation failure.
 *   -EOVERFLOW Size arithmetic overflow.
 *   -EFAULT    Invalid iov pointer input.
 *   -EINVAL    Invalid arguments.
 *
 ****************************************************************************/

int msg_alloc_kbuf(FAR const struct msghdr *src,
                   FAR struct msghdr **msg)
{
  FAR struct msg_kbuf_s *kbuf = NULL;
  FAR uint8_t *data;
  FAR struct iovec *iov = NULL;
  size_t cmsg_len;
  size_t name_len;
  size_t data_len;
  size_t iovbytes;
  size_t iovcount;
  size_t i;

#ifdef CONFIG_MM_IOB
  FAR struct iob_s *iob = NULL;
#endif

  if (src == NULL || msg == NULL)
    {
      return -EINVAL;
    }

  *msg = NULL;

  /* First, calculate size of needed data, and validate all inputs */

  iovcount = src->msg_iovlen;

  if (iovcount > 0 && src->msg_iov == NULL)
    {
      return -EFAULT;
    }

  if (iovcount > SIZE_MAX / sizeof(struct iovec))
    {
      return -EOVERFLOW;
    }

  iovbytes    = iovcount * sizeof(struct iovec);
  cmsg_len    = src->msg_controllen > 0 && src->msg_control != NULL ?
                src->msg_controllen : 0;
  name_len    = src->msg_namelen > 0 && src->msg_name != NULL ?
                src->msg_namelen : 0;

  data_len = sizeof(struct msg_kbuf_s);

  if (iovbytes > SIZE_MAX - data_len)
    {
      return -EOVERFLOW;
    }

  data_len += iovbytes;

  if (name_len > SIZE_MAX - data_len)
    {
      return -EOVERFLOW;
    }

  data_len += name_len;

  if (cmsg_len > SIZE_MAX - data_len)
    {
      return -EOVERFLOW;
    }

  data_len += cmsg_len;

  for (i = 0; i < iovcount; i++)
    {
      if (src->msg_iov[i].iov_len > 0 && src->msg_iov[i].iov_base == NULL)
        {
          return -EFAULT;
        }

      if (src->msg_iov[i].iov_len > SIZE_MAX - data_len)
        {
          return -EOVERFLOW;
        }

      data_len += src->msg_iov[i].iov_len;
    }

  /* Allocate the needed kernel side memory. */

#ifdef CONFIG_MM_IOB
  /* Check if everything fits in one IOB, in that case allocate from the IOB
   * pool instead of kernel heap
   */

  if (data_len <= CONFIG_IOB_BUFSIZE)
    {
      iob = iob_alloc(false);
      if (iob != NULL)
        {
          kbuf = (FAR struct msg_kbuf_s *)iob->io_data;
        }
    }
#endif

  if (kbuf == NULL)
    {
      kbuf = (FAR struct msg_kbuf_s *)kmm_malloc(data_len);
    }

  if (kbuf == NULL)
    {
      return -ENOMEM;
    }

  /* Now we have kernel side buffer at kbuf, clear it */

  memset(kbuf, 0, data_len);

#ifdef CONFIG_MM_IOB
  if (iob)
    {
      /* Store the iob pointer for freeing it later */

      kbuf->iob = iob;
    }
#endif

  /* Now, set all fields of the kernel-side msghdr */

  data = (FAR uint8_t *)&kbuf[1];

  if (iovcount > 0)
    {
      kbuf->kmsg.msg_iov = (FAR struct iovec *)data;
      kbuf->kmsg.msg_iovlen = iovcount;
      iov = kbuf->kmsg.msg_iov;
      data += iovbytes;
    }

  /* Compile-time layout checks:
   * - iov[] must start aligned for pointer-sized accesses.
   * - msg_control starts at: sizeof(msg_kbuf_s) + iovcount * sizeof(iovec).
   *   Therefore both component sizes must be CMSG_ALIGN-stable so the sum
   *   remains naturally aligned for any iovcount.
   */

  static_assert((sizeof(kbuf[0]) & (sizeof(uintptr_t) - 1)) == 0,
                "msg_kbuf_s size must be pointer aligned");
  static_assert((sizeof(struct iovec) & (sizeof(uintptr_t) - 1)) == 0,
                "iovec size must be pointer aligned");
  static_assert(CMSG_ALIGN(sizeof(kbuf[0])) == sizeof(kbuf[0]),
                "msg_kbuf_s size must be CMSG_ALIGN aligned");
  static_assert(CMSG_ALIGN(sizeof(struct iovec)) == sizeof(struct iovec),
                "iovec size must be CMSG_ALIGN aligned");

  if (cmsg_len > 0)
    {
      kbuf->kmsg.msg_control = data;
      kbuf->kmsg.msg_controllen = cmsg_len;
      data += cmsg_len;
    }

  if (name_len > 0)
    {
      kbuf->kmsg.msg_name = data;
      kbuf->kmsg.msg_namelen = name_len;
      data += name_len;
    }

  for (i = 0; i < iovcount; i++)
    {
      size_t iov_len = src->msg_iov[i].iov_len;

      iov[i].iov_len = iov_len;
      iov[i].iov_base = data;

      if (iov_len == 0)
        {
          continue;
        }

      data += iov_len;
    }

  *msg = &kbuf->kmsg;
  return OK;
}

/****************************************************************************
 * Name: msg_copy_from_user
 *
 * Description:
 *   Copy user-space send data into the kernel bounce buffers allocated by
 *   msg_alloc_kbuf().  Call this after alloc and before psock_sendmsg.
 *
 * Input Parameters:
 *   src - Original user msghdr whose payload and control data to copy.
 *   msg - Kernel msghdr state filled by msg_alloc_kbuf().
 *
 * Returned Value:
 *   0 on success; -EFAULT if any iov entry with non-zero length has a
 *   NULL iov_base pointer.
 *
 ****************************************************************************/

int msg_copy_from_user(FAR const struct msghdr *src,
                       FAR struct msghdr *msg)
{
  FAR struct msg_kbuf_s *kbuf = (FAR struct msg_kbuf_s *)msg;
  size_t i;

  /* Validate and copy iov payload from user to kernel IOB. */

  if (src->msg_iov != NULL && kbuf->kmsg.msg_iov != NULL)
    {
      for (i = 0; i < src->msg_iovlen; i++)
        {
          if (src->msg_iov[i].iov_len > 0)
            {
              if (src->msg_iov[i].iov_base == NULL)
                {
                  return -EFAULT;
                }

              if (kbuf->kmsg.msg_iov[i].iov_base != NULL)
                {
                  memcpy(kbuf->kmsg.msg_iov[i].iov_base,
                         src->msg_iov[i].iov_base,
                         src->msg_iov[i].iov_len);
                }
            }
        }
    }

  /* Copy destination address from user to kernel buffer (needed by
   * sendmsg() on datagram sockets).
   */

  if (src->msg_namelen > 0 && src->msg_name != NULL &&
      kbuf->kmsg.msg_name != NULL)
    {
      memcpy(kbuf->kmsg.msg_name, src->msg_name, src->msg_namelen);
    }

  /* Copy ancillary control data from user to kernel IOB. */

  if (src->msg_controllen > 0 && src->msg_control != NULL &&
      kbuf->kmsg.msg_control != NULL)
    {
      memcpy(kbuf->kmsg.msg_control, src->msg_control,
             src->msg_controllen);
    }

  return OK;
}

/****************************************************************************
 * Name: msg_copy_to_user
 *
 * Description:
 *   Copy received data from the kernel bounce buffers back to the original
 *   user-space msghdr fields.  Call this after psock_recvmsg() returns
 *   successfully, while the correct address environment is active.
 *
 * Input Parameters:
 *   dst     - Original user-space msghdr to update.
 *   ksrc    - Kernel msghdr state filled and used by psock_recvmsg().
 *   recvlen - Number of payload bytes returned by psock_recvmsg().
 *
 * Returned Value:
 *   recvlen on success; -EFAULT if any iov entry with non-zero length has
 *   a NULL iov_base pointer.  When recvlen is negative (error from
 *   psock_recvmsg) it is returned unchanged without touching user buffers.
 *
 ****************************************************************************/

ssize_t msg_copy_to_user(FAR struct msghdr *dst,
                         FAR const struct msghdr *src,
                         ssize_t recvlen)
{
  FAR const struct msg_kbuf_s *ksrc = (FAR const struct msg_kbuf_s *)src;
  ssize_t origlen = recvlen;
  size_t ncopy;
  size_t iovcount;
  size_t i;

  /* Only copy results back on success; on error the user buffers
   * must not be touched as they may contain partial or undefined data.
   */

  if (recvlen < 0)
    {
      return recvlen;
    }

  /* Validate and copy received payload back to the user iov buffer. */

  if (recvlen > 0 && dst->msg_iov != NULL && ksrc->kmsg.msg_iov != NULL)
    {
      iovcount = dst->msg_iovlen;
      if (iovcount > ksrc->kmsg.msg_iovlen)
        {
          iovcount = ksrc->kmsg.msg_iovlen;
        }

      for (i = 0; i < iovcount && recvlen > 0; i++)
        {
          ncopy = dst->msg_iov[i].iov_len;
          if (ncopy > (size_t)recvlen)
            {
              ncopy = recvlen;
            }

          if (ncopy > 0)
            {
              if (dst->msg_iov[i].iov_base == NULL)
                {
                  return (ssize_t)-EFAULT;
                }

              memcpy(dst->msg_iov[i].iov_base,
                     ksrc->kmsg.msg_iov[i].iov_base, ncopy);
              recvlen -= ncopy;
            }
        }
    }

  /* Copy source address back. */

  if (ksrc->kmsg.msg_name != NULL && dst->msg_name != NULL)
    {
      memcpy(dst->msg_name, ksrc->kmsg.msg_name,
             ksrc->kmsg.msg_namelen);
    }

  /* Copy ancillary data back.
   * psock_recvmsg() decrements msg_controllen by the amount consumed via
   * cmsg_append(); the difference is the number of bytes written.
   */

  if (ksrc->kmsg.msg_control != NULL && dst->msg_control != NULL)
    {
      size_t written = ksrc->kmsg.msg_controllen;
      if (written > 0)
        {
          memcpy(dst->msg_control, ksrc->kmsg.msg_control, written);
        }
    }

  /* Propagate output fields. */

  dst->msg_namelen    = ksrc->kmsg.msg_namelen;
  dst->msg_controllen = ksrc->kmsg.msg_controllen;
  dst->msg_flags      = ksrc->kmsg.msg_flags;

  return origlen;
}

/****************************************************************************
 * Name: msg_free_kbuf
 *
 * Description:
 *   Release all kernel memory allocated by msg_alloc_kbuf().
 *   Safe to call with partially-initialised state; this function checks
 *   all pointers for NULL before freeing them.
 *
 * Input Parameters:
 *   kbuf - Kernel msghdr state to release.
 *
 ****************************************************************************/

void msg_free_kbuf(FAR struct msghdr *msg)
{
  FAR struct msg_kbuf_s *kbuf = (FAR struct msg_kbuf_s *)msg;

  if (kbuf == NULL)
    {
      return;
    }

#ifdef CONFIG_MM_IOB
  if (kbuf->iob != NULL)
    {
      iob_free(kbuf->iob);
    }
  else
#endif
    {
      kmm_free(kbuf);
    }
}

#endif /* CONFIG_BUILD_KERNEL && CONFIG_ARCH_ADDRENV */
