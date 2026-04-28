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

#include <string.h>
#include <errno.h>
#include <sys/socket.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>

#include "socket/socket.h"

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
 *   Only a single iovec (msg_iovlen == 1) is handled.  All protocols
 *   that reach blocking code already enforce this restriction.
 *
 * Input Parameters:
 *   src     - Original user-space msghdr (read-only after this call).
 *
 * Returned Value:
 *   Pointer to the kernel msghdr on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct msghdr *msg_alloc_kbuf(FAR const struct msghdr *src)
{
  FAR struct msg_kbuf_s *kbuf;
  FAR struct iob_s *meta_iob;
  FAR uint8_t *meta;
  size_t payload_len;
  size_t cmsg_len;
  size_t name_len;
  size_t fixed_len;
  size_t payload_room;
  bool has_payload;
  bool has_cmsg;
  bool has_name;

  static_assert(sizeof(struct msg_kbuf_s) <= CONFIG_IOB_BUFSIZE,
                "msg_kbuf_s does not fit in one IOB");

  has_payload = (src->msg_iovlen >= 1 && src->msg_iov != NULL &&
                 src->msg_iov->iov_base != NULL &&
                 src->msg_iov->iov_len > 0);
  has_cmsg = (src->msg_controllen > 0 && src->msg_control != NULL);
  has_name = (src->msg_namelen > 0 && src->msg_name != NULL);

  payload_len = has_payload ? src->msg_iov->iov_len : 0;
  cmsg_len    = has_cmsg ? src->msg_controllen : 0;
  name_len    = has_name ? src->msg_namelen : 0;

  /* Keep all fixed-size state in the first IOB:
   * [msg_kbuf_s][msg_name][msg_control]. Place payload there too when it
   * fits; otherwise allocate payload from kmm.
   */

  fixed_len = sizeof(struct msg_kbuf_s) + name_len + cmsg_len;
  if (fixed_len > CONFIG_IOB_BUFSIZE)
    {
      /* CONFIG_IOB_BUFSIZE must include room for msg_kbuf_s plus all
       * fixed per-message data (msg_name + msg_control).
       */

      return NULL;
    }

  meta_iob = iob_alloc(false);
  if (meta_iob == NULL)
    {
      return NULL;
    }

  iob_reserve(meta_iob, sizeof(struct msg_kbuf_s));
  kbuf = (FAR struct msg_kbuf_s *)meta_iob->io_data;
  meta = IOB_DATA(meta_iob);

  /* Start with a shallow copy so non-pointer fields are correct. */

  memcpy(&kbuf->kmsg, src, sizeof(struct msghdr));
  kbuf->iob_meta    = meta_iob;
  kbuf->payload_kmem = NULL;

  if (has_name)
    {
      memset(meta, 0, name_len);
      kbuf->kmsg.msg_name = meta;
      meta += name_len;
    }

  if (has_cmsg)
    {
      memset(meta, 0, cmsg_len);
      kbuf->kmsg.msg_control = meta;
      meta += cmsg_len;
    }

  payload_room = CONFIG_IOB_BUFSIZE - fixed_len;

  if (has_payload)
    {
      kbuf->kmsg.msg_iov    = &kbuf->kiov;
      kbuf->kmsg.msg_iovlen = 1;

      if (payload_len <= payload_room)
        {
          kbuf->kiov.iov_base = meta;
          kbuf->kiov.iov_len  = payload_len;
        }
      else
        {
          kbuf->payload_kmem = kmm_malloc(payload_len);
          if (kbuf->payload_kmem == NULL)
            {
              iob_free(meta_iob);
              return NULL;
            }

          kbuf->kiov.iov_base = kbuf->payload_kmem;
          kbuf->kiov.iov_len  = payload_len;
        }
    }

  return &kbuf->kmsg;
}

/****************************************************************************
 * Name: msg_copy_from_user
 *
 * Description:
 *   Copy user-space send data into the kernel bounce buffers allocated by
 *   msg_alloc_kbuf().  Call this after alloc and before psock_sendmsg.
 *
 * Input Parameters:
 *   src  - Original user msghdr whose payload and control data to copy.
 *   kbuf - Kernel msghdr state filled by msg_alloc_kbuf().
 *
 ****************************************************************************/

void msg_copy_from_user(FAR const struct msghdr *src,
                        FAR struct msg_kbuf_s *kbuf)
{
  /* Copy iov payload from user to kernel IOB. */

  if (src->msg_iovlen > 0 && src->msg_iov != NULL &&
      src->msg_iov->iov_len > 0 && kbuf->kmsg.msg_iov != NULL)
    {
      memcpy(kbuf->kmsg.msg_iov->iov_base, src->msg_iov->iov_base,
             src->msg_iov->iov_len);
    }

  /* Copy ancillary control data from user to kernel IOB. */

  if (src->msg_controllen > 0 && src->msg_control != NULL &&
      kbuf->kmsg.msg_control != NULL)
    {
      memcpy(kbuf->kmsg.msg_control, src->msg_control,
             src->msg_controllen);
    }
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
 ****************************************************************************/

void msg_copy_to_user(FAR struct msghdr *dst,
                      FAR const struct msghdr *src,
                      ssize_t recvlen)
{
  FAR const struct msg_kbuf_s *ksrc = (FAR const struct msg_kbuf_s *)src;

  /* Only copy results back on success; on error the user buffers
   * must not be touched as they may contain partial or undefined data.
   */

  if (recvlen < 0)
    {
      return;
    }

  /* Copy received payload back to the user iov buffer. */

  if (recvlen > 0 && dst->msg_iov != NULL && dst->msg_iovlen > 0 &&
      ksrc->kmsg.msg_iov != NULL)
    {
      memcpy(dst->msg_iov->iov_base, ksrc->kmsg.msg_iov->iov_base, recvlen);
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
}

/****************************************************************************
 * Name: msg_free_kbuf
 *
 * Description:
 *   Release all kernel memory allocated by msg_alloc_kbuf().
 *   Safe to call with partially-initialised state (NULL pointers are
 *   ignored by iob_free).
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

  if (kbuf->payload_kmem != NULL)
    {
      kmm_free(kbuf->payload_kmem);
      kbuf->payload_kmem = NULL;
    }

  if (kbuf->iob_meta != NULL)
    {
      iob_free(kbuf->iob_meta);
    }
}

#endif /* CONFIG_BUILD_KERNEL && CONFIG_ARCH_ADDRENV */
