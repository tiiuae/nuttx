/****************************************************************************
 * syscall/syscall_uaccess.c
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

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <spawn.h>
#include <nuttx/spawn.h>
#include <sys/boardctl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>
#include <nuttx/pthread.h>
#include <nuttx/sched.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_I2C_DRIVER
#  include <nuttx/i2c/i2c_master.h>
#endif

#ifdef CONFIG_MMCSD
#  include <nuttx/mmcsd.h>
#endif

#ifdef CONFIG_NET
#  include <net/if.h>
#endif

#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif

#if defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_BUILD_PROTECTED)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool uaccess_arg(uintptr_t arg)
{
#ifdef CONFIG_BUILD_KERNEL
  return up_addrenv_user_vaddr(arg) ||
         up_addrenv_va_to_pa((FAR void *)arg) == 0;
#else
  /* An untyped argument carries either a pointer or a value, so only an
   * address the kernel would read is refused. A typed pointer reaches its
   * stub instead, which checks the whole range it names.
   */

  if (uaccess_ok((FAR const void *)arg, 1))
    {
      return true;
    }

#ifdef CONFIG_MM_KERNEL_HEAP
  if (kmm_heaprange((FAR const void *)arg, 1))
    {
      return false;
    }
#endif

  return arg < CONFIG_RAM_START ||
         arg >= CONFIG_RAM_START + CONFIG_RAM_SIZE;
#endif
}

static FAR struct iovec *uaccess_iov(FAR const struct iovec *iov,
                                     size_t iovcnt)
{
  FAR struct iovec *copy;
  FAR void *base;
  size_t size;
  size_t i;

  if (iovcnt > SIZE_MAX / sizeof(struct iovec))
    {
      return NULL;
    }

  size = iovcnt * sizeof(struct iovec);
  uaccess_check(iov, size);
  copy = kmm_malloc(size);
  if (copy == NULL)
    {
      return NULL;
    }

  memcpy(copy, iov, size);
  for (i = 0; i < iovcnt; i++)
    {
      base = copy[i].iov_base;
      if (copy[i].iov_len > 0 && !uaccess_ok(base, copy[i].iov_len))
        {
          kmm_free(copy);
          uaccess_fault(base);
        }
    }

  return copy;
}

static ssize_t uaccess_rw(ssize_t (*rw)(int, FAR const struct iovec *, int),
                          int fildes, FAR const struct iovec *iov,
                          int iovcnt)
{
  FAR struct iovec *copy;
  ssize_t ret;

  if (iovcnt <= 0)
    {
      return rw(fildes, iov, iovcnt);
    }

  copy = uaccess_iov(iov, iovcnt);
  if (copy == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  ret = rw(fildes, copy, iovcnt);
  kmm_free(copy);
  return ret;
}

static int uaccess_syslog(int priority, FAR const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start(ap, fmt);
  ret = nx_vsyslog(priority, fmt, &ap);
  va_end(ap);
  return ret;
}

#ifdef CONFIG_NET
static ssize_t uaccess_msg(int sockfd, FAR struct msghdr *msg, int flags,
                           bool recv)
{
  struct msghdr copy;
  ssize_t ret;

  if (msg == NULL)
    {
      return recv ? recvmsg(sockfd, NULL, flags) :
                    sendmsg(sockfd, NULL, flags);
    }

  uaccess_check(msg, sizeof(*msg));
  memcpy(&copy, msg, sizeof(copy));

  if (copy.msg_name != NULL)
    {
      uaccess_check(copy.msg_name, copy.msg_namelen);
    }

  if (copy.msg_control != NULL)
    {
      uaccess_check(copy.msg_control, copy.msg_controllen);
    }

  if (copy.msg_iovlen > 0)
    {
      copy.msg_iov = uaccess_iov(copy.msg_iov, copy.msg_iovlen);
      if (copy.msg_iov == NULL)
        {
          set_errno(ENOMEM);
          return ERROR;
        }
    }

  if (recv)
    {
      ret = recvmsg(sockfd, &copy, flags);
      msg->msg_namelen    = copy.msg_namelen;
      msg->msg_controllen = copy.msg_controllen;
      msg->msg_flags      = copy.msg_flags;
    }
  else
    {
      ret = sendmsg(sockfd, &copy, flags);
    }

  if (copy.msg_iovlen > 0)
    {
      kmm_free(copy.msg_iov);
    }

  return ret;
}
#endif

#ifdef CONFIG_NET
static int uaccess_ifconf(int fd, int req, unsigned long arg)
{
  union
  {
    struct ifconf ifc;
    struct lifconf lifc;
  } copy;

  FAR size_t *len;
  FAR void *buf;
  size_t size;
  int ret;

  size = req == SIOCGIFCONF ? sizeof(copy.ifc) : sizeof(copy.lifc);
  uaccess_check((FAR const void *)arg, size);
  memcpy(&copy, (FAR const void *)arg, size);

  if (req == SIOCGIFCONF)
    {
      len = &copy.ifc.ifc_len;
      buf = copy.ifc.ifc_buf;
    }
  else
    {
      len = &copy.lifc.lifc_len;
      buf = copy.lifc.lifc_buf;
    }

  if (*len > 0 && !uaccess_ok(buf, *len))
    {
      set_errno(EFAULT);
      return ERROR;
    }

  ret = ioctl(fd, req, (unsigned long)&copy);
  if (ret >= 0)
    {
      memcpy((FAR void *)arg, &copy, size);
    }

  return ret;
}
#endif

#ifdef CONFIG_MMCSD
static bool uaccess_mmccmd(FAR const struct mmc_ioc_cmd *cmd)
{
  uint64_t len = (uint64_t)cmd->blksz * cmd->blocks;

  return cmd->data_ptr == 0 ||
         uaccess_ok((FAR const void *)(uintptr_t)cmd->data_ptr,
                    len > 512 ? len : 512);
}

static int uaccess_mmc(int fd, int req, unsigned long arg)
{
  FAR struct mmc_ioc_cmd *cmds;
  FAR void *copy;
  uint64_t ncmds = 1;
  size_t size = sizeof(struct mmc_ioc_cmd);
  uint64_t i;
  int ret;

  if (req == MMC_IOC_MULTI_CMD)
    {
      uaccess_check((FAR const void *)arg, sizeof(uint64_t));
      ncmds = ((FAR struct mmc_ioc_multi_cmd *)arg)->num_of_cmds;
      if (ncmds > MMC_IOC_MAX_CMDS)
        {
          set_errno(EINVAL);
          return ERROR;
        }

      size = offsetof(struct mmc_ioc_multi_cmd, cmds) +
             ncmds * sizeof(struct mmc_ioc_cmd);
    }

  uaccess_check((FAR const void *)arg, size);
  copy = kmm_malloc(size);
  if (copy == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  memcpy(copy, (FAR const void *)arg, size);
  cmds = copy;
  if (req == MMC_IOC_MULTI_CMD)
    {
      ((FAR struct mmc_ioc_multi_cmd *)copy)->num_of_cmds = ncmds;
      cmds = ((FAR struct mmc_ioc_multi_cmd *)copy)->cmds;
    }

  for (i = 0; i < ncmds; i++)
    {
      if (!uaccess_mmccmd(&cmds[i]))
        {
          kmm_free(copy);
          set_errno(EFAULT);
          return ERROR;
        }
    }

  ret = ioctl(fd, req, (unsigned long)copy);
  if (ret >= 0)
    {
      memcpy((FAR void *)arg, copy, size);
    }

  kmm_free(copy);
  return ret;
}
#endif

#ifdef CONFIG_I2C_DRIVER
static int uaccess_i2c(int fd, FAR const struct i2c_transfer_s *utrans)
{
  struct i2c_transfer_s trans;
  FAR struct i2c_msg_s *msgv;
  size_t i;
  int ret;

  uaccess_check(utrans, sizeof(trans));
  memcpy(&trans, utrans, sizeof(trans));
  if (trans.msgc == 0 || trans.msgc > SIZE_MAX / sizeof(*msgv))
    {
      set_errno(EINVAL);
      return ERROR;
    }

  uaccess_check(trans.msgv, trans.msgc * sizeof(*msgv));
  msgv = kmm_malloc(trans.msgc * sizeof(*msgv));
  if (msgv == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  memcpy(msgv, trans.msgv, trans.msgc * sizeof(*msgv));
  for (i = 0; i < trans.msgc; i++)
    {
      if (msgv[i].length < 0 ||
          (msgv[i].length > 0 &&
           !uaccess_ok(msgv[i].buffer, msgv[i].length)))
        {
          kmm_free(msgv);
          set_errno(EFAULT);
          return ERROR;
        }
    }

  trans.msgv = msgv;
  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&trans);
  kmm_free(msgv);
  return ret;
}
#endif

#ifdef CONFIG_SPI_DRIVER
static int uaccess_spi(int fd, FAR const struct spi_sequence_s *useq)
{
  struct spi_sequence_s seq;
  FAR struct spi_trans_s *trans;
  size_t width;
  size_t i;
  int ret;

  uaccess_check(useq, sizeof(seq));
  memcpy(&seq, useq, sizeof(seq));
  if (seq.ntrans == 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  width = seq.nbits <= 8 ? 1 : seq.nbits <= 16 ? 2 : 4;
  uaccess_check(seq.trans, seq.ntrans * sizeof(*trans));
  trans = kmm_malloc(seq.ntrans * sizeof(*trans));
  if (trans == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  memcpy(trans, seq.trans, seq.ntrans * sizeof(*trans));
  for (i = 0; i < seq.ntrans; i++)
    {
      if (trans[i].nwords > SIZE_MAX / width ||
          (trans[i].nwords > 0 &&
           ((trans[i].txbuffer != NULL &&
             !uaccess_ok(trans[i].txbuffer, trans[i].nwords * width)) ||
            (trans[i].rxbuffer != NULL &&
             !uaccess_ok(trans[i].rxbuffer, trans[i].nwords * width)))))
        {
          kmm_free(trans);
          set_errno(EFAULT);
          return ERROR;
        }
    }

  seq.trans = trans;
  ret = ioctl(fd, SPIIOC_TRANSFER, (unsigned long)&seq);
  kmm_free(trans);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL
int uaccess_boardctl(unsigned int cmd, uintptr_t arg)
{
  if (!uaccess_arg(arg))
    {
      set_errno(EFAULT);
      return ERROR;
    }

  switch (cmd)
    {
#ifdef CONFIG_BOARDCTL_RESET
      case BOARDIOC_RESET:
#endif
#ifdef CONFIG_BOARDCTL_POWEROFF
      case BOARDIOC_POWEROFF:
#endif
        if (!nxsched_capable(PR_CAP_ADMIN))
          {
            set_errno(EPERM);
            return ERROR;
          }

        return boardctl(cmd, arg);

#ifdef CONFIG_BOARDCTL_ROMDISK
      case BOARDIOC_ROMDISK:
        set_errno(EPERM);
        return ERROR;
#endif

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
      case BOARDIOC_USBDEV_CONTROL:
        {
          struct boardioc_usbdev_ctrl_s ctrl;

          uaccess_check((FAR const void *)arg, sizeof(ctrl));
          memcpy(&ctrl, (FAR const void *)arg, sizeof(ctrl));
          if (ctrl.handle != NULL)
            {
              uaccess_check(ctrl.handle, sizeof(*ctrl.handle));
            }

          if (ctrl.action == BOARDIOC_USBDEV_DISCONNECT &&
              ctrl.usbdev != BOARDIOC_USBDEV_CDCACM)
            {
              set_errno(EPERM);
              return ERROR;
            }

          return boardctl(cmd, (uintptr_t)&ctrl);
        }
#endif

      default:
        return boardctl(cmd, arg);
    }
}
#endif

#ifndef CONFIG_BUILD_KERNEL

/* As many entries as the task setup will accept, so a vector without a
 * NULL is refused rather than walked until it faults.
 */

#define UACCESS_MAX_STRV 256

/* A kernel build copies these vectors through binfmt_copyargv(), which
 * checks each entry. A protected build hands them straight to the task
 * setup, so the strings are checked here instead.
 */

static bool uaccess_strv(FAR char * const *v)
{
  size_t i;

  if (v == NULL)
    {
      return true;
    }

  for (i = 0; i <= UACCESS_MAX_STRV; i++)
    {
      if (!uaccess_ok(&v[i], sizeof(FAR char *)))
        {
          return false;
        }

      if (v[i] == NULL)
        {
          return true;
        }

      if (!uaccess_ok(v[i], 1))
        {
          return false;
        }
    }

  return false;
}
#else
#  define uaccess_strv(v) true
#endif

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
int uaccess_execve(FAR const char *path, FAR char * const argv[],
                   FAR char * const envp[])
{
  if (!nxsched_capable(PR_CAP_SPAWN))
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (!uaccess_strv(argv) || !uaccess_strv(envp))
    {
      set_errno(EFAULT);
      return ERROR;
    }

  return execve(path, argv, envp);
}

int uaccess_posix_spawn(FAR pid_t *pid, FAR const char *path,
                        FAR const posix_spawn_file_actions_t *file_actions,
                        FAR const posix_spawnattr_t *attr,
                        FAR char * const argv[], FAR char * const envp[])
{
  if (!nxsched_capable(PR_CAP_SPAWN))
    {
      return EPERM;
    }

  if (!uaccess_strv(argv) || !uaccess_strv(envp))
    {
      return EFAULT;
    }

  return posix_spawn(pid, path, file_actions, attr, argv, envp);
}
#endif

#ifndef CONFIG_BUILD_KERNEL
int uaccess_task_spawn(FAR const char *name, main_t entry,
                       FAR const posix_spawn_file_actions_t *file_actions,
                       FAR const posix_spawnattr_t *attr,
                       FAR char * const argv[], FAR char * const envp[])
{
  if (!nxsched_capable(PR_CAP_SPAWN))
    {
      return -EPERM;
    }

  if (!uaccess_strv(argv) || !uaccess_strv(envp))
    {
      return -EFAULT;
    }

  return task_spawn(name, entry, file_actions, attr, argv, envp);
}
#endif

#ifndef CONFIG_DISABLE_MOUNTPOINT
int uaccess_mount(FAR const char *source, FAR const char *target,
                  FAR const char *filesystemtype, unsigned long mountflags,
                  FAR const void *data)
{
  if (!nxsched_capable(PR_CAP_RAWIO))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return mount(source, target, filesystemtype, mountflags, data);
}

int uaccess_umount2(FAR const char *target, unsigned int flags)
{
  if (!nxsched_capable(PR_CAP_RAWIO))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return umount2(target, flags);
}
#endif

static bool uaccess_owns(pid_t tid)
{
  FAR struct tcb_s *tcb;

  if (tid == 0 || nxsched_capable(PR_CAP_ADMIN))
    {
      return true;
    }

  tcb = nxsched_get_tcb(tid);
  return tcb == NULL || tcb->group == nxsched_self()->group;
}

int uaccess_kill(pid_t pid, int sig)
{
  if (sig != 0 && !uaccess_owns(pid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return kill(pid, sig);
}

int uaccess_tgkill(pid_t pid, pid_t tid, int sig)
{
  if (sig != 0 && !uaccess_owns(tid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return tgkill(pid, tid, sig);
}

#ifndef CONFIG_DISABLE_ALL_SIGNALS
int uaccess_sigqueue(int pid, int sig, union sigval value)
{
  if (sig != 0 && !uaccess_owns(pid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return sigqueue(pid, sig, value);
}
#endif

int uaccess_sched_setparam(pid_t pid, FAR const struct sched_param *param)
{
  if (!uaccess_owns(pid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return sched_setparam(pid, param);
}

int uaccess_sched_setscheduler(pid_t pid, int policy,
                               FAR const struct sched_param *param)
{
  if (!uaccess_owns(pid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return sched_setscheduler(pid, policy, param);
}

#ifdef CONFIG_SMP
int uaccess_sched_setaffinity(pid_t pid, size_t cpusetsize,
                              FAR const cpu_set_t *mask)
{
  if (!uaccess_owns(pid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  return sched_setaffinity(pid, cpusetsize, mask);
}
#endif

#ifndef CONFIG_DISABLE_PTHREAD
int uaccess_pthread_cancel(pthread_t thread)
{
  return uaccess_owns((pid_t)thread) ? pthread_cancel(thread) : EPERM;
}

int uaccess_pthread_setschedparam(pthread_t thread, int policy,
                                  FAR const struct sched_param *param)
{
  return uaccess_owns((pid_t)thread) ?
         pthread_setschedparam(thread, policy, param) : EPERM;
}

int uaccess_pthread_setschedprio(pthread_t thread, int prio)
{
  return uaccess_owns((pid_t)thread) ?
         pthread_setschedprio(thread, prio) : EPERM;
}

#ifdef CONFIG_SMP
int uaccess_pthread_setaffinity_np(pthread_t thread, size_t cpusetsize,
                                   FAR const cpu_set_t *cpuset)
{
  return uaccess_owns((pid_t)thread) ?
         pthread_setaffinity_np(thread, cpusetsize, cpuset) : EPERM;
}
#endif
#endif

int uaccess_fcntl(int fd, int cmd, ...)
{
  uintptr_t arg;
  va_list ap;

  va_start(ap, cmd);
  arg = va_arg(ap, uintptr_t);
  va_end(ap);

  switch (cmd)
    {
      case F_GETLK:
      case F_SETLK:
      case F_SETLKW:
        uaccess_check((FAR const void *)arg, sizeof(struct flock));
        break;

      case F_GETPATH:
        uaccess_check((FAR const void *)arg, PATH_MAX);
        break;

      default:
        if (!uaccess_arg(arg))
          {
            set_errno(EFAULT);
            return ERROR;
          }
        break;
    }

  return fcntl(fd, cmd, arg);
}

int uaccess_ioctl(int fd, int req, ...)
{
  unsigned long arg;
  va_list ap;

  va_start(ap, req);
  arg = va_arg(ap, unsigned long);
  va_end(ap);

  switch (req)
    {
      case BIOC_XIPBASE:
      case DIOC_GETPRIV:
#ifdef CONFIG_CDCACM
      case CAIOC_REGISTERCB:
#endif
        set_errno(EPERM);
        return ERROR;

#ifdef CONFIG_NET
      case SIOCGIFCONF:
      case SIOCGLIFCONF:
        return uaccess_ifconf(fd, req, arg);
#endif

#ifdef CONFIG_MMCSD
      case MMC_IOC_CMD:
      case MMC_IOC_MULTI_CMD:
        return uaccess_mmc(fd, req, arg);
#endif

#ifdef CONFIG_I2C_DRIVER
      case I2CIOC_TRANSFER:
        return uaccess_i2c(fd, (FAR const struct i2c_transfer_s *)arg);
#endif

#ifdef CONFIG_SPI_DRIVER
      case SPIIOC_TRANSFER:
        return uaccess_spi(fd, (FAR const struct spi_sequence_s *)arg);
#endif

      default:
        break;
    }

  if (!uaccess_arg(arg))
    {
      set_errno(EFAULT);
      return ERROR;
    }

  return ioctl(fd, req, arg);
}

#ifndef CONFIG_DISABLE_PTHREAD
int uaccess_nx_pthread_create(pthread_trampoline_t trampoline,
                              FAR pthread_t *thread,
                              FAR const pthread_attr_t *attr,
                              pthread_startroutine_t entry,
                              pthread_addr_t arg)
{
  pthread_attr_t copy;

  if (attr == NULL)
    {
      return nx_pthread_create(trampoline, thread, NULL, entry, arg);
    }

  memcpy(&copy, attr, sizeof(copy));
  if (copy.stackaddr != NULL)
    {
      uaccess_check(copy.stackaddr, copy.stacksize);
    }

  return nx_pthread_create(trampoline, thread, &copy, entry, arg);
}
#endif

int uaccess_nx_vsyslog(int priority, FAR const IPTR char *src,
                       FAR va_list *ap)
{
  return uaccess_syslog(priority, "%s", src);
}

int uaccess_prctl(int option, ...)
{
  uintptr_t arg1;
  uintptr_t arg2;
  va_list ap;

  va_start(ap, option);
  arg1 = va_arg(ap, uintptr_t);
  arg2 = va_arg(ap, uintptr_t);
  va_end(ap);

  if (option == PR_SET_NAME_EXT && !uaccess_owns((pid_t)arg2))
    {
      set_errno(EPERM);
      return ERROR;
    }

  switch (option)
    {
      case PR_SET_NAME:
      case PR_GET_NAME:
      case PR_SET_NAME_EXT:
      case PR_GET_NAME_EXT:
        uaccess_check((FAR const void *)arg1, CONFIG_TASK_NAME_SIZE + 1);
        break;

      default:
        break;
    }

  return prctl(option, arg1, arg2);
}

ssize_t uaccess_readv(int fildes, FAR const struct iovec *iov, int iovcnt)
{
  return uaccess_rw(readv, fildes, iov, iovcnt);
}

ssize_t uaccess_writev(int fildes, FAR const struct iovec *iov, int iovcnt)
{
  return uaccess_rw(writev, fildes, iov, iovcnt);
}

#ifdef CONFIG_NET
ssize_t uaccess_recvmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  return uaccess_msg(sockfd, msg, flags, true);
}

ssize_t uaccess_sendmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  return uaccess_msg(sockfd, msg, flags, false);
}
#endif

#endif /* CONFIG_BUILD_KERNEL || CONFIG_BUILD_PROTECTED */
