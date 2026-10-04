/****************************************************************************
 * drivers/crypto/se05x.c
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

/* Copyright 2023 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pnt/pnt_se05x_api.h"
#include "se05x_internal.h"
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/crypto/se05x.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <string.h>
#include <strings.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CRYPTO_CONTROLSE
#warning Controlse is not available; This is probably not what you want.
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int se05x_open(FAR struct file *filep);
static int se05x_close(FAR struct file *filep);
static ssize_t se05x_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t se05x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int se05x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct se05x_dev_s *g_se05x;

static const FAR struct file_operations g_fops =
{
    se05x_open, se05x_close, se05x_read, se05x_write,
    NULL,       se05x_ioctl, NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int se05x_open(FAR struct file *filep)
{
#ifndef CONFIG_BUILD_FLAT
  if ((nxsched_self()->flags & TCB_FLAG_SYSCALL) != 0)
    {
      return -EPERM;
    }
#endif

  return OK;
}

static int se05x_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t se05x_read(FAR struct file *filep, char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

static ssize_t se05x_write(FAR struct file *filep, const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

static int se05x_dispatch(FAR struct se05x_dev_s *priv, int cmd,
                          unsigned long arg)
{
  int ret = -ENOTTY;

  switch (cmd)
    {
    case SEIOC_GET_INFO:
      {
        FAR struct se05x_info_s *info = (FAR struct se05x_info_s *)arg;
        ret = pnt_se05x_get_info(priv, info);
      }
      break;

    case SEIOC_GET_VERSION:
      {
        FAR struct se05x_version_s *version =
          (FAR struct se05x_version_s *)arg;
        ret = pnt_se05x_get_version(priv, version);
      }
      break;

    case SEIOC_GET_UID:
      {
        FAR struct se05x_uid_s *uid = (FAR struct se05x_uid_s *)arg;
        ret = pnt_se05x_get_uid(priv, uid);
      }
      break;

    case SEIOC_GENERATE_KEYPAIR:
      {
        FAR struct se05x_generate_keypair_s *generate_keypair_args =
            (FAR struct se05x_generate_keypair_s *)arg;
        ret = pnt_se05x_generate_keypair(priv, generate_keypair_args);
      }
      break;

    case SEIOC_SET_KEY:
      {
        FAR struct se05x_key_transmission_s *set_key_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_set_public_key(priv, set_key_args);
      }
      break;

    case SEIOC_SET_DATA:
      {
        FAR struct se05x_key_transmission_s *set_key_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_set_data(priv, set_key_args);
      }
      break;

    case SEIOC_GET_KEY:
      {
        FAR struct se05x_key_transmission_s *get_key_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_get_key(priv, get_key_args);
      }
      break;

    case SEIOC_GET_DATA:
      {
        FAR struct se05x_key_transmission_s *get_data_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_get_data(priv, get_data_args);
      }
      break;

    case SEIOC_DELETE_KEY:
      {
        ret = pnt_se05x_delete_key(priv, arg);
      }
      break;

    case SEIOC_DERIVE_SYMM_KEY:
      {
        FAR struct se05x_derive_key_s *derive_key_args =
            (FAR struct se05x_derive_key_s *)arg;
        ret = pnt_se05x_derive_key(priv, derive_key_args);
      }
      break;

    case SEIOC_CREATE_SIGNATURE:
      {
        FAR struct se05x_signature_s *create_signature_args =
            (FAR struct se05x_signature_s *)arg;
        ret = pnt_se05x_create_signature(priv, create_signature_args);
      }
      break;

    case SEIOC_ROTATE_SCP03:
      {
        FAR const struct se05x_scp03_keys_s *keys =
            (FAR const struct se05x_scp03_keys_s *)arg;
        ret = pnt_se05x_rotate_scp03(priv, keys);
      }
      break;

    case SEIOC_PLATFORM_SCP:
      {
        ret = pnt_se05x_platform_scp(priv, arg != 0);
      }
      break;

    case SEIOC_VERIFY_SIGNATURE:
      {
        FAR struct se05x_signature_s *verify_signature_args =
            (FAR struct se05x_signature_s *)arg;
        ret = pnt_se05x_verify_signature(priv, verify_signature_args);
      }
      break;

    default:
      crypterr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

static int se05x_worker(int argc, FAR char *argv[])
{
  FAR struct se05x_dev_s *priv = g_se05x;

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->request);

      if (priv->pnt == NULL && pnt_se05x_open(priv) < 0)
        {
          priv->result = -EIO;
        }
      else
        {
          priv->result = se05x_dispatch(priv, priv->cmd, priv->arg);
          if (priv->result == -EIO)
            {
              pnt_se05x_close(priv);
            }
        }

      nxsem_post(&priv->done);
    }

  return OK;
}

static int se05x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return se05x_kioctl(cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int se05x_kioctl(int cmd, unsigned long arg)
{
  FAR struct se05x_dev_s *priv = g_se05x;
  int ret;

  if (priv == NULL)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&priv->mutex);
  if (ret < 0)
    {
      return ret;
    }

  priv->cmd = cmd;
  priv->arg = arg;
  nxsem_post(&priv->request);
  nxsem_wait_uninterruptible(&priv->done);
  ret = priv->result;
  nxmutex_unlock(&priv->mutex);
  return ret;
}

int se05x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   FAR struct se05x_config_s *config)
{
  int ret;

  FAR struct se05x_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  if (g_se05x != NULL)
    {
      return -EEXIST;
    }

  priv = (FAR struct se05x_dev_s *)kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      crypterr("ERROR: Failed to allocate instance\n");
      ret = -ENOMEM;
      goto errout;
    }

  priv->config = config;
  priv->i2c = i2c;
  priv->pnt = NULL;

#ifdef CONFIG_DEV_SE05X_SCP03
  if (config->scp03 == NULL)
    {
      ret = -EINVAL;
      goto errout_with_alloc;
    }

  memcpy(&priv->scp03, config->scp03, sizeof(priv->scp03));
#endif

  /* Check se05x availability */

  ret = pnt_se05x_open(priv);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to open se05x driver: %d\n", ret);
      ret = -ENODEV;
      goto errout_with_alloc;
    }

  struct se05x_uid_s uid;
  ret = pnt_se05x_get_uid(priv, &uid);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to probe se05x driver: %d\n", ret);
      ret = -ENODEV;
      goto errout_with_alloc_and_open;
    }

  nxmutex_init(&priv->mutex);
  nxsem_init(&priv->request, 0, 0);
  nxsem_init(&priv->done, 0, 0);
  g_se05x = priv;

  ret = kthread_create("se05x", CONFIG_DEV_SE05X_PRIORITY,
                       CONFIG_DEV_SE05X_STACKSIZE, se05x_worker, NULL);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to start the worker: %d\n", ret);
      goto errout_with_sync;
    }

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to register driver: %d\n", ret);
    }

  return OK;

errout_with_sync:
  g_se05x = NULL;
  nxsem_destroy(&priv->done);
  nxsem_destroy(&priv->request);
  nxmutex_destroy(&priv->mutex);

errout_with_alloc_and_open:
  pnt_se05x_close(priv);

errout_with_alloc:
  explicit_bzero(priv, sizeof(*priv));
  kmm_free(priv);

errout:
  return ret;
}
