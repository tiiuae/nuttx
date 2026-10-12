/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_i2c_board.c
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
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>

#include "stm32_i2c.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_i2c_lock = NXMUTEX_INITIALIZER;
static struct i2c_master_s *g_i2c2;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nucleo_i2c_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  ret = nxmutex_lock(&g_i2c_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_i2c2 != NULL)
    {
      nxmutex_unlock(&g_i2c_lock);
      return OK;
    }

  i2c = stm32_i2cbus_initialize(2);
  if (i2c == NULL)
    {
      nxmutex_unlock(&g_i2c_lock);
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  {
    ret = i2c_register(i2c, 2);
    if (ret < 0)
      {
        int uninit_ret = stm32_i2cbus_uninitialize(i2c);

        if (uninit_ret < 0)
          {
            syslog(LOG_ERR,
                   "ERROR: I2C2 uninitialize after registration "
                   "failure: %d\n",
                   uninit_ret);
          }

        nxmutex_unlock(&g_i2c_lock);
        return ret;
      }
  }
#endif

  g_i2c2 = i2c;
  nxmutex_unlock(&g_i2c_lock);
  return OK;
}

struct i2c_master_s *nucleo_i2c2_bus(void)
{
  struct i2c_master_s *i2c;
  int ret = nxmutex_lock(&g_i2c_lock);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: I2C2 board lock failed: %d\n", ret);
      return NULL;
    }

  i2c = g_i2c2;
  nxmutex_unlock(&g_i2c_lock);
  return i2c;
}
