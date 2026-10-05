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

#include "stm32_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nucleo_i2c_initialize(void)
{
  struct i2c_master_s *i2c;

  i2c = stm32_i2cbus_initialize(2);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  {
    int ret = i2c_register(i2c, 2);
    if (ret < 0)
      {
        int uninit_ret = stm32_i2cbus_uninitialize(i2c);
        if (uninit_ret < 0)
          {
            syslog(LOG_ERR,
                   "ERROR: I2C2 uninitialize after registration failure: %d\n",
                   uninit_ret);
          }

        return ret;
      }
  }
#endif

  return 0;
}
