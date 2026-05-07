/****************************************************************************
 * arch/arm64/src/imx9/imx9_pmic.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <debug.h>
#include <sys/types.h>
#include <syslog.h>
#include <errno.h>
#include <nuttx/i2c/i2c_master.h>
#include "imx9_lpi2c.h"
#include "chip.h"
#include "arm64_internal.h"
#include "imx9_trdc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCA9451A in IMX9 platform */

#define PCA9451A_I2C_ADDR       0x25
#define REG_SW_RST              0x06
#define REG_POWERON_STAT        0x05
#define REG_RESET_CTRL          0x08
#define COLD_RESET              0x64
#define WARM_RESET              0x35
#define PMIC_RESET_WAIT_US      1000
#define PMIC_TRY_COUNT             3

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

struct i2c_master_s *imx9_i2cbus_initialize_forced_polling(int port);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_pmic_reg_read
 *
 * Description:
 *   Read 8-bit register value from pmic
 *
 ****************************************************************************/

static int imx9_pmic_reg_read(uint8_t reg, uint8_t *value)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msgs[2];
  uint8_t reg_addr = reg;
  int trycount;
  int ret = 0;

  if (!value)
    {
      i2cerr("Invalid value parameter\n");
      return -EINVAL;
    }

  i2c = imx9_i2cbus_initialize(CONFIG_IMX9_PMIC_I2C);

  if (i2c == NULL)
    {
      i2cerr("Failed to initialize I2C bus\n");
      return -ENODEV;
    }

  msgs[0].frequency = 400000;
  msgs[0].addr      = PCA9451A_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg_addr;
  msgs[0].length    = 1;

  msgs[1].frequency = 400000;
  msgs[1].addr      = PCA9451A_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = value;
  msgs[1].length    = 1;

  trycount = PMIC_TRY_COUNT;
  while (trycount--)
    {
      ret = I2C_TRANSFER(i2c, msgs, 2);
      if (ret == 0)
        {
          break;
        }

      i2cerr("I2C transfer failed: %d\n", ret);
#ifdef CONFIG_I2C_RESET
      I2C_RESET(i2c);
#endif
    }

  imx9_i2cbus_uninitialize(i2c);

  return ret < 0 ? ret : 0; /* negative errno or 0 */
}

/****************************************************************************
 * Name: imx9_pmic_reg_write
 *
 * Description:
 *   Write 8-bit register value to pmic
 *
 ****************************************************************************/

static int imx9_pmic_reg_write(uint8_t reg, uint8_t val)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int trycount;
  int ret = 0;

  i2c = imx9_i2cbus_initialize(CONFIG_IMX9_PMIC_I2C);
  if (i2c == NULL)
    {
      i2cerr("Failed to initialize I2C bus\n");
      return -ENODEV;
    }

  buffer[0] = reg;
  buffer[1] = val;

  msg.frequency = 400000;
  msg.addr      = PCA9451A_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  trycount = PMIC_TRY_COUNT;
  while (trycount--)
    {
      ret = I2C_TRANSFER(i2c, &msg, 1);
      if (ret == 0)
        {
          break;
        }

      i2cerr("I2C transfer failed: %d\n", ret);
#ifdef CONFIG_I2C_RESET
      I2C_RESET(i2c);
#endif
    }

  imx9_i2cbus_uninitialize(i2c);

  return ret < 0 ? ret : 0; /* negative errno or 0 */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_pmic_reset
 *
 * Description:
 *   Reset SoC via pmic
 *
 * Returned Value:
 *   Zero on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_pmic_reset(void)
{
  return imx9_pmic_reg_write(REG_SW_RST, COLD_RESET);
}

/****************************************************************************
 * Name: imx9_pmic_get_reset_reason
 *
 * Description:
 *   Read reset reason from pmic
 *
 * Returned Value:
 *   Zero on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_pmic_get_reset_reason(uint8_t *value)
{
  return imx9_pmic_reg_read(REG_POWERON_STAT, value);
}

/****************************************************************************
 * Name: imx9_pmic_get_reset_ctrl
 *
 * Description:
 *  Read reset control register value
 *
 * Returned Value:
 *   Zero on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_pmic_get_reset_ctrl(uint8_t *value)
{
  return imx9_pmic_reg_read(REG_RESET_CTRL, value);
}

/****************************************************************************
 * Name: imx9_pmic_set_reset_ctrl
 *
 * Description:
 *  Set reset control register value
 *
 * Input Parameters:
 *   Register value
 *
 * Returned Value:
 *   Zero on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_pmic_set_reset_ctrl(uint8_t val)
{
  return imx9_pmic_reg_write(REG_RESET_CTRL, val);
}

/****************************************************************************
 * Name: imx9_pmic_forced_reset_with_ctrl
 *
 * Description:
 *  Force the polling mode transfer, set reset control register and
 *  reset SoC via pmic.
 *  This can be called from kernel panic handler.
 *
 * Input Parameters:
 *   Reset Control register value
 *
 * Returned Value:
 *   Function does not return
 *
 ****************************************************************************/

noreturn_function void imx9_pmic_forced_reset_with_ctrl(uint8_t val)
{
  int ret = 1;
  struct i2c_master_s *i2c;

  enter_critical_section();

  /* Forced reset, loop until success */

  while (ret)
    {
      /* Busy-wait here to make sure previous i2c transfer has
       * been ended before the forced pmic reset is performed.
       */

      up_udelay(PMIC_RESET_WAIT_US);

      i2c = imx9_i2cbus_initialize_forced_polling(
                                                  CONFIG_IMX9_PMIC_I2C);
      if (i2c == NULL)
        {
          /* Forced initialization failed, try again */

          continue;
        }

      ret = imx9_pmic_reg_write(REG_RESET_CTRL, val);
      if (ret < 0)
        {
          imx9_i2cbus_uninitialize(i2c);
          continue;
        }

      ret = imx9_pmic_reg_write(REG_SW_RST, COLD_RESET);
      if (ret < 0)
        {
          imx9_i2cbus_uninitialize(i2c);
        }
    }

  imx9_i2cbus_uninitialize(i2c);

  while (1);
}
