/****************************************************************************
 * STM32N6 host-test NuttX I2C API subset
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __TEST_NUTTX_I2C_MASTER_H
#define __TEST_NUTTX_I2C_MASTER_H

#include <stdint.h>
#include <sys/types.h>

#define I2C_M_READ       0x0001
#define I2C_M_TEN        0x0002
#define I2C_M_NOSTOP     0x0040
#define I2C_M_NOSTART    0x0080

#define I2C_SPEED_STANDARD   100000
#define I2C_SPEED_FAST       400000
#define I2C_SPEED_FAST_PLUS  1000000

struct i2c_msg_s
{
  uint32_t frequency;
  uint16_t addr;
  uint16_t flags;
  uint8_t *buffer;
  ssize_t length;
};

#endif
