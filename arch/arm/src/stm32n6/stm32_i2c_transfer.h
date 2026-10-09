/****************************************************************************
 * arch/arm/src/stm32n6/stm32_i2c_transfer.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_I2C_TRANSFER_H
#define __ARCH_ARM_SRC_STM32N6_STM32_I2C_TRANSFER_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/i2c/i2c_master.h>

struct stm32_i2c_message_vector_s
{
  uint32_t frequency;
  size_t payload_bytes;
  size_t address_phases;
  size_t stop_phases;
};

static inline int stm32_i2c_validate_messages(
    const struct i2c_msg_s *msgs, int count,
    struct stm32_i2c_message_vector_s *vector)
{
  const uint16_t allowed = I2C_M_READ | I2C_M_TEN | I2C_M_NOSTOP |
                           I2C_M_NOSTART;
  size_t payload_bytes = 0;
  size_t address_phases = 0;
  size_t stop_phases = 0;
  uint32_t frequency;
  int i;

  if (msgs == NULL || vector == NULL || count <= 0)
    {
      return -EINVAL;
    }

  frequency = msgs[0].frequency;
  if (frequency != I2C_SPEED_STANDARD && frequency != I2C_SPEED_FAST)
    {
      return frequency == 0 ? -EINVAL : -ENOTSUP;
    }

  for (i = 0; i < count; i++)
    {
      const struct i2c_msg_s *msg = &msgs[i];
      size_t length;

      if ((msg->flags & I2C_M_TEN) != 0 ||
          (msg->flags & ~allowed) != 0)
        {
          return -ENOTSUP;
        }

      if (msg->addr > 0x7fu || msg->frequency != frequency ||
          msg->length < 0 ||
          (msg->length > 0 && msg->buffer == NULL) ||
          (msg->length == 0 && (msg->flags & I2C_M_READ) != 0))
        {
          return -EINVAL;
        }

      if (i == 0 && (msg->flags & I2C_M_NOSTART) != 0)
        {
          return -EINVAL;
        }

      if (i == count - 1 && (msg->flags & I2C_M_NOSTOP) != 0)
        {
          return -ENOTSUP;
        }

      if (i > 0 && (msg->flags & I2C_M_NOSTART) != 0)
        {
          const struct i2c_msg_s *previous = &msgs[i - 1];
          bool read = (msg->flags & I2C_M_READ) != 0;
          bool previous_read = (previous->flags & I2C_M_READ) != 0;

          if (msg->addr != previous->addr || read != previous_read ||
              msg->frequency != previous->frequency ||
              msg->length == 0 || previous->length == 0)
            {
              return -EINVAL;
            }
        }

      length = (size_t)msg->length;
      if (length > SIZE_MAX - payload_bytes)
        {
          return -EOVERFLOW;
        }

      payload_bytes += length;
      if ((msg->flags & I2C_M_NOSTART) == 0)
        {
          address_phases++;
        }

      if (i == count - 1 ||
          ((msg->flags & I2C_M_NOSTOP) == 0 &&
           (msgs[i + 1].flags & I2C_M_NOSTART) == 0))
        {
          stop_phases++;
        }
    }

  vector->frequency = frequency;
  vector->payload_bytes = payload_bytes;
  vector->address_phases = address_phases;
  vector->stop_phases = stop_phases;
  return 0;
}

static inline uint8_t stm32_i2c_message_block_size(size_t remaining)
{
  return remaining > 255u ? 255u : (uint8_t)remaining;
}

static inline bool stm32_i2c_message_reload(
    const struct i2c_msg_s *msgs, int count, int index, size_t remaining,
    uint8_t block_size)
{
  return remaining > block_size ||
         (index + 1 < count &&
          (msgs[index + 1].flags & I2C_M_NOSTART) != 0);
}

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_I2C_TRANSFER_H */
