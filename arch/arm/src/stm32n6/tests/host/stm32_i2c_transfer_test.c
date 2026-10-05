/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_i2c_transfer_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../stm32_i2c_transfer.h"

#define CHECK(condition) \
  do \
    { \
      if (!(condition)) \
        { \
          fprintf(stderr, "%s:%d: check failed: %s\n", \
                  __FILE__, __LINE__, #condition); \
          return EXIT_FAILURE; \
        } \
    } \
  while (0)

static struct i2c_msg_s message(uint8_t *buffer, ssize_t length,
                               uint16_t flags, uint16_t address)
{
  struct i2c_msg_s msg =
  {
    .frequency = I2C_SPEED_STANDARD,
    .addr = address,
    .flags = flags,
    .buffer = buffer,
    .length = length
  };

  return msg;
}

static int test_valid_vectors(void)
{
  uint8_t payload[511] = {0};
  struct stm32_i2c_message_vector_s vector;
  struct i2c_msg_s msgs[3];

  msgs[0] = message(payload, 1, 0, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == 0);
  CHECK(vector.frequency == I2C_SPEED_STANDARD);
  CHECK(vector.payload_bytes == 1);
  CHECK(vector.address_phases == 1 && vector.stop_phases == 1);

  msgs[0] = message(NULL, 0, 0, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == 0);

  msgs[0] = message(payload, 1, I2C_M_NOSTOP, 0x76);
  msgs[1] = message(payload + 1, 2, I2C_M_READ, 0x77);
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == 0);
  CHECK(vector.address_phases == 2 && vector.stop_phases == 1);

  msgs[0] = message(payload, 2, 0, 0x76);
  msgs[1] = message(payload + 2, 3, I2C_M_NOSTART, 0x76);
  msgs[2] = message(payload + 5, 4, I2C_M_NOSTART, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 3, &vector) == 0);
  CHECK(vector.payload_bytes == 9);
  CHECK(vector.address_phases == 1 && vector.stop_phases == 1);

  msgs[0] = message(payload, 1, 0, 0x76);
  msgs[1] = message(payload + 1, 1, 0, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == 0);
  CHECK(vector.address_phases == 2 && vector.stop_phases == 2);

  msgs[0] = message(payload, 255, 0, 0x76);
  CHECK(stm32_i2c_message_block_size(254) == 254);
  CHECK(stm32_i2c_message_block_size(255) == 255);
  CHECK(stm32_i2c_message_block_size(256) == 255);
  CHECK(!stm32_i2c_message_reload(msgs, 1, 0, 255, 255));
  CHECK(stm32_i2c_message_reload(msgs, 1, 0, 256, 255));
  msgs[1] = message(payload + 255, 1, I2C_M_NOSTART, 0x76);
  CHECK(stm32_i2c_message_reload(msgs, 2, 0, 255, 255));

  msgs[0].frequency = I2C_SPEED_FAST;
  msgs[1].frequency = I2C_SPEED_FAST;
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == 0);
  CHECK(vector.frequency == I2C_SPEED_FAST);
  return EXIT_SUCCESS;
}

static int test_invalid_vectors(void)
{
  uint8_t payload[2] = {0};
  struct stm32_i2c_message_vector_s vector;
  struct i2c_msg_s msgs[2];

  msgs[0] = message(payload, 1, 0, 0x76);
  CHECK(stm32_i2c_validate_messages(NULL, 1, &vector) == -EINVAL);
  CHECK(stm32_i2c_validate_messages(msgs, 0, &vector) == -EINVAL);
  CHECK(stm32_i2c_validate_messages(msgs, 1, NULL) == -EINVAL);

  msgs[0].flags = I2C_M_NOSTART;
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -EINVAL);
  msgs[0].flags = I2C_M_NOSTOP;
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -ENOTSUP);
  msgs[0].flags = I2C_M_TEN;
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -ENOTSUP);
  msgs[0].flags = 0x0100;
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -ENOTSUP);

  msgs[0] = message(payload, 1, 0, 0x80);
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -EINVAL);
  msgs[0] = message(payload, -1, 0, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -EINVAL);
  msgs[0] = message(NULL, 1, 0, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -EINVAL);
  msgs[0] = message(NULL, 0, I2C_M_READ, 0x76);
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -EINVAL);
  msgs[0] = message(payload, 1, 0, 0x76);
  msgs[0].frequency = 0;
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -EINVAL);
  msgs[0].frequency = I2C_SPEED_FAST_PLUS;
  CHECK(stm32_i2c_validate_messages(msgs, 1, &vector) == -ENOTSUP);

  msgs[0] = message(payload, 1, 0, 0x76);
  msgs[1] = message(payload + 1, 1, I2C_M_NOSTART, 0x77);
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == -EINVAL);
  msgs[1].addr = 0x76;
  msgs[1].flags |= I2C_M_READ;
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == -EINVAL);
  msgs[1].flags = I2C_M_NOSTART;
  msgs[1].frequency = I2C_SPEED_FAST;
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == -EINVAL);
  msgs[1].frequency = I2C_SPEED_STANDARD;
  msgs[1].length = 0;
  msgs[1].buffer = NULL;
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == -EINVAL);
  msgs[1] = message(payload + 1, 1, 0, 0x76);
  msgs[1].frequency = I2C_SPEED_FAST;
  CHECK(stm32_i2c_validate_messages(msgs, 2, &vector) == -EINVAL);
  return EXIT_SUCCESS;
}

int main(void)
{
  if (test_valid_vectors() != EXIT_SUCCESS ||
      test_invalid_vectors() != EXIT_SUCCESS)
    {
      return EXIT_FAILURE;
    }

  puts("STM32N6 I2C transfer contract tests passed");
  return EXIT_SUCCESS;
}
