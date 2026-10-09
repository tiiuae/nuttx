/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_i2c_timing_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../stm32_i2c_timing.h"

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

static int check_timing(const struct stm32_i2c_timing_input_s *input,
                        struct stm32_i2c_timing_result_s *result)
{
  uint64_t kernel_period_max_ps = stm32_i2c_timing_period_max(
      input->kernel_frequency_hz, input->clock_tolerance_ppm);
  uint64_t kernel_period_min_ps = stm32_i2c_timing_period_min(
      input->kernel_frequency_hz, input->clock_tolerance_ppm);
  uint64_t tpresc_min_ps =
      kernel_period_min_ps * ((uint64_t)result->prescaler + 1u);
  uint64_t tpresc_max_ps =
      kernel_period_max_ps * ((uint64_t)result->prescaler + 1u);
  uint64_t rise_ps = (uint64_t)input->rise_time_ns * 1000u;
  uint64_t fall_ps = (uint64_t)input->fall_time_ns * 1000u;
  uint64_t af_min_ps = (uint64_t)input->analog_filter_min_ns * 1000u;
  uint64_t af_max_ps = (uint64_t)input->analog_filter_max_ns * 1000u;
  uint64_t hold_max_ps;
  uint64_t valid_max_ps;
  uint64_t setup_min_ps;
  uint64_t low_min_ps;
  uint64_t high_min_ps;
  uint64_t sdadel_min_ps;
  uint64_t sdadel_lower_bound_ps;
  uint64_t sync_min_ps;
  uint64_t period_min_ps;
  uint64_t sdadel_ps;

  if (input->frequency_hz == 100000u)
    {
      setup_min_ps = 250000u;
      hold_max_ps = 3450000u;
      valid_max_ps = 3450000u;
      low_min_ps = 4700000u;
      high_min_ps = 4000000u;
    }
  else
    {
      setup_min_ps = 100000u;
      hold_max_ps = 900000u;
      valid_max_ps = 900000u;
      low_min_ps = 1300000u;
      high_min_ps = 600000u;
    }

  sdadel_ps = (uint64_t)result->sda_delay * tpresc_max_ps;
  sdadel_lower_bound_ps = af_min_ps +
      (uint64_t)(input->digital_filter + 3u) * kernel_period_min_ps;
  sdadel_min_ps = fall_ps > sdadel_lower_bound_ps ?
                  fall_ps - sdadel_lower_bound_ps : 0;
  sync_min_ps = 2u * af_min_ps +
      (uint64_t)(2u * input->digital_filter + 4u) * kernel_period_min_ps;
  period_min_ps = ((uint64_t)result->scl_low + result->scl_high + 2u) *
      tpresc_min_ps + sync_min_ps;

  CHECK(result->prescaler <= 15u);
  CHECK(result->scl_delay <= 15u);
  CHECK(result->sda_delay <= 15u);
  CHECK(result->scl_high <= 255u);
  CHECK(result->scl_low <= 255u);
  CHECK(result->maximum_scl_hz <= input->frequency_hz);
  CHECK((((result->timingr & I2C_TIMINGR_PRESC_MASK) >>
          I2C_TIMINGR_PRESC_SHIFT) == result->prescaler));
  CHECK((((result->timingr & I2C_TIMINGR_SCLDEL_MASK) >>
          I2C_TIMINGR_SCLDEL_SHIFT) == result->scl_delay));
  CHECK((((result->timingr & I2C_TIMINGR_SDADEL_MASK) >>
          I2C_TIMINGR_SDADEL_SHIFT) == result->sda_delay));
  CHECK((((result->timingr & I2C_TIMINGR_SCLH_MASK) >>
          I2C_TIMINGR_SCLH_SHIFT) == result->scl_high));
  CHECK((((result->timingr & I2C_TIMINGR_SCLL_MASK) >>
          I2C_TIMINGR_SCLL_SHIFT) == result->scl_low));

  CHECK(((uint64_t)result->scl_delay + 1u) * tpresc_min_ps >=
        rise_ps + setup_min_ps);
  CHECK(((uint64_t)result->scl_low + 1u) * tpresc_min_ps >= low_min_ps);
  CHECK(((uint64_t)result->scl_high + 1u) * tpresc_min_ps >= high_min_ps);
  CHECK(sdadel_ps >= sdadel_min_ps);
  CHECK(sdadel_ps + af_max_ps +
        (uint64_t)(input->digital_filter + 4u) * kernel_period_max_ps <=
        hold_max_ps);
  CHECK(sdadel_ps + rise_ps + af_max_ps +
        (uint64_t)(input->digital_filter + 4u) * kernel_period_max_ps <=
        valid_max_ps);
  CHECK(4u * kernel_period_max_ps + af_max_ps +
        (uint64_t)input->digital_filter * kernel_period_max_ps < low_min_ps);
  CHECK(kernel_period_max_ps < high_min_ps);
  CHECK(3u * stm32_i2c_timing_period_max(input->apb_frequency_hz,
                                        input->clock_tolerance_ppm) <
        4u * period_min_ps);
  CHECK(period_min_ps >=
        stm32_i2c_timing_div_ceil(STM32_I2C_TIMING_PS_PER_SECOND,
                                  input->frequency_hz));
  CHECK(result->maximum_scl_hz ==
        STM32_I2C_TIMING_PS_PER_SECOND / period_min_ps);
  return EXIT_SUCCESS;
}

static int test_standard_mode(void)
{
  struct stm32_i2c_timing_input_s input =
  {
    .kernel_frequency_hz = 64000000u,
    .apb_frequency_hz = 50000000u,
    .clock_tolerance_ppm = 10000u,
    .frequency_hz = 100000u,
    .rise_time_ns = 1000u,
    .fall_time_ns = 300u,
    .digital_filter = 0u,
    .analog_filter = false
  };
  struct stm32_i2c_timing_result_s result;

  CHECK(stm32_i2c_calculate_timing(&input, &result) == 0);
  CHECK(check_timing(&input, &result) == EXIT_SUCCESS);
  return EXIT_SUCCESS;
}

static int test_fast_mode_with_filters(void)
{
  struct stm32_i2c_timing_input_s input =
  {
    .kernel_frequency_hz = 64000000u,
    .apb_frequency_hz = 50000000u,
    .clock_tolerance_ppm = 10000u,
    .frequency_hz = 400000u,
    .rise_time_ns = 300u,
    .fall_time_ns = 300u,
    .analog_filter_min_ns = 50u,
    .analog_filter_max_ns = 260u,
    .digital_filter = 1u,
    .analog_filter = true
  };
  struct stm32_i2c_timing_result_s result;

  CHECK(stm32_i2c_calculate_timing(&input, &result) == 0);
  CHECK(check_timing(&input, &result) == EXIT_SUCCESS);
  return EXIT_SUCCESS;
}

static int test_rejections(void)
{
  struct stm32_i2c_timing_input_s input =
  {
    .kernel_frequency_hz = 64000000u,
    .apb_frequency_hz = 50000000u,
    .frequency_hz = 100000u,
    .rise_time_ns = 1000u,
    .fall_time_ns = 300u
  };
  struct stm32_i2c_timing_result_s result;

  CHECK(stm32_i2c_calculate_timing(&input, &result) == 0);
  input.frequency_hz = 0;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -EINVAL);
  input.frequency_hz = 200000u;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -ENOTSUP);
  input.frequency_hz = 400000u;
  input.rise_time_ns = 301u;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -ERANGE);
  input.frequency_hz = 100000u;
  input.rise_time_ns = 0;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -ERANGE);
  input.rise_time_ns = 1000u;
  input.clock_tolerance_ppm = 1000000u;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -EINVAL);
  input.clock_tolerance_ppm = 0;
  input.analog_filter = true;
  input.analog_filter_max_ns = 0;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -EINVAL);
  input.analog_filter = false;
  input.analog_filter_max_ns = 0;
  input.kernel_frequency_hz = 1000000u;
  input.frequency_hz = 400000u;
  CHECK(stm32_i2c_calculate_timing(&input, &result) == -ERANGE);
  return EXIT_SUCCESS;
}

int main(void)
{
  if (test_standard_mode() != EXIT_SUCCESS ||
      test_fast_mode_with_filters() != EXIT_SUCCESS ||
      test_rejections() != EXIT_SUCCESS)
    {
      return EXIT_FAILURE;
    }

  puts("STM32N6 I2C timing tests passed");
  return EXIT_SUCCESS;
}
