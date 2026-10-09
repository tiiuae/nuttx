/****************************************************************************
 * arch/arm/src/stm32n6/stm32_i2c_timing.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_I2C_TIMING_H
#define __ARCH_ARM_SRC_STM32N6_STM32_I2C_TIMING_H

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/stm32n6xxx_i2c.h"

#define STM32_I2C_TIMING_PS_PER_SECOND 1000000000000ull
#define STM32_I2C_TIMING_PPM           1000000ull

struct stm32_i2c_timing_input_s
{
  uint32_t kernel_frequency_hz;
  uint32_t apb_frequency_hz;
  uint32_t clock_tolerance_ppm;
  uint32_t frequency_hz;
  uint32_t rise_time_ns;
  uint32_t fall_time_ns;
  uint32_t analog_filter_min_ns;
  uint32_t analog_filter_max_ns;
  uint8_t digital_filter;
  bool analog_filter;
};

struct stm32_i2c_timing_result_s
{
  uint32_t timingr;
  uint32_t maximum_scl_hz;
  uint8_t prescaler;
  uint8_t scl_delay;
  uint8_t sda_delay;
  uint8_t scl_high;
  uint8_t scl_low;
};

static inline uint64_t stm32_i2c_timing_div_ceil(uint64_t numerator,
                                                  uint64_t denominator)
{
  return numerator / denominator + (numerator % denominator != 0);
}

static inline uint64_t stm32_i2c_timing_period_max(uint32_t frequency_hz,
                                                   uint32_t tolerance_ppm)
{
  uint64_t frequency = (uint64_t)frequency_hz *
                       (STM32_I2C_TIMING_PPM - tolerance_ppm) /
                       STM32_I2C_TIMING_PPM;

  return frequency == 0 ? 0 :
         STM32_I2C_TIMING_PS_PER_SECOND / frequency +
         (STM32_I2C_TIMING_PS_PER_SECOND % frequency != 0);
}

static inline uint64_t stm32_i2c_timing_period_min(uint32_t frequency_hz,
                                                  uint32_t tolerance_ppm)
{
  uint64_t frequency = stm32_i2c_timing_div_ceil(
      (uint64_t)frequency_hz * (STM32_I2C_TIMING_PPM + tolerance_ppm),
      STM32_I2C_TIMING_PPM);

  return frequency == 0 ? 0 :
         STM32_I2C_TIMING_PS_PER_SECOND / frequency;
}

static inline int stm32_i2c_calculate_timing(
    const struct stm32_i2c_timing_input_s *input,
    struct stm32_i2c_timing_result_s *result)
{
  uint64_t kernel_period_max_ps;
  uint64_t kernel_period_min_ps;
  uint64_t apb_period_max_ps;
  uint64_t rise_ps;
  uint64_t fall_ps;
  uint64_t af_min_ps;
  uint64_t af_max_ps;
  uint64_t filter_max_ps;
  uint64_t setup_min_ps;
  uint64_t hold_max_ps;
  uint64_t valid_max_ps;
  uint64_t low_min_ps;
  uint64_t high_min_ps;
  uint64_t period_target_ps;
  uint64_t best_period_ps = UINT64_MAX;
  uint32_t best_presc = 0;
  uint32_t best_scldel = 0;
  uint32_t best_sdadel = 0;
  uint32_t best_sclh = 0;
  uint32_t best_scll = 0;
  uint32_t rise_limit_ns;
  uint32_t fall_limit_ns;
  uint32_t hold_limit_ns;
  uint32_t valid_limit_ns;
  uint32_t setup_limit_ns;
  uint32_t low_limit_ns;
  uint32_t high_limit_ns;
  uint32_t presc;

  if (input == NULL || result == NULL ||
      input->kernel_frequency_hz < 1000000u ||
      input->kernel_frequency_hz > 100000000u ||
      input->apb_frequency_hz == 0 ||
      input->clock_tolerance_ppm >= STM32_I2C_TIMING_PPM ||
      input->digital_filter > 15u)
    {
      return -EINVAL;
    }

  if (input->frequency_hz == 0)
    {
      return -EINVAL;
    }
  else if (input->frequency_hz == 100000u)
    {
      rise_limit_ns = 1000u;
      fall_limit_ns = 300u;
      setup_limit_ns = 250u;
      hold_limit_ns = 3450u;
      valid_limit_ns = 3450u;
      low_limit_ns = 4700u;
      high_limit_ns = 4000u;
    }
  else if (input->frequency_hz == 400000u)
    {
      rise_limit_ns = 300u;
      fall_limit_ns = 300u;
      setup_limit_ns = 100u;
      hold_limit_ns = 900u;
      valid_limit_ns = 900u;
      low_limit_ns = 1300u;
      high_limit_ns = 600u;
    }
  else
    {
      return -ENOTSUP;
    }

  if (input->rise_time_ns == 0 || input->fall_time_ns == 0 ||
      input->rise_time_ns > rise_limit_ns ||
      input->fall_time_ns > fall_limit_ns)
    {
      return -ERANGE;
    }

  if (input->analog_filter)
    {
      if (input->analog_filter_max_ns == 0 ||
          input->analog_filter_min_ns > input->analog_filter_max_ns)
        {
          return -EINVAL;
        }

      af_min_ps = (uint64_t)input->analog_filter_min_ns * 1000u;
      af_max_ps = (uint64_t)input->analog_filter_max_ns * 1000u;
    }
  else
    {
      if (input->analog_filter_min_ns != 0 ||
          input->analog_filter_max_ns != 0)
        {
          return -EINVAL;
        }

      af_min_ps = 0;
      af_max_ps = 0;
    }

  /* Check timing at both oscillator tolerance endpoints. */

  kernel_period_max_ps = stm32_i2c_timing_period_max(
      input->kernel_frequency_hz, input->clock_tolerance_ppm);
  kernel_period_min_ps = stm32_i2c_timing_period_min(
      input->kernel_frequency_hz, input->clock_tolerance_ppm);
  apb_period_max_ps = stm32_i2c_timing_period_max(
      input->apb_frequency_hz, input->clock_tolerance_ppm);
  if (kernel_period_max_ps == 0 || kernel_period_min_ps == 0 ||
      apb_period_max_ps == 0)
    {
      return -ERANGE;
    }

  if ((uint64_t)input->kernel_frequency_hz *
      (STM32_I2C_TIMING_PPM + input->clock_tolerance_ppm) >
      100000000ull * STM32_I2C_TIMING_PPM)
    {
      return -ERANGE;
    }

  rise_ps = (uint64_t)input->rise_time_ns * 1000u;
  fall_ps = (uint64_t)input->fall_time_ns * 1000u;
  filter_max_ps = af_max_ps +
                  (uint64_t)input->digital_filter * kernel_period_max_ps;
  low_min_ps = (uint64_t)low_limit_ns * 1000u;
  high_min_ps = (uint64_t)high_limit_ns * 1000u;
  setup_min_ps = (uint64_t)setup_limit_ns * 1000u;
  hold_max_ps = (uint64_t)hold_limit_ns * 1000u;
  valid_max_ps = (uint64_t)valid_limit_ns * 1000u;
  period_target_ps = stm32_i2c_timing_div_ceil(
      STM32_I2C_TIMING_PS_PER_SECOND, input->frequency_hz);

  for (presc = 0; presc < 16u; presc++)
    {
      uint64_t tpresc_min_ps = kernel_period_min_ps * (presc + 1u);
      uint64_t tpresc_max_ps = kernel_period_max_ps * (presc + 1u);
      uint64_t scldel_cycles;
      int64_t sdadel_min_ps;
      uint64_t sdadel_cycles;
      int64_t sdadel_upper_hold_ps;
      int64_t sdadel_upper_valid_ps;
      uint64_t sync_min_ps;
      uint64_t required_total_cycles;
      uint64_t scll_min_cycles;
      uint64_t sclh_min_cycles;
      uint64_t scll;

      if (4u * kernel_period_max_ps + filter_max_ps >= low_min_ps ||
          kernel_period_max_ps >= high_min_ps)
        {
          return -ERANGE;
        }

      scldel_cycles = stm32_i2c_timing_div_ceil(
          rise_ps + setup_min_ps, tpresc_min_ps);
      if (scldel_cycles == 0 || scldel_cycles > 16u)
        {
          continue;
        }

      sdadel_min_ps = (int64_t)fall_ps -
          (int64_t)(af_min_ps +
                    (uint64_t)(input->digital_filter + 3u) *
                    kernel_period_min_ps);
      sdadel_cycles = sdadel_min_ps > 0 ?
          stm32_i2c_timing_div_ceil((uint64_t)sdadel_min_ps,
                                    tpresc_min_ps) : 0;
      if (sdadel_cycles > 15u)
        {
          continue;
        }

      sdadel_upper_hold_ps = (int64_t)hold_max_ps -
          (int64_t)(af_max_ps +
                    (uint64_t)(input->digital_filter + 4u) *
                    kernel_period_max_ps);
      sdadel_upper_valid_ps = (int64_t)valid_max_ps -
          (int64_t)(rise_ps + af_max_ps +
                    (uint64_t)(input->digital_filter + 4u) *
                    kernel_period_max_ps);
      if (sdadel_upper_hold_ps < 0 || sdadel_upper_valid_ps < 0 ||
          sdadel_cycles * tpresc_max_ps >
          (uint64_t)sdadel_upper_hold_ps ||
          sdadel_cycles * tpresc_max_ps >
          (uint64_t)sdadel_upper_valid_ps)
        {
          continue;
        }

      scll_min_cycles = stm32_i2c_timing_div_ceil(low_min_ps,
                                                   tpresc_min_ps);
      sclh_min_cycles = stm32_i2c_timing_div_ceil(high_min_ps,
                                                   tpresc_min_ps);
      if (scll_min_cycles > 256u || sclh_min_cycles > 256u)
        {
          continue;
        }

      /* Use the RM0486 two-cycle minimum per synchronization phase. */

      sync_min_ps = 2u * af_min_ps +
                    (uint64_t)(2u * input->digital_filter + 4u) *
                    kernel_period_min_ps;
      required_total_cycles = period_target_ps > sync_min_ps ?
          stm32_i2c_timing_div_ceil(period_target_ps - sync_min_ps,
                                    tpresc_min_ps) : 0;

      for (scll = scll_min_cycles; scll <= 256u; scll++)
        {
          uint64_t sclh = sclh_min_cycles;
          uint64_t period_ps;

          if (required_total_cycles > scll + sclh)
            {
              sclh = required_total_cycles - scll;
            }

          if (sclh > 256u)
            {
              continue;
            }

          period_ps = (scll + sclh) * tpresc_min_ps + sync_min_ps;
          if (period_ps < period_target_ps ||
              3u * apb_period_max_ps >= 4u * period_ps)
            {
              continue;
            }

          if (period_ps < best_period_ps)
            {
              best_period_ps = period_ps;
              best_presc = presc;
              best_scldel = (uint32_t)scldel_cycles - 1u;
              best_sdadel = (uint32_t)sdadel_cycles;
              best_sclh = (uint32_t)sclh - 1u;
              best_scll = (uint32_t)scll - 1u;
            }
        }
    }

  if (best_period_ps == UINT64_MAX)
    {
      return -ERANGE;
    }

  result->timingr =
      (best_presc << I2C_TIMINGR_PRESC_SHIFT) |
      (best_scldel << I2C_TIMINGR_SCLDEL_SHIFT) |
      (best_sdadel << I2C_TIMINGR_SDADEL_SHIFT) |
      (best_sclh << I2C_TIMINGR_SCLH_SHIFT) |
      (best_scll << I2C_TIMINGR_SCLL_SHIFT);
  result->maximum_scl_hz = (uint32_t)(STM32_I2C_TIMING_PS_PER_SECOND /
                                      best_period_ps);
  result->prescaler = (uint8_t)best_presc;
  result->scl_delay = (uint8_t)best_scldel;
  result->sda_delay = (uint8_t)best_sdadel;
  result->scl_high = (uint8_t)best_sclh;
  result->scl_low = (uint8_t)best_scll;
  return 0;
}

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_I2C_TIMING_H */
