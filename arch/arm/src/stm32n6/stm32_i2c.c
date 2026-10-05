/****************************************************************************
 * arch/arm/src/stm32n6/stm32_i2c.c
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

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32_i2c_timing.h"
#include "stm32_i2c_transfer.h"
#include "hardware/stm32n6xxx_pinmap.h"
#include "hardware/stm32n6xxx_rcc.h"

#if defined(CONFIG_STM32_I2C1) || defined(CONFIG_STM32_I2C2) || \
    defined(CONFIG_STM32_I2C3) || defined(CONFIG_STM32_I2C4)

#ifndef STM32_HSI_FREQUENCY
#  error "STM32_HSI_FREQUENCY must be defined by the board"
#endif

#ifndef CONFIG_STM32_I2CTIMEOTICKS
#  define CONFIG_STM32_I2CTIMEOTICKS MSEC2TICK(500)
#endif

#ifndef CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP 1000
#endif

enum stm32_i2c_phase_e
{
  STM32_I2C_PHASE_IDLE = 0,
  STM32_I2C_PHASE_ACTIVE,
  STM32_I2C_PHASE_STOPPING,
  STM32_I2C_PHASE_WAIT_NEXT,
  STM32_I2C_PHASE_WAIT_FINAL,
  STM32_I2C_PHASE_DONE,
  STM32_I2C_PHASE_ERROR
};

enum stm32_i2c_after_stop_e
{
  STM32_I2C_STOP_FINAL = 0,
  STM32_I2C_STOP_NEXT
};

struct stm32_i2c_config_s
{
  uintptr_t base;
  uintptr_t rcc_enable;
  uintptr_t rcc_enable_set;
  uintptr_t rcc_enable_clear;
  uintptr_t rcc_reset;
  uintptr_t rcc_reset_set;
  uintptr_t rcc_reset_clear;
  uint32_t enable_mask;
  uint32_t reset_mask;
  uint32_t ccipr_mask;
  uint32_t ccipr_source;
  uint32_t kernel_clock_hz;
  uint32_t apb_clock_hz;
  uint32_t clock_tolerance_ppm;
  uint32_t rise_time_ns;
  uint32_t fall_time_ns;
  uint32_t analog_filter_min_ns;
  uint32_t analog_filter_max_ns;
  uint32_t scl_pin;
  uint32_t sda_pin;
  uint8_t digital_filter;
  bool analog_filter;
  int event_irq;
  int error_irq;
  uint8_t port;
};

struct stm32_i2c_priv_s
{
  struct i2c_master_s dev;
  const struct stm32_i2c_config_s *config;
  mutex_t lock;
  unsigned int references;
  uint32_t kernel_frequency;
  uint32_t configured_frequency;
  bool initialized;
  bool scl_configured;
  bool sda_configured;
  volatile bool transfer_active;
  volatile enum stm32_i2c_phase_e phase;
  volatile int transfer_result;
  struct i2c_msg_s *messages;
  int message_count;
  int message_index;
  size_t message_offset;
  size_t message_remaining;
  uint16_t block_remaining;
  enum stm32_i2c_after_stop_e after_stop;
  clock_t transfer_start;
  uint32_t transfer_timeout;
#ifndef CONFIG_I2C_POLLED
  sem_t transfer_sem;
  bool sem_initialized;
  bool event_irq_attached;
  bool error_irq_attached;
#endif
};

static int stm32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);
#ifndef CONFIG_I2C_POLLED
static int stm32_i2c_interrupt(int irq, void *context, void *arg);
#endif

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = stm32_i2c_transfer
};

#if defined(CONFIG_STM32_I2C1)
#  if !defined(GPIO_I2C1_SCL) || !defined(GPIO_I2C1_SDA) || \
      !defined(BOARD_I2C1_KERNEL_CLOCK_SOURCE) || \
      !defined(BOARD_I2C1_KERNEL_CLOCK_HZ) || \
      !defined(BOARD_I2C1_APB_CLOCK_HZ) || \
      !defined(BOARD_I2C1_CLOCK_TOLERANCE_PPM) || \
      !defined(BOARD_I2C1_RISE_TIME_NS) || \
      !defined(BOARD_I2C1_FALL_TIME_NS) || \
      !defined(BOARD_I2C1_ANALOG_FILTER_MIN_NS) || \
      !defined(BOARD_I2C1_ANALOG_FILTER_MAX_NS) || \
      !defined(BOARD_I2C1_DIGITAL_FILTER) || \
      !defined(BOARD_I2C1_ANALOG_FILTER)
#    error "I2C1 requires board pin, clock, and timing input definitions"
#  endif
#  if BOARD_I2C1_KERNEL_CLOCK_SOURCE != RCC_CCIPR4_I2C1SEL_HSI_DIV_CK
#    error "STM32N6 I2C1 currently requires hsi_div_ck"
#  endif

static const struct stm32_i2c_config_s g_i2c1_config =
{
  .base = STM32_I2C1_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .rcc_enable_set = STM32_RCC_APB1LENSR,
  .rcc_enable_clear = STM32_RCC_APB1LENCR,
  .rcc_reset = STM32_RCC_APB1LRSTR,
  .rcc_reset_set = STM32_RCC_APB1LRSTSR,
  .rcc_reset_clear = STM32_RCC_APB1LRSTCR,
  .enable_mask = RCC_APB1LENR_I2C1EN,
  .reset_mask = RCC_APB1LRSTR_I2C1RST,
  .ccipr_mask = RCC_CCIPR4_I2C1SEL_MASK,
  .ccipr_source = BOARD_I2C1_KERNEL_CLOCK_SOURCE,
  .kernel_clock_hz = BOARD_I2C1_KERNEL_CLOCK_HZ,
  .apb_clock_hz = BOARD_I2C1_APB_CLOCK_HZ,
  .clock_tolerance_ppm = BOARD_I2C1_CLOCK_TOLERANCE_PPM,
  .rise_time_ns = BOARD_I2C1_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C1_FALL_TIME_NS,
  .analog_filter_min_ns = BOARD_I2C1_ANALOG_FILTER_MIN_NS,
  .analog_filter_max_ns = BOARD_I2C1_ANALOG_FILTER_MAX_NS,
  .scl_pin = GPIO_I2C1_SCL,
  .sda_pin = GPIO_I2C1_SDA,
  .digital_filter = BOARD_I2C1_DIGITAL_FILTER,
  .analog_filter = BOARD_I2C1_ANALOG_FILTER,
  .event_irq = STM32_IRQ_I2C1_EV,
  .error_irq = STM32_IRQ_I2C1_ER,
  .port = 1
};

static struct stm32_i2c_priv_s g_i2c1 =
{
  .dev = { &g_i2c_ops },
  .config = &g_i2c1_config,
  .lock = NXMUTEX_INITIALIZER
};
#endif

#if defined(CONFIG_STM32_I2C2)
#  if !defined(GPIO_I2C2_SCL) || !defined(GPIO_I2C2_SDA) || \
      !defined(BOARD_I2C2_KERNEL_CLOCK_SOURCE) || \
      !defined(BOARD_I2C2_KERNEL_CLOCK_HZ) || \
      !defined(BOARD_I2C2_APB_CLOCK_HZ) || \
      !defined(BOARD_I2C2_CLOCK_TOLERANCE_PPM) || \
      !defined(BOARD_I2C2_RISE_TIME_NS) || \
      !defined(BOARD_I2C2_FALL_TIME_NS) || \
      !defined(BOARD_I2C2_ANALOG_FILTER_MIN_NS) || \
      !defined(BOARD_I2C2_ANALOG_FILTER_MAX_NS) || \
      !defined(BOARD_I2C2_DIGITAL_FILTER) || \
      !defined(BOARD_I2C2_ANALOG_FILTER)
#    error "I2C2 requires board pin, clock, and timing input definitions"
#  endif
#  if BOARD_I2C2_KERNEL_CLOCK_SOURCE != RCC_CCIPR4_I2C2SEL_HSI_DIV_CK
#    error "STM32N6 I2C2 currently requires hsi_div_ck"
#  endif

static const struct stm32_i2c_config_s g_i2c2_config =
{
  .base = STM32_I2C2_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .rcc_enable_set = STM32_RCC_APB1LENSR,
  .rcc_enable_clear = STM32_RCC_APB1LENCR,
  .rcc_reset = STM32_RCC_APB1LRSTR,
  .rcc_reset_set = STM32_RCC_APB1LRSTSR,
  .rcc_reset_clear = STM32_RCC_APB1LRSTCR,
  .enable_mask = RCC_APB1LENR_I2C2EN,
  .reset_mask = RCC_APB1LRSTR_I2C2RST,
  .ccipr_mask = RCC_CCIPR4_I2C2SEL_MASK,
  .ccipr_source = BOARD_I2C2_KERNEL_CLOCK_SOURCE,
  .kernel_clock_hz = BOARD_I2C2_KERNEL_CLOCK_HZ,
  .apb_clock_hz = BOARD_I2C2_APB_CLOCK_HZ,
  .clock_tolerance_ppm = BOARD_I2C2_CLOCK_TOLERANCE_PPM,
  .rise_time_ns = BOARD_I2C2_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C2_FALL_TIME_NS,
  .analog_filter_min_ns = BOARD_I2C2_ANALOG_FILTER_MIN_NS,
  .analog_filter_max_ns = BOARD_I2C2_ANALOG_FILTER_MAX_NS,
  .scl_pin = GPIO_I2C2_SCL,
  .sda_pin = GPIO_I2C2_SDA,
  .digital_filter = BOARD_I2C2_DIGITAL_FILTER,
  .analog_filter = BOARD_I2C2_ANALOG_FILTER,
  .event_irq = STM32_IRQ_I2C2_EV,
  .error_irq = STM32_IRQ_I2C2_ER,
  .port = 2
};

static struct stm32_i2c_priv_s g_i2c2 =
{
  .dev = { &g_i2c_ops },
  .config = &g_i2c2_config,
  .lock = NXMUTEX_INITIALIZER
};
#endif

#if defined(CONFIG_STM32_I2C3)
#  if !defined(GPIO_I2C3_SCL) || !defined(GPIO_I2C3_SDA) || \
      !defined(BOARD_I2C3_KERNEL_CLOCK_SOURCE) || \
      !defined(BOARD_I2C3_KERNEL_CLOCK_HZ) || \
      !defined(BOARD_I2C3_APB_CLOCK_HZ) || \
      !defined(BOARD_I2C3_CLOCK_TOLERANCE_PPM) || \
      !defined(BOARD_I2C3_RISE_TIME_NS) || \
      !defined(BOARD_I2C3_FALL_TIME_NS) || \
      !defined(BOARD_I2C3_ANALOG_FILTER_MIN_NS) || \
      !defined(BOARD_I2C3_ANALOG_FILTER_MAX_NS) || \
      !defined(BOARD_I2C3_DIGITAL_FILTER) || \
      !defined(BOARD_I2C3_ANALOG_FILTER)
#    error "I2C3 requires board pin, clock, and timing input definitions"
#  endif
#  if BOARD_I2C3_KERNEL_CLOCK_SOURCE != RCC_CCIPR4_I2C3SEL_HSI_DIV_CK
#    error "STM32N6 I2C3 currently requires hsi_div_ck"
#  endif

static const struct stm32_i2c_config_s g_i2c3_config =
{
  .base = STM32_I2C3_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .rcc_enable_set = STM32_RCC_APB1LENSR,
  .rcc_enable_clear = STM32_RCC_APB1LENCR,
  .rcc_reset = STM32_RCC_APB1LRSTR,
  .rcc_reset_set = STM32_RCC_APB1LRSTSR,
  .rcc_reset_clear = STM32_RCC_APB1LRSTCR,
  .enable_mask = RCC_APB1LENR_I2C3EN,
  .reset_mask = RCC_APB1LRSTR_I2C3RST,
  .ccipr_mask = RCC_CCIPR4_I2C3SEL_MASK,
  .ccipr_source = BOARD_I2C3_KERNEL_CLOCK_SOURCE,
  .kernel_clock_hz = BOARD_I2C3_KERNEL_CLOCK_HZ,
  .apb_clock_hz = BOARD_I2C3_APB_CLOCK_HZ,
  .clock_tolerance_ppm = BOARD_I2C3_CLOCK_TOLERANCE_PPM,
  .rise_time_ns = BOARD_I2C3_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C3_FALL_TIME_NS,
  .analog_filter_min_ns = BOARD_I2C3_ANALOG_FILTER_MIN_NS,
  .analog_filter_max_ns = BOARD_I2C3_ANALOG_FILTER_MAX_NS,
  .scl_pin = GPIO_I2C3_SCL,
  .sda_pin = GPIO_I2C3_SDA,
  .digital_filter = BOARD_I2C3_DIGITAL_FILTER,
  .analog_filter = BOARD_I2C3_ANALOG_FILTER,
  .event_irq = STM32_IRQ_I2C3_EV,
  .error_irq = STM32_IRQ_I2C3_ER,
  .port = 3
};

static struct stm32_i2c_priv_s g_i2c3 =
{
  .dev = { &g_i2c_ops },
  .config = &g_i2c3_config,
  .lock = NXMUTEX_INITIALIZER
};
#endif

#if defined(CONFIG_STM32_I2C4)
#  if !defined(GPIO_I2C4_SCL) || !defined(GPIO_I2C4_SDA) || \
      !defined(BOARD_I2C4_KERNEL_CLOCK_SOURCE) || \
      !defined(BOARD_I2C4_KERNEL_CLOCK_HZ) || \
      !defined(BOARD_I2C4_APB_CLOCK_HZ) || \
      !defined(BOARD_I2C4_CLOCK_TOLERANCE_PPM) || \
      !defined(BOARD_I2C4_RISE_TIME_NS) || \
      !defined(BOARD_I2C4_FALL_TIME_NS) || \
      !defined(BOARD_I2C4_ANALOG_FILTER_MIN_NS) || \
      !defined(BOARD_I2C4_ANALOG_FILTER_MAX_NS) || \
      !defined(BOARD_I2C4_DIGITAL_FILTER) || \
      !defined(BOARD_I2C4_ANALOG_FILTER)
#    error "I2C4 requires board pin, clock, and timing input definitions"
#  endif
#  if BOARD_I2C4_KERNEL_CLOCK_SOURCE != RCC_CCIPR4_I2C4SEL_HSI_DIV_CK
#    error "STM32N6 I2C4 currently requires hsi_div_ck"
#  endif

static const struct stm32_i2c_config_s g_i2c4_config =
{
  .base = STM32_I2C4_BASE,
  .rcc_enable = STM32_RCC_APB4LENR,
  .rcc_enable_set = STM32_RCC_APB4LENSR,
  .rcc_enable_clear = STM32_RCC_APB4LENCR,
  .rcc_reset = STM32_RCC_APB4LRSTR,
  .rcc_reset_set = STM32_RCC_APB4LRSTSR,
  .rcc_reset_clear = STM32_RCC_APB4LRSTCR,
  .enable_mask = RCC_APB4LENR_I2C4EN,
  .reset_mask = RCC_APB4LRSTR_I2C4RST,
  .ccipr_mask = RCC_CCIPR4_I2C4SEL_MASK,
  .ccipr_source = BOARD_I2C4_KERNEL_CLOCK_SOURCE,
  .kernel_clock_hz = BOARD_I2C4_KERNEL_CLOCK_HZ,
  .apb_clock_hz = BOARD_I2C4_APB_CLOCK_HZ,
  .clock_tolerance_ppm = BOARD_I2C4_CLOCK_TOLERANCE_PPM,
  .rise_time_ns = BOARD_I2C4_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C4_FALL_TIME_NS,
  .analog_filter_min_ns = BOARD_I2C4_ANALOG_FILTER_MIN_NS,
  .analog_filter_max_ns = BOARD_I2C4_ANALOG_FILTER_MAX_NS,
  .scl_pin = GPIO_I2C4_SCL,
  .sda_pin = GPIO_I2C4_SDA,
  .digital_filter = BOARD_I2C4_DIGITAL_FILTER,
  .analog_filter = BOARD_I2C4_ANALOG_FILTER,
  .event_irq = STM32_IRQ_I2C4_EV,
  .error_irq = STM32_IRQ_I2C4_ER,
  .port = 4
};

static struct stm32_i2c_priv_s g_i2c4 =
{
  .dev = { &g_i2c_ops },
  .config = &g_i2c4_config,
  .lock = NXMUTEX_INITIALIZER
};
#endif

static int stm32_i2c_kernel_frequency(
  const struct stm32_i2c_config_s *config, uint32_t *frequency)
{
  uint32_t divider;

  if ((getreg32(STM32_RCC_CCIPR4) & config->ccipr_mask) !=
      config->ccipr_source)
    {
      return -EIO;
    }

  if ((getreg32(STM32_RCC_SR) & RCC_SR_HSIRDY) == 0)
    {
      return -ENODEV;
    }

  divider = (getreg32(STM32_RCC_HSICFGR) & RCC_HSICFGR_HSIDIV_MASK) >>
            RCC_HSICFGR_HSIDIV_SHIFT;
  *frequency = STM32_HSI_FREQUENCY >> divider;
  if (*frequency == 0 || *frequency != config->kernel_clock_hz ||
      *frequency > 100000000u)
    {
      return -ERANGE;
    }

  return OK;
}

static int stm32_i2c_set_timing(struct stm32_i2c_priv_s *priv,
                                uint32_t frequency_hz)
{
  const struct stm32_i2c_config_s *config = priv->config;
  struct stm32_i2c_timing_input_s input =
  {
    .kernel_frequency_hz = priv->kernel_frequency,
    .apb_frequency_hz = config->apb_clock_hz,
    .clock_tolerance_ppm = config->clock_tolerance_ppm,
    .frequency_hz = frequency_hz,
    .rise_time_ns = config->rise_time_ns,
    .fall_time_ns = config->fall_time_ns,
    .analog_filter_min_ns = config->analog_filter_min_ns,
    .analog_filter_max_ns = config->analog_filter_max_ns,
    .digital_filter = config->digital_filter,
    .analog_filter = config->analog_filter
  };
  struct stm32_i2c_timing_result_s result;
  uintptr_t cr1 = config->base + STM32_I2C_CR1_OFFSET;
  uintptr_t isr = config->base + STM32_I2C_ISR_OFFSET;
  uintptr_t timingr = config->base + STM32_I2C_TIMINGR_OFFSET;
  uint32_t filter_mask = I2C_CR1_DNF_MASK | I2C_CR1_ANFOFF;
  uint32_t filter_value =
      ((uint32_t)config->digital_filter << I2C_CR1_DNF_SHIFT) |
      (config->analog_filter ? 0 : I2C_CR1_ANFOFF);
  int ret;

  if (priv->configured_frequency == frequency_hz)
    {
      return OK;
    }

  ret = stm32_i2c_calculate_timing(&input, &result);
  if (ret < 0)
    {
      i2cerr("I2C%u cannot calculate %lu Hz timing: %d\n", config->port,
             (unsigned long)frequency_hz, ret);
      return ret;
    }

  if ((getreg32(isr) & I2C_ISR_BUSY) != 0)
    {
      i2cerr("I2C%u cannot configure timing while the bus is busy\n",
             config->port);
      return -EBUSY;
    }

  modifyreg32(cr1, I2C_CR1_PE, 0);
  if ((getreg32(cr1) & I2C_CR1_PE) != 0)
    {
      i2cerr("I2C%u failed to disable the peripheral for timing setup\n",
             config->port);
      return -EIO;
    }

  modifyreg32(cr1, filter_mask, filter_value);
  if ((getreg32(cr1) & filter_mask) != filter_value)
    {
      i2cerr("I2C%u filter configuration readback failed\n", config->port);
      return -EIO;
    }

  putreg32(result.timingr, timingr);
  if (getreg32(timingr) != result.timingr)
    {
      i2cerr("I2C%u TIMINGR readback failed\n", config->port);
      return -EIO;
    }

  modifyreg32(cr1, 0, I2C_CR1_PE);
  if ((getreg32(cr1) & I2C_CR1_PE) == 0)
    {
      i2cerr("I2C%u failed to enable the peripheral after timing setup\n",
             config->port);
      return -EIO;
    }

  priv->configured_frequency = frequency_hz;
  i2cinfo("I2C%u timing %lu Hz: TIMINGR=%08lx, max=%lu Hz\n",
          config->port, (unsigned long)frequency_hz,
          (unsigned long)result.timingr,
          (unsigned long)result.maximum_scl_hz);
  return OK;
}

#define STM32_I2C_INT_MASK (I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE | \
                            I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE)

#define STM32_I2C_CLEARABLE_FLAGS (I2C_ISR_NACKF | I2C_ISR_STOPF | \
                                   I2C_ISR_BERR | I2C_ISR_ARLO | \
                                   I2C_ISR_OVR | I2C_ISR_PECERR | \
                                   I2C_ISR_TIMEOUT | I2C_ISR_ALERT)

static void stm32_i2c_set_interrupt_sources(struct stm32_i2c_priv_s *priv,
                                            uint32_t sources)
{
#ifdef CONFIG_I2C_POLLED
  sources = 0;
#endif

  modifyreg32(priv->config->base + STM32_I2C_CR1_OFFSET,
              STM32_I2C_INT_MASK, sources & STM32_I2C_INT_MASK);
}

static void stm32_i2c_clear_flags(struct stm32_i2c_priv_s *priv,
                                  uint32_t status)
{
  uint32_t clear = 0;

  if ((status & I2C_ISR_NACKF) != 0)
    {
      clear |= I2C_ICR_NACKCF;
    }

  if ((status & I2C_ISR_STOPF) != 0)
    {
      clear |= I2C_ICR_STOPCF;
    }

  if ((status & I2C_ISR_BERR) != 0)
    {
      clear |= I2C_ICR_BERRCF;
    }

  if ((status & I2C_ISR_ARLO) != 0)
    {
      clear |= I2C_ICR_ARLOCF;
    }

  if ((status & I2C_ISR_OVR) != 0)
    {
      clear |= I2C_ICR_OVRCF;
    }

  if ((status & I2C_ISR_PECERR) != 0)
    {
      clear |= I2C_ICR_PECCF;
    }

  if ((status & I2C_ISR_TIMEOUT) != 0)
    {
      clear |= I2C_ICR_TIMOUTCF;
    }

  if ((status & I2C_ISR_ALERT) != 0)
    {
      clear |= I2C_ICR_ALERTCF;
    }

  if (clear != 0)
    {
      putreg32(clear, priv->config->base + STM32_I2C_ICR_OFFSET);
    }
}

#ifndef CONFIG_I2C_POLLED
static void stm32_i2c_wake_transfer(struct stm32_i2c_priv_s *priv)
{
  nxsem_post(&priv->transfer_sem);
}
#else
#  define stm32_i2c_wake_transfer(p) ((void)(p))
#endif

static void stm32_i2c_fail_transfer(struct stm32_i2c_priv_s *priv, int result)
{
  priv->transfer_result = result;
  priv->phase = STM32_I2C_PHASE_ERROR;
  priv->transfer_active = false;
  stm32_i2c_set_interrupt_sources(priv, 0);
  stm32_i2c_wake_transfer(priv);
}

static void stm32_i2c_load_message(struct stm32_i2c_priv_s *priv,
                                   bool start)
{
  const struct i2c_msg_s *msg = &priv->messages[priv->message_index];
  uint32_t cr2 = ((uint32_t)msg->addr << I2C_CR2_SADD7_SHIFT) |
                 ((uint32_t)stm32_i2c_message_block_size(
                      priv->message_remaining) << I2C_CR2_NBYTES_SHIFT);
  uint8_t block_size = stm32_i2c_message_block_size(
      priv->message_remaining);
  bool reload = stm32_i2c_message_reload(priv->messages,
                                         priv->message_count,
                                         priv->message_index,
                                         priv->message_remaining,
                                         block_size);

  priv->block_remaining = block_size;
  if ((msg->flags & I2C_M_READ) != 0)
    {
      cr2 |= I2C_CR2_RD_WRN;
      stm32_i2c_set_interrupt_sources(priv, I2C_CR1_RXIE |
          I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE);
    }
  else
    {
      stm32_i2c_set_interrupt_sources(priv, I2C_CR1_TXIE |
          I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE);
    }

  if (reload)
    {
      cr2 |= I2C_CR2_RELOAD;
    }

  if (start)
    {
      cr2 |= I2C_CR2_START;
    }

  priv->phase = STM32_I2C_PHASE_ACTIVE;
  putreg32(cr2, priv->config->base + STM32_I2C_CR2_OFFSET);
}

static void stm32_i2c_reload_block(struct stm32_i2c_priv_s *priv)
{
  uint8_t block_size;
  uint32_t cr2;
  bool reload;

  if (priv->block_remaining != 0)
    {
      stm32_i2c_fail_transfer(priv, -EIO);
      return;
    }

  if (priv->message_remaining == 0)
    {
      if (priv->message_index + 1 >= priv->message_count ||
          (priv->messages[priv->message_index + 1].flags &
           I2C_M_NOSTART) == 0)
        {
          stm32_i2c_fail_transfer(priv, -EIO);
          return;
        }

      priv->message_index++;
      priv->message_offset = 0;
      priv->message_remaining =
          (size_t)priv->messages[priv->message_index].length;
    }

  block_size = stm32_i2c_message_block_size(priv->message_remaining);
  reload = stm32_i2c_message_reload(priv->messages, priv->message_count,
                                    priv->message_index,
                                    priv->message_remaining, block_size);
  priv->block_remaining = block_size;
  cr2 = (uint32_t)block_size << I2C_CR2_NBYTES_SHIFT;
  if (reload)
    {
      cr2 |= I2C_CR2_RELOAD;
    }

  modifyreg32(priv->config->base + STM32_I2C_CR2_OFFSET,
              I2C_CR2_NBYTES_MASK | I2C_CR2_RELOAD, cr2);
}

static void stm32_i2c_request_stop(struct stm32_i2c_priv_s *priv,
                                   enum stm32_i2c_after_stop_e after_stop)
{
  priv->after_stop = after_stop;
  priv->phase = STM32_I2C_PHASE_STOPPING;
  stm32_i2c_set_interrupt_sources(priv, I2C_CR1_NACKIE |
      I2C_CR1_STOPIE | I2C_CR1_ERRIE);
  modifyreg32(priv->config->base + STM32_I2C_CR2_OFFSET, 0, I2C_CR2_STOP);
}

static void stm32_i2c_transfer_status(struct stm32_i2c_priv_s *priv)
{
  uint32_t isr = getreg32(priv->config->base + STM32_I2C_ISR_OFFSET);
  uint32_t errors = isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO |
                           I2C_ISR_OVR | I2C_ISR_PECERR |
                           I2C_ISR_TIMEOUT | I2C_ISR_ALERT);
  int result;

  if (!priv->transfer_active ||
      (priv->phase != STM32_I2C_PHASE_ACTIVE &&
       priv->phase != STM32_I2C_PHASE_STOPPING))
    {
      return;
    }

  if (errors != 0)
    {
      if (errors & I2C_ISR_ARLO)
        {
          result = -EAGAIN;
        }
      else if (errors & I2C_ISR_NACKF)
        {
          result = -ENXIO;
        }
      else if (errors & I2C_ISR_TIMEOUT)
        {
          result = -ETIMEDOUT;
        }
      else
        {
          result = -EIO;
        }

      if ((isr & I2C_ISR_RXNE) != 0)
        {
          (void)getreg8(priv->config->base + STM32_I2C_RXDR_OFFSET);
        }

      stm32_i2c_clear_flags(priv, isr);
      stm32_i2c_fail_transfer(priv, result);
      return;
    }

  if ((isr & I2C_ISR_RXNE) != 0)
    {
      const struct i2c_msg_s *msg =
          &priv->messages[priv->message_index];

      if ((msg->flags & I2C_M_READ) == 0 ||
          priv->message_remaining == 0 || priv->block_remaining == 0)
        {
          (void)getreg8(priv->config->base + STM32_I2C_RXDR_OFFSET);
          stm32_i2c_fail_transfer(priv, -EIO);
          return;
        }

      msg->buffer[priv->message_offset++] =
          getreg8(priv->config->base + STM32_I2C_RXDR_OFFSET);
      priv->message_remaining--;
      priv->block_remaining--;
    }

  if ((isr & I2C_ISR_TXIS) != 0)
    {
      const struct i2c_msg_s *msg =
          &priv->messages[priv->message_index];

      if ((msg->flags & I2C_M_READ) != 0 ||
          priv->message_remaining == 0 || priv->block_remaining == 0)
        {
          stm32_i2c_fail_transfer(priv, -EIO);
          return;
        }

      putreg8(msg->buffer[priv->message_offset++],
              priv->config->base + STM32_I2C_TXDR_OFFSET);
      priv->message_remaining--;
      priv->block_remaining--;
    }

  if ((isr & I2C_ISR_TCR) != 0)
    {
      stm32_i2c_reload_block(priv);
      if (!priv->transfer_active)
        {
          return;
        }
    }

  if ((isr & I2C_ISR_TC) != 0)
    {
      const struct i2c_msg_s *msg =
          &priv->messages[priv->message_index];

      if (priv->block_remaining != 0 || priv->message_remaining != 0)
        {
          stm32_i2c_fail_transfer(priv, -EIO);
          return;
        }

      if (priv->message_index + 1 < priv->message_count)
        {
          if ((priv->messages[priv->message_index + 1].flags &
               I2C_M_NOSTART) != 0)
            {
              stm32_i2c_fail_transfer(priv, -EIO);
              return;
            }

          if ((msg->flags & I2C_M_NOSTOP) != 0)
            {
              priv->message_index++;
              priv->message_offset = 0;
              priv->message_remaining =
                  (size_t)priv->messages[priv->message_index].length;
              stm32_i2c_load_message(priv, true);
            }
          else
            {
              stm32_i2c_request_stop(priv, STM32_I2C_STOP_NEXT);
            }
        }
      else
        {
          stm32_i2c_request_stop(priv, STM32_I2C_STOP_FINAL);
        }
    }

  if ((isr & I2C_ISR_STOPF) != 0)
    {
      stm32_i2c_clear_flags(priv, isr & I2C_ISR_STOPF);
      if (priv->phase != STM32_I2C_PHASE_STOPPING)
        {
          stm32_i2c_fail_transfer(priv, -EIO);
          return;
        }

      priv->phase = priv->after_stop == STM32_I2C_STOP_NEXT ?
                    STM32_I2C_PHASE_WAIT_NEXT :
                    STM32_I2C_PHASE_WAIT_FINAL;
      stm32_i2c_set_interrupt_sources(priv, 0);
      stm32_i2c_wake_transfer(priv);
    }
}

#ifndef CONFIG_I2C_POLLED
static int stm32_i2c_interrupt(int irq, void *context, void *arg)
{
  struct stm32_i2c_priv_s *priv = arg;

  (void)irq;
  (void)context;
  if (priv != NULL && priv->transfer_active)
    {
      stm32_i2c_transfer_status(priv);
    }

  return OK;
}

static int stm32_i2c_attach_irqs(struct stm32_i2c_priv_s *priv)
{
  const struct stm32_i2c_config_s *config = priv->config;
  int ret;

  ret = nxsem_init(&priv->transfer_sem, 0, 0);
  if (ret < 0)
    {
      return ret;
    }

  priv->sem_initialized = true;
  ret = irq_attach(config->event_irq, stm32_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout;
    }

  priv->event_irq_attached = true;
  ret = irq_attach(config->error_irq, stm32_i2c_interrupt, priv);
  if (ret < 0)
    {
      goto errout;
    }

  priv->error_irq_attached = true;
  up_disable_irq(config->event_irq);
  up_disable_irq(config->error_irq);
  return OK;

errout:
  if (priv->event_irq_attached)
    {
      up_disable_irq(config->event_irq);
      irq_detach(config->event_irq);
      priv->event_irq_attached = false;
    }

  if (priv->sem_initialized)
    {
      nxsem_destroy(&priv->transfer_sem);
      priv->sem_initialized = false;
    }

  return ret;
}

static int stm32_i2c_detach_irqs(struct stm32_i2c_priv_s *priv)
{
  const struct stm32_i2c_config_s *config = priv->config;
  int ret = OK;
  int tmp;

  if (priv->event_irq_attached)
    {
      up_disable_irq(config->event_irq);
      tmp = irq_detach(config->event_irq);
      if (tmp < 0 && ret == OK)
        {
          ret = tmp;
        }
      else if (tmp >= 0)
        {
          priv->event_irq_attached = false;
        }
    }

  if (priv->error_irq_attached)
    {
      up_disable_irq(config->error_irq);
      tmp = irq_detach(config->error_irq);
      if (tmp < 0 && ret == OK)
        {
          ret = tmp;
        }
      else if (tmp >= 0)
        {
          priv->error_irq_attached = false;
        }
    }

  if (priv->sem_initialized)
    {
      tmp = nxsem_destroy(&priv->transfer_sem);
      if (tmp < 0 && ret == OK)
        {
          ret = tmp;
        }
      else if (tmp >= 0)
        {
          priv->sem_initialized = false;
        }
    }

  return ret;
}
#endif

static int stm32_i2c_clock_disable(
  const struct stm32_i2c_config_s *config)
{
  putreg32(config->enable_mask, config->rcc_enable_clear);
  return (getreg32(config->rcc_enable) & config->enable_mask) == 0 ?
         OK : -EACCES;
}

static int stm32_i2c_hardware_initialize(struct stm32_i2c_priv_s *priv)
{
  const struct stm32_i2c_config_s *config = priv->config;
  irqstate_t flags;
  int cleanup;
  int ret;

  ret = stm32_configgpio(config->scl_pin);
  if (ret < 0)
    {
      return ret;
    }

  priv->scl_configured = true;
  ret = stm32_configgpio(config->sda_pin);
  if (ret < 0)
    {
      goto errout;
    }

  priv->sda_configured = true;

  flags = enter_critical_section();
  modifyreg32(STM32_RCC_CCIPR4, config->ccipr_mask,
              config->ccipr_source);
  ret = (getreg32(STM32_RCC_CCIPR4) & config->ccipr_mask) ==
        config->ccipr_source ? OK : -EACCES;
  leave_critical_section(flags);
  if (ret < 0)
    {
      goto errout;
    }

  putreg32(config->enable_mask, config->rcc_enable_set);
  if ((getreg32(config->rcc_enable) & config->enable_mask) == 0)
    {
      ret = -EACCES;
      goto errout;
    }

  putreg32(config->reset_mask, config->rcc_reset_set);
  if ((getreg32(config->rcc_reset) & config->reset_mask) == 0)
    {
      ret = -EACCES;
      goto errout_clock;
    }

  putreg32(config->reset_mask, config->rcc_reset_clear);
  if ((getreg32(config->rcc_reset) & config->reset_mask) != 0)
    {
      ret = -EACCES;
      goto errout_clock;
    }

  ret = stm32_i2c_kernel_frequency(config, &priv->kernel_frequency);
  if (ret < 0)
    {
      goto errout_clock;
    }

  ret = stm32_i2c_set_timing(priv, 100000u);
  if (ret < 0)
    {
      goto errout_clock;
    }

#ifndef CONFIG_I2C_POLLED
  ret = stm32_i2c_attach_irqs(priv);
  if (ret < 0)
    {
      goto errout_clock;
    }
#endif

  return OK;

errout_clock:
  cleanup = stm32_i2c_clock_disable(config);
  if (cleanup < 0)
    {
      i2cerr("I2C%d failed to disable clock during rollback: %d\n",
             config->port, cleanup);
    }
errout:
  if (priv->sda_configured)
    {
      cleanup = stm32_unconfiggpio(config->sda_pin);
      if (cleanup < 0)
        {
          i2cerr("I2C%d failed to release SDA during rollback: %d\n",
                 config->port, cleanup);
        }
      else
        {
          priv->sda_configured = false;
        }
    }

  if (priv->scl_configured)
    {
      cleanup = stm32_unconfiggpio(config->scl_pin);
      if (cleanup < 0)
        {
          i2cerr("I2C%d failed to release SCL during rollback: %d\n",
                 config->port, cleanup);
        }
      else
        {
          priv->scl_configured = false;
        }
    }

  return ret;
}

static int stm32_i2c_hardware_uninitialize(struct stm32_i2c_priv_s *priv)
{
  const struct stm32_i2c_config_s *config = priv->config;
  int ret = OK;
  int tmp;

  priv->transfer_active = false;
  putreg32(0, config->base + STM32_I2C_CR1_OFFSET);

#ifndef CONFIG_I2C_POLLED
  ret = stm32_i2c_detach_irqs(priv);
#endif

  tmp = stm32_i2c_clock_disable(config);
  if (tmp < 0 && ret == OK)
    {
      ret = tmp;
    }

  if (priv->sda_configured)
    {
      tmp = stm32_unconfiggpio(config->sda_pin);
      if (tmp < 0 && ret == OK)
        {
          ret = tmp;
        }

      if (tmp >= 0)
        {
          priv->sda_configured = false;
        }
    }

  if (priv->scl_configured)
    {
      tmp = stm32_unconfiggpio(config->scl_pin);
      if (tmp < 0 && ret == OK)
        {
          ret = tmp;
        }

      if (tmp >= 0)
        {
          priv->scl_configured = false;
        }
    }

  if (ret == OK)
    {
      priv->kernel_frequency = 0;
      priv->configured_frequency = 0;
    }

  return ret;
}

static struct stm32_i2c_priv_s *stm32_i2c_get_instance(int port)
{
  switch (port)
    {
#ifdef CONFIG_STM32_I2C1
      case 1:
        return &g_i2c1;
#endif
#ifdef CONFIG_STM32_I2C2
      case 2:
        return &g_i2c2;
#endif
#ifdef CONFIG_STM32_I2C3
      case 3:
        return &g_i2c3;
#endif
#ifdef CONFIG_STM32_I2C4
      case 4:
        return &g_i2c4;
#endif
      default:
        return NULL;
    }
}

static int stm32_i2c_timeout_ticks(
    const struct stm32_i2c_message_vector_s *vector,
    uint32_t *timeout_ticks)
{
#ifdef CONFIG_STM32_I2C_DYNTIMEO
  uint64_t wire_bytes;
  uint64_t phases;
  uint64_t timeout_us;
  uint64_t overhead_us;
  int64_t configured_per_byte_us =
      CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE;
  int64_t configured_startstop_ms =
      CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP;
  uint64_t per_byte_us;
  uint64_t startstop_ms;

  if (configured_per_byte_us <= 0 || configured_startstop_ms < 0 ||
      USEC_PER_TICK <= 0 ||
      vector->address_phases > SIZE_MAX - vector->payload_bytes)
    {
      return -EINVAL;
    }

  per_byte_us = (uint64_t)configured_per_byte_us;
  startstop_ms = (uint64_t)configured_startstop_ms;
  wire_bytes = vector->payload_bytes + vector->address_phases;
  phases = vector->address_phases + vector->stop_phases;
  if (wire_bytes != 0 && per_byte_us > UINT64_MAX / wire_bytes)
    {
      return -EOVERFLOW;
    }

  timeout_us = wire_bytes * per_byte_us;
  if (startstop_ms > UINT64_MAX / 1000u)
    {
      return -EOVERFLOW;
    }

  overhead_us = startstop_ms * 1000u;
  if (phases != 0 && overhead_us > UINT64_MAX / phases)
    {
      return -EOVERFLOW;
    }

  overhead_us *= phases;
  if (timeout_us > UINT64_MAX - overhead_us)
    {
      return -EOVERFLOW;
    }

  timeout_us += overhead_us;
  timeout_us = timeout_us / USEC_PER_TICK +
               (timeout_us % USEC_PER_TICK != 0);
  if (timeout_us == 0)
    {
      timeout_us = 1;
    }

  if (timeout_us > UINT32_MAX)
    {
      return -EOVERFLOW;
    }

  *timeout_ticks = (uint32_t)timeout_us;
#else
  (void)vector;

  if (CONFIG_STM32_I2CTIMEOTICKS <= 0)
    {
      return -EINVAL;
    }

  *timeout_ticks = CONFIG_STM32_I2CTIMEOTICKS;
#endif

  return OK;
}

static bool stm32_i2c_timed_out(const struct stm32_i2c_priv_s *priv)
{
  return (uint64_t)(clock_systime_ticks() - priv->transfer_start) >=
         priv->transfer_timeout;
}

#ifndef CONFIG_I2C_POLLED
static uint32_t stm32_i2c_remaining_ticks(
    const struct stm32_i2c_priv_s *priv)
{
  uint64_t elapsed =
      (uint64_t)(clock_systime_ticks() - priv->transfer_start);

  return elapsed >= priv->transfer_timeout ? 0 :
         priv->transfer_timeout - (uint32_t)elapsed;
}
#endif

static int stm32_i2c_wait_idle(struct stm32_i2c_priv_s *priv)
{
  while ((getreg32(priv->config->base + STM32_I2C_ISR_OFFSET) &
          I2C_ISR_BUSY) != 0)
    {
      if (stm32_i2c_timed_out(priv))
        {
          return -ETIMEDOUT;
        }

      up_udelay(10);
    }

  return OK;
}

static void stm32_i2c_drain_semaphore(struct stm32_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED
  int ret;

  do
    {
      ret = nxsem_trywait(&priv->transfer_sem);
    }
  while (ret == OK);

  if (ret != -EAGAIN)
    {
      i2cerr("I2C%u semaphore drain failed: %d\n",
             priv->config->port, ret);
    }
#else
  (void)priv;
#endif
}

static void stm32_i2c_begin_transfer(struct stm32_i2c_priv_s *priv,
                                     struct i2c_msg_s *msgs, int count,
                                     uint32_t timeout_ticks)
{
  uint32_t status;

  irqstate_t flags = enter_critical_section();

  stm32_i2c_set_interrupt_sources(priv, 0);
  status = getreg32(priv->config->base + STM32_I2C_ISR_OFFSET);
  if ((status & I2C_ISR_RXNE) != 0)
    {
      (void)getreg8(priv->config->base + STM32_I2C_RXDR_OFFSET);
    }

  stm32_i2c_clear_flags(priv, status);
  stm32_i2c_drain_semaphore(priv);

  priv->messages = msgs;
  priv->message_count = count;
  priv->message_index = 0;
  priv->message_offset = 0;
  priv->message_remaining = (size_t)msgs[0].length;
  priv->block_remaining = 0;
  priv->transfer_result = 0;
  priv->transfer_timeout = timeout_ticks;
  priv->transfer_start = clock_systime_ticks();
  priv->transfer_active = true;

#ifndef CONFIG_I2C_POLLED
  up_enable_irq(priv->config->event_irq);
  up_enable_irq(priv->config->error_irq);
#endif

  stm32_i2c_load_message(priv, true);
  leave_critical_section(flags);
}

static void stm32_i2c_start_next_message(struct stm32_i2c_priv_s *priv)
{
  irqstate_t flags = enter_critical_section();

  if (priv->phase == STM32_I2C_PHASE_WAIT_NEXT &&
      priv->transfer_active)
    {
      priv->message_index++;
      priv->message_offset = 0;
      priv->message_remaining =
          (size_t)priv->messages[priv->message_index].length;

#ifndef CONFIG_I2C_POLLED
      up_enable_irq(priv->config->event_irq);
      up_enable_irq(priv->config->error_irq);
#endif

      stm32_i2c_load_message(priv, true);
    }

  leave_critical_section(flags);
}

static int stm32_i2c_wait_next_message(struct stm32_i2c_priv_s *priv)
{
  uint32_t delay_us = priv->messages[0].frequency == I2C_SPEED_STANDARD ?
                      5u : 2u;

  up_udelay(delay_us);
  if (stm32_i2c_timed_out(priv))
    {
      return -ETIMEDOUT;
    }

  return stm32_i2c_wait_idle(priv);
}

static int stm32_i2c_wait_final_stop(struct stm32_i2c_priv_s *priv)
{
  int ret = stm32_i2c_wait_idle(priv);
  irqstate_t flags;

  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  if (priv->phase == STM32_I2C_PHASE_WAIT_FINAL)
    {
      priv->phase = STM32_I2C_PHASE_DONE;
      priv->transfer_active = false;
    }
  leave_critical_section(flags);
  return OK;
}

static void stm32_i2c_quiesce_transfer(struct stm32_i2c_priv_s *priv)
{
  irqstate_t flags = enter_critical_section();

  priv->transfer_active = false;
  stm32_i2c_set_interrupt_sources(priv, 0);
#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->event_irq);
  up_disable_irq(priv->config->error_irq);
#endif
  leave_critical_section(flags);
}

static int stm32_i2c_cleanup_stop(struct stm32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t timeout = MSEC2TICK(20);
  uint32_t status;

  if (timeout <= 0)
    {
      timeout = 1;
    }

  status = getreg32(priv->config->base + STM32_I2C_ISR_OFFSET);
  if ((status & I2C_ISR_BUSY) == 0)
    {
      return OK;
    }

  if (priv->transfer_result != -EAGAIN &&
      (getreg32(priv->config->base + STM32_I2C_CR2_OFFSET) &
       I2C_CR2_STOP) == 0)
    {
      modifyreg32(priv->config->base + STM32_I2C_CR2_OFFSET,
                  0, I2C_CR2_STOP);
    }

  start = clock_systime_ticks();
  do
    {
      status = getreg32(priv->config->base + STM32_I2C_ISR_OFFSET);
      if ((status & I2C_ISR_STOPF) != 0)
        {
          stm32_i2c_clear_flags(priv, I2C_ISR_STOPF);
          return OK;
        }

      if ((status & I2C_ISR_BUSY) == 0)
        {
          return OK;
        }

      up_udelay(10);
    }
  while (clock_systime_ticks() - start < timeout);

  return -ETIMEDOUT;
}

static void stm32_i2c_clear_transfer_status(struct stm32_i2c_priv_s *priv)
{
  uint32_t status = getreg32(priv->config->base + STM32_I2C_ISR_OFFSET);

  if ((status & I2C_ISR_RXNE) != 0)
    {
      (void)getreg8(priv->config->base + STM32_I2C_RXDR_OFFSET);
    }

  stm32_i2c_clear_flags(priv, status & STM32_I2C_CLEARABLE_FLAGS);
}

static int stm32_i2c_run_transfer(struct stm32_i2c_priv_s *priv)
{
  for (;;)
    {
      enum stm32_i2c_phase_e phase = priv->phase;
      int ret;

      if (stm32_i2c_timed_out(priv))
        {
          return -ETIMEDOUT;
        }

      if (phase == STM32_I2C_PHASE_DONE)
        {
          return OK;
        }

      if (phase == STM32_I2C_PHASE_ERROR)
        {
          return priv->transfer_result;
        }

      if (phase == STM32_I2C_PHASE_WAIT_NEXT)
        {
          ret = stm32_i2c_wait_next_message(priv);
          if (ret < 0)
            {
              return ret;
            }

          stm32_i2c_start_next_message(priv);
          continue;
        }

      if (phase == STM32_I2C_PHASE_WAIT_FINAL)
        {
          ret = stm32_i2c_wait_final_stop(priv);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      if (phase != STM32_I2C_PHASE_ACTIVE &&
          phase != STM32_I2C_PHASE_STOPPING)
        {
          return -EIO;
        }

#ifdef CONFIG_I2C_POLLED
      stm32_i2c_transfer_status(priv);
      up_udelay(1);
#else
      uint32_t remaining;

      remaining = stm32_i2c_remaining_ticks(priv);
      if (remaining == 0)
        {
          return -ETIMEDOUT;
        }

      ret = nxsem_tickwait_uninterruptible(&priv->transfer_sem, remaining);
      if (ret < 0 && ret != -ETIMEDOUT)
        {
          return ret;
        }
#endif
    }
}

static int stm32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  struct stm32_i2c_message_vector_s vector;
  struct stm32_i2c_priv_s *priv = NULL;
  uint32_t kernel_frequency;
  uint32_t timeout_ticks;
  int ret;
  int i;
  bool started = false;

  if (dev == NULL || up_interrupt_context() ||
      getprimask() != 0 || getbasepri() != 0)
    {
      return -EWOULDBLOCK;
    }

  ret = stm32_i2c_validate_messages(msgs, count, &vector);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_i2c_timeout_ticks(&vector, &timeout_ticks);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 1; i <= 4; i++)
    {
      struct stm32_i2c_priv_s *candidate = stm32_i2c_get_instance(i);

      if (candidate != NULL && dev == &candidate->dev)
        {
          priv = candidate;
          break;
        }
    }

  if (priv == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->initialized || priv->references == 0)
    {
      ret = -ENODEV;
      goto out;
    }

  priv->transfer_start = clock_systime_ticks();
  priv->transfer_timeout = timeout_ticks;
  ret = stm32_i2c_wait_idle(priv);
  if (ret < 0)
    {
      goto out;
    }

  up_udelay(vector.frequency == I2C_SPEED_STANDARD ? 5u : 2u);
  if (stm32_i2c_timed_out(priv))
    {
      ret = -ETIMEDOUT;
      goto out;
    }

  ret = stm32_i2c_kernel_frequency(priv->config, &kernel_frequency);
  if (ret < 0 || kernel_frequency != priv->kernel_frequency)
    {
      ret = ret < 0 ? ret : -EIO;
      goto out;
    }

  ret = stm32_i2c_set_timing(priv, vector.frequency);
  if (ret < 0)
    {
      goto out;
    }

  stm32_i2c_begin_transfer(priv, msgs, count, timeout_ticks);
  started = true;
  ret = stm32_i2c_run_transfer(priv);

  stm32_i2c_quiesce_transfer(priv);
  if (ret < 0 && started)
    {
      int cleanup = stm32_i2c_cleanup_stop(priv);

      if (cleanup < 0)
        {
          i2cerr("I2C%u stop cleanup failed after %d: %d\n",
                 priv->config->port, ret, cleanup);
        }
    }

  stm32_i2c_clear_transfer_status(priv);
  if (ret < 0)
    {
      i2cerr("I2C%u transfer failed: %d, message %d, ISR=%08lx\n",
             priv->config->port, ret, priv->message_index,
             (unsigned long)getreg32(priv->config->base +
                                     STM32_I2C_ISR_OFFSET));
    }

  priv->messages = NULL;
  priv->message_count = 0;
  priv->message_remaining = 0;
  priv->block_remaining = 0;
  priv->phase = STM32_I2C_PHASE_IDLE;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

struct i2c_master_s *stm32_i2cbus_initialize(int port)
{
  struct stm32_i2c_priv_s *priv = stm32_i2c_get_instance(port);
  int ret;

  if (priv == NULL)
    {
      return NULL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      i2cerr("I2C%d lock failed: %d\n", port, ret);
      return NULL;
    }

  if (priv->references == 0)
    {
      if (priv->initialized)
        {
          nxmutex_unlock(&priv->lock);
          i2cerr("I2C%d is unavailable after failed cleanup\n", port);
          return NULL;
        }

      ret = stm32_i2c_hardware_initialize(priv);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          i2cerr("I2C%d initialization failed: %d\n", port, ret);
          return NULL;
        }

      priv->initialized = true;
    }

  priv->references++;
  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

int stm32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct stm32_i2c_priv_s *priv = NULL;
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_STM32_I2C1
  if (dev == &g_i2c1.dev)
    {
      priv = &g_i2c1;
    }
#endif
#ifdef CONFIG_STM32_I2C2
  if (dev == &g_i2c2.dev)
    {
      priv = &g_i2c2;
    }
#endif
#ifdef CONFIG_STM32_I2C3
  if (dev == &g_i2c3.dev)
    {
      priv = &g_i2c3;
    }
#endif
#ifdef CONFIG_STM32_I2C4
  if (dev == &g_i2c4.dev)
    {
      priv = &g_i2c4;
    }
#endif

  if (priv == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->references == 0)
    {
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  priv->references--;
  if (priv->references == 0)
    {
      ret = stm32_i2c_hardware_uninitialize(priv);
      if (ret == OK)
        {
          priv->initialized = false;
        }
    }
  else
    {
      ret = OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

#endif /* CONFIG_STM32_I2C1 || CONFIG_STM32_I2C2 || \
        * CONFIG_STM32_I2C3 || CONFIG_STM32_I2C4 */
