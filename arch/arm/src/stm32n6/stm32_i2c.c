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
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "hardware/stm32n6xxx_pinmap.h"
#include "hardware/stm32n6xxx_rcc.h"

#if defined(CONFIG_STM32_I2C1) || defined(CONFIG_STM32_I2C2) || \
    defined(CONFIG_STM32_I2C3) || defined(CONFIG_STM32_I2C4)

#ifndef STM32_HSI_FREQUENCY
#  error "STM32_HSI_FREQUENCY must be defined by the board"
#endif

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
  uint32_t rise_time_ns;
  uint32_t fall_time_ns;
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
  bool initialized;
  bool scl_configured;
  bool sda_configured;
};

static int stm32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = stm32_i2c_transfer
};

#if defined(CONFIG_STM32_I2C1)
#  if !defined(GPIO_I2C1_SCL) || !defined(GPIO_I2C1_SDA) || \
      !defined(BOARD_I2C1_KERNEL_CLOCK_SOURCE) || \
      !defined(BOARD_I2C1_KERNEL_CLOCK_HZ) || \
      !defined(BOARD_I2C1_RISE_TIME_NS) || \
      !defined(BOARD_I2C1_FALL_TIME_NS) || \
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
  .rise_time_ns = BOARD_I2C1_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C1_FALL_TIME_NS,
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
      !defined(BOARD_I2C2_RISE_TIME_NS) || \
      !defined(BOARD_I2C2_FALL_TIME_NS) || \
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
  .rise_time_ns = BOARD_I2C2_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C2_FALL_TIME_NS,
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
      !defined(BOARD_I2C3_RISE_TIME_NS) || \
      !defined(BOARD_I2C3_FALL_TIME_NS) || \
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
  .rise_time_ns = BOARD_I2C3_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C3_FALL_TIME_NS,
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
      !defined(BOARD_I2C4_RISE_TIME_NS) || \
      !defined(BOARD_I2C4_FALL_TIME_NS) || \
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
  .rise_time_ns = BOARD_I2C4_RISE_TIME_NS,
  .fall_time_ns = BOARD_I2C4_FALL_TIME_NS,
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

  ret = stm32_i2c_clock_disable(config);

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

static int stm32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  (void)dev;
  (void)msgs;
  (void)count;

  i2cerr("I2C transfer engine is not implemented\n");
  return -ENOSYS;
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
