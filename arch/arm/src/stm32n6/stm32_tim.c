/****************************************************************************
 * arch/arm/src/stm32n6/stm32_tim.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes
 * include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_STM32_TIMn is defined then the CONFIG_STM32_TIMn_PWM may
 *   also be defined to indicate that the timer is intended to be used for
 *   pulsed output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32_TIMn is
 *   defined then CONFIG_STM32_TIMn_ADC may also be defined to indicate
 *   that timer "n" is intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_STM32_TIMn is defined
 *   then CONFIG_STM32_TIMn_DAC may also be defined to indicate that
 *   timer "n" is intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_STM32_TIMn is defined then
 *   CONFIG_STM32_TIMn_QE may also be defined to indicate that timer "n"
 *   is intended to be used for that purpose.
 *
 * - To capture input signals.  CONFIG_STM32_TIMn_CAP reserves timer "n"
 *   for the capture driver.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_STM32_TIM1_PWM) || defined(CONFIG_STM32_TIM1_ADC) || \
    defined(CONFIG_STM32_TIM1_DAC) || defined(CONFIG_STM32_TIM1_QE) || \
    defined(CONFIG_STM32_TIM1_CAP)
#  undef CONFIG_STM32_TIM1
#endif

#if defined(CONFIG_STM32_TIM2_PWM) || defined(CONFIG_STM32_TIM2_ADC) || \
    defined(CONFIG_STM32_TIM2_DAC) || defined(CONFIG_STM32_TIM2_QE) || \
    defined(CONFIG_STM32_TIM2_CAP)
#  undef CONFIG_STM32_TIM2
#endif

#if defined(CONFIG_STM32_TIM3_PWM) || defined(CONFIG_STM32_TIM3_ADC) || \
    defined(CONFIG_STM32_TIM3_DAC) || defined(CONFIG_STM32_TIM3_QE) || \
    defined(CONFIG_STM32_TIM3_CAP)
#  undef CONFIG_STM32_TIM3
#endif

#if defined(CONFIG_STM32_TIM4_PWM) || defined(CONFIG_STM32_TIM4_ADC) || \
    defined(CONFIG_STM32_TIM4_DAC) || defined(CONFIG_STM32_TIM4_QE) || \
    defined(CONFIG_STM32_TIM4_CAP)
#  undef CONFIG_STM32_TIM4
#endif

#if defined(CONFIG_STM32_TIM5_PWM) || defined(CONFIG_STM32_TIM5_ADC) || \
    defined(CONFIG_STM32_TIM5_DAC) || defined(CONFIG_STM32_TIM5_QE) || \
    defined(CONFIG_STM32_TIM5_CAP)
#  undef CONFIG_STM32_TIM5
#endif

#if defined(CONFIG_STM32_TIM6_PWM) || defined(CONFIG_STM32_TIM6_ADC) || \
    defined(CONFIG_STM32_TIM6_DAC) || defined(CONFIG_STM32_TIM6_QE)
#  undef CONFIG_STM32_TIM6
#endif

#if defined(CONFIG_STM32_TIM7_PWM) || defined(CONFIG_STM32_TIM7_ADC) || \
    defined(CONFIG_STM32_TIM7_DAC) || defined(CONFIG_STM32_TIM7_QE)
#  undef CONFIG_STM32_TIM7
#endif

#if defined(CONFIG_STM32_TIM8_PWM) || defined(CONFIG_STM32_TIM8_ADC) || \
    defined(CONFIG_STM32_TIM8_DAC) || defined(CONFIG_STM32_TIM8_QE) || \
    defined(CONFIG_STM32_TIM8_CAP)
#  undef CONFIG_STM32_TIM8
#endif

#if defined(CONFIG_STM32_TIM9_PWM) || defined(CONFIG_STM32_TIM9_ADC) || \
    defined(CONFIG_STM32_TIM9_DAC) || defined(CONFIG_STM32_TIM9_QE) || \
    defined(CONFIG_STM32_TIM9_CAP)
#  undef CONFIG_STM32_TIM9
#endif

#if defined(CONFIG_STM32_TIM10_PWM) || defined(CONFIG_STM32_TIM10_ADC) || \
    defined(CONFIG_STM32_TIM10_DAC) || defined(CONFIG_STM32_TIM10_QE) || \
    defined(CONFIG_STM32_TIM10_CAP)
#  undef CONFIG_STM32_TIM10
#endif

#if defined(CONFIG_STM32_TIM11_PWM) || defined(CONFIG_STM32_TIM11_ADC) || \
    defined(CONFIG_STM32_TIM11_DAC) || defined(CONFIG_STM32_TIM11_QE) || \
    defined(CONFIG_STM32_TIM11_CAP)
#  undef CONFIG_STM32_TIM11
#endif

#if defined(CONFIG_STM32_TIM12_PWM) || defined(CONFIG_STM32_TIM12_ADC) || \
    defined(CONFIG_STM32_TIM12_DAC) || defined(CONFIG_STM32_TIM12_QE) || \
    defined(CONFIG_STM32_TIM12_CAP)
#  undef CONFIG_STM32_TIM12
#endif

#if defined(CONFIG_STM32_TIM13_PWM) || defined(CONFIG_STM32_TIM13_ADC) || \
    defined(CONFIG_STM32_TIM13_DAC) || defined(CONFIG_STM32_TIM13_QE) || \
    defined(CONFIG_STM32_TIM13_CAP)
#  undef CONFIG_STM32_TIM13
#endif

#if defined(CONFIG_STM32_TIM14_PWM) || defined(CONFIG_STM32_TIM14_ADC) || \
    defined(CONFIG_STM32_TIM14_DAC) || defined(CONFIG_STM32_TIM14_QE) || \
    defined(CONFIG_STM32_TIM14_CAP)
#  undef CONFIG_STM32_TIM14
#endif

#if defined(CONFIG_STM32_TIM15_PWM) || defined(CONFIG_STM32_TIM15_ADC) || \
    defined(CONFIG_STM32_TIM15_DAC) || defined(CONFIG_STM32_TIM15_QE) || \
    defined(CONFIG_STM32_TIM15_CAP)
#  undef CONFIG_STM32_TIM15
#endif

#if defined(CONFIG_STM32_TIM16_PWM) || defined(CONFIG_STM32_TIM16_ADC) || \
    defined(CONFIG_STM32_TIM16_DAC) || defined(CONFIG_STM32_TIM16_QE) || \
    defined(CONFIG_STM32_TIM16_CAP)
#  undef CONFIG_STM32_TIM16
#endif

#if defined(CONFIG_STM32_TIM17_PWM) || defined(CONFIG_STM32_TIM17_ADC) || \
    defined(CONFIG_STM32_TIM17_DAC) || defined(CONFIG_STM32_TIM17_QE) || \
    defined(CONFIG_STM32_TIM17_CAP)
#  undef CONFIG_STM32_TIM17
#endif

#if defined(CONFIG_STM32_TIM18_PWM) || defined(CONFIG_STM32_TIM18_ADC) || \
    defined(CONFIG_STM32_TIM18_DAC) || defined(CONFIG_STM32_TIM18_QE)
#  undef CONFIG_STM32_TIM18
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32_TIM1)  || defined(CONFIG_STM32_TIM2)  || \
    defined(CONFIG_STM32_TIM3)  || defined(CONFIG_STM32_TIM4)  || \
    defined(CONFIG_STM32_TIM5)  || defined(CONFIG_STM32_TIM6)  || \
    defined(CONFIG_STM32_TIM7)  || defined(CONFIG_STM32_TIM8)  || \
    defined(CONFIG_STM32_TIM9)  || defined(CONFIG_STM32_TIM10) || \
    defined(CONFIG_STM32_TIM11) || \
    defined(CONFIG_STM32_TIM12) || defined(CONFIG_STM32_TIM13) || \
    defined(CONFIG_STM32_TIM14) || defined(CONFIG_STM32_TIM15) || \
    defined(CONFIG_STM32_TIM16) || defined(CONFIG_STM32_TIM17) || \
    defined(CONFIG_STM32_TIM18)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Immutable per-timer configuration */

struct stm32_tim_config_s
{
  uintptr_t base;
  uintptr_t rcc_enable;
  uint32_t enable_mask;
  uint32_t clkin;
  uint32_t gpio[6];
  int irq;
  uint8_t width;
  uint8_t channels;
  uint8_t flags;
  uint8_t gpio_mask;              /* Board-defined output pins */
};

/* TIM Device Structure */

struct stm32_tim_priv_s
{
  const struct stm32_tim_ops_s *ops;
  const struct stm32_tim_config_s *config;
  stm32_tim_mode_t mode;
};

#define STM32_TIM_FLAG_BASIC          (1 << 0)
#define STM32_TIM_FLAG_BIDIRECTIONAL  (1 << 1)
#define STM32_TIM_FLAG_MOE            (1 << 2)
#define STM32_TIM_FLAG_ADVANCED       (1 << 3)

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Timer methods */

static void     stm32_tim_enable(struct stm32_tim_dev_s *dev);
static void     stm32_tim_disable(struct stm32_tim_dev_s *dev);
static int      stm32_tim_setmode(struct stm32_tim_dev_s *dev,
                                  stm32_tim_mode_t mode);
static int      stm32_tim_setclock(struct stm32_tim_dev_s *dev,
                                   uint32_t freq);
static void     stm32_tim_setperiod(struct stm32_tim_dev_s *dev,
                                    uint32_t period);
static uint32_t stm32_tim_getcounter(struct stm32_tim_dev_s *dev);
static void     stm32_tim_setcounter(struct stm32_tim_dev_s *dev,
                                     uint32_t count);
static int      stm32_tim_getwidth(struct stm32_tim_dev_s *dev);
static int      stm32_tim_setchannel(struct stm32_tim_dev_s *dev,
                                     uint8_t channel,
                                     stm32_tim_channel_t mode);
static int      stm32_tim_setcompare(struct stm32_tim_dev_s *dev,
                                     uint8_t channel, uint32_t compare);
static int      stm32_tim_getcapture(struct stm32_tim_dev_s *dev,
                                     uint8_t channel);
static int      stm32_tim_setisr(struct stm32_tim_dev_s *dev,
                                 xcpt_t handler, void *arg, int source);
static void     stm32_tim_enableint(struct stm32_tim_dev_s *dev,
                                    int source);
static void     stm32_tim_disableint(struct stm32_tim_dev_s *dev,
                                     int source);
static void     stm32_tim_ackint(struct stm32_tim_dev_s *dev,
                                 int source);
static int      stm32_tim_checkint(struct stm32_tim_dev_s *dev,
                                   int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32_tim_ops_s stm32_tim_ops =
{
  .enable         = &stm32_tim_enable,
  .disable        = &stm32_tim_disable,
  .setmode        = &stm32_tim_setmode,
  .setclock       = &stm32_tim_setclock,
  .setperiod      = &stm32_tim_setperiod,
  .getcounter     = &stm32_tim_getcounter,
  .setcounter     = &stm32_tim_setcounter,
  .getwidth       = &stm32_tim_getwidth,
  .setchannel     = &stm32_tim_setchannel,
  .setcompare     = &stm32_tim_setcompare,
  .getcapture     = &stm32_tim_getcapture,
  .setisr         = &stm32_tim_setisr,
  .enableint      = &stm32_tim_enableint,
  .disableint     = &stm32_tim_disableint,
  .ackint         = &stm32_tim_ackint,
  .checkint       = &stm32_tim_checkint,
};

#ifdef CONFIG_STM32_TIM1
static const struct stm32_tim_config_s stm32_tim1_config =
{
  .base       = STM32_TIM1_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM1EN,
  .clkin      = STM32_TIM1_CLKIN,
  .irq        = STM32_IRQ_TIM1_UP,
  .width      = 16,
  .channels   = 6,
  .flags      = STM32_TIM_FLAG_BIDIRECTIONAL | STM32_TIM_FLAG_MOE |
                STM32_TIM_FLAG_ADVANCED,
  .gpio       =
  {
#  ifdef GPIO_TIM1_CH1OUT
    [0] = GPIO_TIM1_CH1OUT,
#  endif
#  ifdef GPIO_TIM1_CH2OUT
    [1] = GPIO_TIM1_CH2OUT,
#  endif
#  ifdef GPIO_TIM1_CH3OUT
    [2] = GPIO_TIM1_CH3OUT,
#  endif
#  ifdef GPIO_TIM1_CH4OUT
    [3] = GPIO_TIM1_CH4OUT,
#  endif
#  ifdef GPIO_TIM1_CH5OUT
    [4] = GPIO_TIM1_CH5OUT,
#  endif
#  ifdef GPIO_TIM1_CH6OUT
    [5] = GPIO_TIM1_CH6OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM1_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM1_CH2OUT
                | (1 << 1)
#  endif
#  ifdef GPIO_TIM1_CH3OUT
                | (1 << 2)
#  endif
#  ifdef GPIO_TIM1_CH4OUT
                | (1 << 3)
#  endif
#  ifdef GPIO_TIM1_CH5OUT
                | (1 << 4)
#  endif
#  ifdef GPIO_TIM1_CH6OUT
                | (1 << 5)
#  endif
};

static struct stm32_tim_priv_s stm32_tim1_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim1_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif
#ifdef CONFIG_STM32_TIM2
static const struct stm32_tim_config_s stm32_tim2_config =
{
  .base       = STM32_TIM2_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM2EN,
  .clkin      = STM32_TIM2_CLKIN,
  .irq        = STM32_IRQ_TIM2,
  .width      = 32,
  .channels   = 4,
  .flags      = STM32_TIM_FLAG_BIDIRECTIONAL,
  .gpio       =
  {
#  ifdef GPIO_TIM2_CH1OUT
    [0] = GPIO_TIM2_CH1OUT,
#  endif
#  ifdef GPIO_TIM2_CH2OUT
    [1] = GPIO_TIM2_CH2OUT,
#  endif
#  ifdef GPIO_TIM2_CH3OUT
    [2] = GPIO_TIM2_CH3OUT,
#  endif
#  ifdef GPIO_TIM2_CH4OUT
    [3] = GPIO_TIM2_CH4OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM2_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM2_CH2OUT
                | (1 << 1)
#  endif
#  ifdef GPIO_TIM2_CH3OUT
                | (1 << 2)
#  endif
#  ifdef GPIO_TIM2_CH4OUT
                | (1 << 3)
#  endif
};

static struct stm32_tim_priv_s stm32_tim2_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim2_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM3
static const struct stm32_tim_config_s stm32_tim3_config =
{
  .base       = STM32_TIM3_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM3EN,
  .clkin      = STM32_TIM3_CLKIN,
  .irq        = STM32_IRQ_TIM3,
  .width      = 16,
  .channels   = 4,
  .flags      = STM32_TIM_FLAG_BIDIRECTIONAL,
  .gpio       =
  {
#  ifdef GPIO_TIM3_CH1OUT
    [0] = GPIO_TIM3_CH1OUT,
#  endif
#  ifdef GPIO_TIM3_CH2OUT
    [1] = GPIO_TIM3_CH2OUT,
#  endif
#  ifdef GPIO_TIM3_CH3OUT
    [2] = GPIO_TIM3_CH3OUT,
#  endif
#  ifdef GPIO_TIM3_CH4OUT
    [3] = GPIO_TIM3_CH4OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM3_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM3_CH2OUT
                | (1 << 1)
#  endif
#  ifdef GPIO_TIM3_CH3OUT
                | (1 << 2)
#  endif
#  ifdef GPIO_TIM3_CH4OUT
                | (1 << 3)
#  endif
};

static struct stm32_tim_priv_s stm32_tim3_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim3_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM4
static const struct stm32_tim_config_s stm32_tim4_config =
{
  .base       = STM32_TIM4_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM4EN,
  .clkin      = STM32_TIM4_CLKIN,
  .irq        = STM32_IRQ_TIM4,
  .width      = 32,
  .channels   = 4,
  .flags      = STM32_TIM_FLAG_BIDIRECTIONAL,
  .gpio       =
  {
#  ifdef GPIO_TIM4_CH1OUT
    [0] = GPIO_TIM4_CH1OUT,
#  endif
#  ifdef GPIO_TIM4_CH2OUT
    [1] = GPIO_TIM4_CH2OUT,
#  endif
#  ifdef GPIO_TIM4_CH3OUT
    [2] = GPIO_TIM4_CH3OUT,
#  endif
#  ifdef GPIO_TIM4_CH4OUT
    [3] = GPIO_TIM4_CH4OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM4_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM4_CH2OUT
                | (1 << 1)
#  endif
#  ifdef GPIO_TIM4_CH3OUT
                | (1 << 2)
#  endif
#  ifdef GPIO_TIM4_CH4OUT
                | (1 << 3)
#  endif
};

static struct stm32_tim_priv_s stm32_tim4_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim4_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM5
static const struct stm32_tim_config_s stm32_tim5_config =
{
  .base       = STM32_TIM5_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM5EN,
  .clkin      = STM32_TIM5_CLKIN,
  .irq        = STM32_IRQ_TIM5,
  .width      = 32,
  .channels   = 4,
  .flags      = STM32_TIM_FLAG_BIDIRECTIONAL,
  .gpio       =
  {
#  ifdef GPIO_TIM5_CH1OUT
    [0] = GPIO_TIM5_CH1OUT,
#  endif
#  ifdef GPIO_TIM5_CH2OUT
    [1] = GPIO_TIM5_CH2OUT,
#  endif
#  ifdef GPIO_TIM5_CH3OUT
    [2] = GPIO_TIM5_CH3OUT,
#  endif
#  ifdef GPIO_TIM5_CH4OUT
    [3] = GPIO_TIM5_CH4OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM5_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM5_CH2OUT
                | (1 << 1)
#  endif
#  ifdef GPIO_TIM5_CH3OUT
                | (1 << 2)
#  endif
#  ifdef GPIO_TIM5_CH4OUT
                | (1 << 3)
#  endif
};

static struct stm32_tim_priv_s stm32_tim5_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim5_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM6
static const struct stm32_tim_config_s stm32_tim6_config =
{
  .base       = STM32_TIM6_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM6EN,
  .clkin      = STM32_TIM6_CLKIN,
  .irq        = STM32_IRQ_TIM6,
  .width      = 16,
  .channels   = 0,
  .flags      = STM32_TIM_FLAG_BASIC,
};

static struct stm32_tim_priv_s stm32_tim6_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim6_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM7
static const struct stm32_tim_config_s stm32_tim7_config =
{
  .base       = STM32_TIM7_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM7EN,
  .clkin      = STM32_TIM7_CLKIN,
  .irq        = STM32_IRQ_TIM7,
  .width      = 16,
  .channels   = 0,
  .flags      = STM32_TIM_FLAG_BASIC,
};

static struct stm32_tim_priv_s stm32_tim7_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim7_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM8
static const struct stm32_tim_config_s stm32_tim8_config =
{
  .base       = STM32_TIM8_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM8EN,
  .clkin      = STM32_TIM8_CLKIN,
  .irq        = STM32_IRQ_TIM8_UP,
  .width      = 16,
  .channels   = 6,
  .flags      = STM32_TIM_FLAG_BIDIRECTIONAL | STM32_TIM_FLAG_MOE |
                STM32_TIM_FLAG_ADVANCED,
  .gpio       =
  {
#  ifdef GPIO_TIM8_CH1OUT
    [0] = GPIO_TIM8_CH1OUT,
#  endif
#  ifdef GPIO_TIM8_CH2OUT
    [1] = GPIO_TIM8_CH2OUT,
#  endif
#  ifdef GPIO_TIM8_CH3OUT
    [2] = GPIO_TIM8_CH3OUT,
#  endif
#  ifdef GPIO_TIM8_CH4OUT
    [3] = GPIO_TIM8_CH4OUT,
#  endif
#  ifdef GPIO_TIM8_CH5OUT
    [4] = GPIO_TIM8_CH5OUT,
#  endif
#  ifdef GPIO_TIM8_CH6OUT
    [5] = GPIO_TIM8_CH6OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM8_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM8_CH2OUT
                | (1 << 1)
#  endif
#  ifdef GPIO_TIM8_CH3OUT
                | (1 << 2)
#  endif
#  ifdef GPIO_TIM8_CH4OUT
                | (1 << 3)
#  endif
#  ifdef GPIO_TIM8_CH5OUT
                | (1 << 4)
#  endif
#  ifdef GPIO_TIM8_CH6OUT
                | (1 << 5)
#  endif
};

static struct stm32_tim_priv_s stm32_tim8_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim8_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM9
static const struct stm32_tim_config_s stm32_tim9_config =
{
  .base       = STM32_TIM9_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM9EN,
  .clkin      = STM32_TIM9_CLKIN,
  .irq        = STM32_IRQ_TIM9,
  .width      = 16,
  .channels   = 2,
  .flags      = 0,
  .gpio       =
  {
#  ifdef GPIO_TIM9_CH1OUT
    [0] = GPIO_TIM9_CH1OUT,
#  endif
#  ifdef GPIO_TIM9_CH2OUT
    [1] = GPIO_TIM9_CH2OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM9_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM9_CH2OUT
                | (1 << 1)
#  endif
};

static struct stm32_tim_priv_s stm32_tim9_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim9_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM10
static const struct stm32_tim_config_s stm32_tim10_config =
{
  .base       = STM32_TIM10_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM10EN,
  .clkin      = STM32_TIM10_CLKIN,
  .irq        = STM32_IRQ_TIM10,
  .width      = 16,
  .channels   = 1,
  .flags      = 0,
  .gpio       =
  {
#  ifdef GPIO_TIM10_CH1OUT
    [0] = GPIO_TIM10_CH1OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM10_CH1OUT
                | (1 << 0)
#  endif
};

static struct stm32_tim_priv_s stm32_tim10_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim10_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM11
static const struct stm32_tim_config_s stm32_tim11_config =
{
  .base       = STM32_TIM11_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM11EN,
  .clkin      = STM32_TIM11_CLKIN,
  .irq        = STM32_IRQ_TIM11,
  .width      = 16,
  .channels   = 1,
  .flags      = 0,
  .gpio       =
  {
#  ifdef GPIO_TIM11_CH1OUT
    [0] = GPIO_TIM11_CH1OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM11_CH1OUT
                | (1 << 0)
#  endif
};

static struct stm32_tim_priv_s stm32_tim11_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim11_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM12
static const struct stm32_tim_config_s stm32_tim12_config =
{
  .base       = STM32_TIM12_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM12EN,
  .clkin      = STM32_TIM12_CLKIN,
  .irq        = STM32_IRQ_TIM12,
  .width      = 16,
  .channels   = 2,
  .flags      = 0,
  .gpio       =
  {
#  ifdef GPIO_TIM12_CH1OUT
    [0] = GPIO_TIM12_CH1OUT,
#  endif
#  ifdef GPIO_TIM12_CH2OUT
    [1] = GPIO_TIM12_CH2OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM12_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM12_CH2OUT
                | (1 << 1)
#  endif
};

static struct stm32_tim_priv_s stm32_tim12_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim12_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM13
static const struct stm32_tim_config_s stm32_tim13_config =
{
  .base       = STM32_TIM13_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM13EN,
  .clkin      = STM32_TIM13_CLKIN,
  .irq        = STM32_IRQ_TIM13,
  .width      = 16,
  .channels   = 1,
  .flags      = 0,
  .gpio       =
  {
#  ifdef GPIO_TIM13_CH1OUT
    [0] = GPIO_TIM13_CH1OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM13_CH1OUT
                | (1 << 0)
#  endif
};

static struct stm32_tim_priv_s stm32_tim13_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim13_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM14
static const struct stm32_tim_config_s stm32_tim14_config =
{
  .base       = STM32_TIM14_BASE,
  .rcc_enable = STM32_RCC_APB1LENR,
  .enable_mask = RCC_APB1LENR_TIM14EN,
  .clkin      = STM32_TIM14_CLKIN,
  .irq        = STM32_IRQ_TIM14,
  .width      = 16,
  .channels   = 1,
  .flags      = 0,
  .gpio       =
  {
#  ifdef GPIO_TIM14_CH1OUT
    [0] = GPIO_TIM14_CH1OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM14_CH1OUT
                | (1 << 0)
#  endif
};

static struct stm32_tim_priv_s stm32_tim14_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim14_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM15
static const struct stm32_tim_config_s stm32_tim15_config =
{
  .base       = STM32_TIM15_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM15EN,
  .clkin      = STM32_TIM15_CLKIN,
  .irq        = STM32_IRQ_TIM15,
  .width      = 16,
  .channels   = 2,
  .flags      = STM32_TIM_FLAG_MOE,
  .gpio       =
  {
#  ifdef GPIO_TIM15_CH1OUT
    [0] = GPIO_TIM15_CH1OUT,
#  endif
#  ifdef GPIO_TIM15_CH2OUT
    [1] = GPIO_TIM15_CH2OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM15_CH1OUT
                | (1 << 0)
#  endif
#  ifdef GPIO_TIM15_CH2OUT
                | (1 << 1)
#  endif
};

static struct stm32_tim_priv_s stm32_tim15_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim15_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM16
static const struct stm32_tim_config_s stm32_tim16_config =
{
  .base       = STM32_TIM16_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM16EN,
  .clkin      = STM32_TIM16_CLKIN,
  .irq        = STM32_IRQ_TIM16,
  .width      = 16,
  .channels   = 1,
  .flags      = STM32_TIM_FLAG_MOE,
  .gpio       =
  {
#  ifdef GPIO_TIM16_CH1OUT
    [0] = GPIO_TIM16_CH1OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM16_CH1OUT
                | (1 << 0)
#  endif
};

static struct stm32_tim_priv_s stm32_tim16_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim16_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM17
static const struct stm32_tim_config_s stm32_tim17_config =
{
  .base       = STM32_TIM17_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM17EN,
  .clkin      = STM32_TIM17_CLKIN,
  .irq        = STM32_IRQ_TIM17,
  .width      = 16,
  .channels   = 1,
  .flags      = STM32_TIM_FLAG_MOE,
  .gpio       =
  {
#  ifdef GPIO_TIM17_CH1OUT
    [0] = GPIO_TIM17_CH1OUT,
#  endif
  },
  .gpio_mask  = 0
#  ifdef GPIO_TIM17_CH1OUT
                | (1 << 0)
#  endif
};

static struct stm32_tim_priv_s stm32_tim17_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim17_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

#ifdef CONFIG_STM32_TIM18
static const struct stm32_tim_config_s stm32_tim18_config =
{
  .base       = STM32_TIM18_BASE,
  .rcc_enable = STM32_RCC_APB2ENR,
  .enable_mask = RCC_APB2ENR_TIM18EN,
  .clkin      = STM32_TIM18_CLKIN,
  .irq        = STM32_IRQ_TIM18,
  .width      = 16,
  .channels   = 0,
  .flags      = STM32_TIM_FLAG_BASIC,
};

static struct stm32_tim_priv_s stm32_tim18_priv =
{
  .ops        = &stm32_tim_ops,
  .config     = &stm32_tim18_config,
  .mode       = STM32_TIM_MODE_UNUSED,
};
#endif

/* Disabled or reserved timers have NULL entries. */

static struct stm32_tim_priv_s * const stm32_tim_devices[19] =
{
#ifdef CONFIG_STM32_TIM1
  [1] = &stm32_tim1_priv,
#endif
#ifdef CONFIG_STM32_TIM2
  [2] = &stm32_tim2_priv,
#endif
#ifdef CONFIG_STM32_TIM3
  [3] = &stm32_tim3_priv,
#endif
#ifdef CONFIG_STM32_TIM4
  [4] = &stm32_tim4_priv,
#endif
#ifdef CONFIG_STM32_TIM5
  [5] = &stm32_tim5_priv,
#endif
#ifdef CONFIG_STM32_TIM6
  [6] = &stm32_tim6_priv,
#endif
#ifdef CONFIG_STM32_TIM7
  [7] = &stm32_tim7_priv,
#endif
#ifdef CONFIG_STM32_TIM8
  [8] = &stm32_tim8_priv,
#endif
#ifdef CONFIG_STM32_TIM9
  [9] = &stm32_tim9_priv,
#endif
#ifdef CONFIG_STM32_TIM10
  [10] = &stm32_tim10_priv,
#endif
#ifdef CONFIG_STM32_TIM11
  [11] = &stm32_tim11_priv,
#endif
#ifdef CONFIG_STM32_TIM12
  [12] = &stm32_tim12_priv,
#endif
#ifdef CONFIG_STM32_TIM13
  [13] = &stm32_tim13_priv,
#endif
#ifdef CONFIG_STM32_TIM14
  [14] = &stm32_tim14_priv,
#endif
#ifdef CONFIG_STM32_TIM15
  [15] = &stm32_tim15_priv,
#endif
#ifdef CONFIG_STM32_TIM16
  [16] = &stm32_tim16_priv,
#endif
#ifdef CONFIG_STM32_TIM17
  [17] = &stm32_tim17_priv,
#endif
#ifdef CONFIG_STM32_TIM18
  [18] = &stm32_tim18_priv,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t stm32_getreg16(struct stm32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg16(((struct stm32_tim_priv_s *)dev)->config->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(struct stm32_tim_dev_s *dev,
                                  uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct stm32_tim_priv_s *)dev)->config->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(struct stm32_tim_dev_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(((struct stm32_tim_priv_s *)dev)->config->base + offset,
              clearbits, setbits);
}

/* Get a 32-bit register value by offset.  This applies to registers which
 * are 32 bits wide, including CNT, ARR, and CCR1-4 on TIM2, TIM4, and TIM5.
 */

static inline uint32_t stm32_getreg32(struct stm32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg32(((struct stm32_tim_priv_s *)dev)->config->base + offset);
}

/* Put a 32-bit register value by offset.  This applies to registers which
 * are 32 bits wide, including CNT, ARR, and CCR1-4 on TIM2, TIM4, and TIM5.
 */

static inline void stm32_putreg32(struct stm32_tim_dev_s *dev,
                                  uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct stm32_tim_priv_s *)dev)->config->base + offset);
}

static void stm32_tim_reload_counter(struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_EGR_OFFSET);

  val |= GTIM_EGR_UG;
  stm32_putreg16(dev, STM32_GTIM_EGR_OFFSET, val);
}

static void stm32_tim_enable(struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_CR1_OFFSET);

  val |= GTIM_CR1_CEN;
  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);
}

static void stm32_tim_disable(struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_CR1_OFFSET);

  val &= ~GTIM_CR1_CEN;
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: stm32_tim_getwidth
 ****************************************************************************/

static int stm32_tim_getwidth(struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return ((struct stm32_tim_priv_s *)dev)->config->width;
}

/****************************************************************************
 * Name: stm32_tim_getcounter
 ****************************************************************************/

static uint32_t stm32_tim_getcounter(struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return stm32_tim_getwidth(dev) > 16 ?
    stm32_getreg32(dev, STM32_BTIM_CNT_OFFSET) :
    (uint32_t)stm32_getreg16(dev, STM32_BTIM_CNT_OFFSET);
}

/****************************************************************************
 * Name: stm32_tim_setcounter
 ****************************************************************************/

static void stm32_tim_setcounter(struct stm32_tim_dev_s *dev,
                                 uint32_t count)
{
  DEBUGASSERT(dev != NULL);

  if (stm32_tim_getwidth(dev) > 16)
    {
      stm32_putreg32(dev, STM32_BTIM_CNT_OFFSET, count);
    }
  else
    {
      stm32_putreg16(dev, STM32_BTIM_CNT_OFFSET, (uint16_t)count);
    }
}

/* Reset timer into system default state, but do not affect output/input
 * pins
 */

static void stm32_tim_reset(struct stm32_tim_dev_s *dev)
{
  ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_DISABLED;
  stm32_tim_disable(dev);
}

static void stm32_tim_gpioconfig(uint32_t cfg, stm32_tim_channel_t mode)
{
  /* TODO: Add support for input capture and bipolar dual outputs for TIM8 */

  if (mode & STM32_TIM_CH_MODE_MASK)
    {
      stm32_configgpio(cfg);
    }
  else
    {
      stm32_unconfiggpio(cfg);
    }
}

/****************************************************************************
 * Basic Functions
 ****************************************************************************/

static int stm32_tim_setclock(struct stm32_tim_dev_s *dev, uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      stm32_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  freqin = ((struct stm32_tim_priv_s *)dev)->config->clkin;

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   */

  prescaler = freqin / freq;

  /* We need to decrement value for '1', but only, if that will not to
   * cause underflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow as well. */

  if (prescaler > 0xffff)
    {
      prescaler = 0xffff;
    }

  stm32_putreg16(dev, STM32_GTIM_PSC_OFFSET, prescaler);
  stm32_tim_enable(dev);

  return prescaler;
}

static void stm32_tim_setperiod(struct stm32_tim_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev != NULL);

  if (stm32_tim_getwidth(dev) > 16)
    {
      stm32_putreg32(dev, STM32_GTIM_ARR_OFFSET, period);
    }
  else
    {
      stm32_putreg16(dev, STM32_GTIM_ARR_OFFSET, (uint16_t)period);
    }
}

static int stm32_tim_setisr(struct stm32_tim_dev_s *dev,
                            xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  vectorno = ((struct stm32_tim_priv_s *)dev)->config->irq;

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(vectorno, handler, arg);
  up_enable_irq(vectorno);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  return OK;
}

static void stm32_tim_enableint(struct stm32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, 0, source);
}

static void stm32_tim_disableint(struct stm32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, source, 0);
}

static int stm32_tim_checkint(struct stm32_tim_dev_s *dev, int source)
{
  uint16_t regval = stm32_getreg16(dev, STM32_BTIM_SR_OFFSET);

  return (regval & source) ? 1 : 0;
}

static void stm32_tim_ackint(struct stm32_tim_dev_s *dev, int source)
{
  stm32_putreg16(dev, STM32_GTIM_SR_OFFSET, ~source);
}

/****************************************************************************
 * General Functions
 ****************************************************************************/

static int stm32_tim_setmode(struct stm32_tim_dev_s *dev,
                             stm32_tim_mode_t mode)
{
  struct stm32_tim_priv_s *priv = (struct stm32_tim_priv_s *)dev;
  uint16_t val = GTIM_CR1_CEN | GTIM_CR1_ARPE;

  DEBUGASSERT(dev != NULL);

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

  if ((priv->config->flags & STM32_TIM_FLAG_BASIC) != 0)
    {
      return -EINVAL;
    }

  /* Decode operational modes */

  switch (mode & STM32_TIM_MODE_MASK)
    {
      case STM32_TIM_MODE_DISABLED:
        val = 0;
        break;

      case STM32_TIM_MODE_DOWN:
        if ((priv->config->flags & STM32_TIM_FLAG_BIDIRECTIONAL) == 0)
          {
            return -EINVAL;
          }

        val |= GTIM_CR1_DIR;

      case STM32_TIM_MODE_UP:
        break;

      case STM32_TIM_MODE_UPDOWN:
        if ((priv->config->flags & STM32_TIM_FLAG_BIDIRECTIONAL) == 0)
          {
            return -EINVAL;
          }

        val |= GTIM_CR1_CENTER1;

        /* Our default: Interrupts are generated on compare, when counting
         * down
         */

        break;

      case STM32_TIM_MODE_PULSE:
        val |= GTIM_CR1_OPM;
        break;

      default:
        return -EINVAL;
    }

  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);

  /* Timers with break/dead-time logic require Main Output Enable. */

  if ((priv->config->flags & STM32_TIM_FLAG_MOE) != 0)
    {
      stm32_modifyreg16(dev, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }

  return OK;
}

static int stm32_tim_setchannel(struct stm32_tim_dev_s *dev,
                                uint8_t channel, stm32_tim_channel_t mode)
{
  struct stm32_tim_priv_s *priv = (struct stm32_tim_priv_s *)dev;
  uint32_t ccmr_orig;
  uint32_t ccmr_val    = 0;
  uint32_t ccmr_mask   = 0x000100ff;
  uint32_t ccer_val;
  uint8_t  ccmr_offset = STM32_GTIM_CCMR1_OFFSET;

  DEBUGASSERT(dev != NULL);

  /* Validate the one-based channel number before accessing any capture /
   * compare register.  Basic timers have zero channels.
   */

  if (channel == 0 || channel > priv->config->channels)
    {
      return -EINVAL;
    }

  channel--;
  ccer_val = stm32_getreg32(dev, STM32_GTIM_CCER_OFFSET);

  /* Assume that channel is disabled and polarity is active high */

  ccer_val &= ~(3 << GTIM_CCER_CCXBASE(channel));

  /* Decode configuration */

  switch (mode & STM32_TIM_CH_MODE_MASK)
    {
      case STM32_TIM_CH_DISABLED:
        break;

      case STM32_TIM_CH_OUTTOGGLE:
        ccmr_val  = (GTIM_CCMR_MODE_OCREFTOG << GTIM_CCMR1_OC1M_SHIFT);
        ccer_val |= GTIM_CCER_CC1E << GTIM_CCER_CCXBASE(channel);
        break;

      case STM32_TIM_CH_OUTPWM:
        ccmr_val  = (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) +
                    GTIM_CCMR1_OC1PE;
        ccer_val |= GTIM_CCER_CC1E << GTIM_CCER_CCXBASE(channel);
        break;

      default:
        return -EINVAL;
    }

  /* Set polarity */

  if (mode & STM32_TIM_CH_POLARITY_NEG)
    {
      ccer_val |= GTIM_CCER_CC1P << GTIM_CCER_CCXBASE(channel);
    }

  /* Define its position (shift) and get register offset */

  if (channel & 1)
    {
      ccmr_val  <<= 8;
      ccmr_mask <<= 8;
    }

  if (channel > 3)
    {
      ccmr_offset = STM32_ATIM_CCMR3_OFFSET;
    }
  else if (channel > 1)
    {
      ccmr_offset = STM32_GTIM_CCMR2_OFFSET;
    }

  ccmr_orig  = stm32_getreg32(dev, ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  stm32_putreg32(dev, ccmr_offset, ccmr_orig);
  stm32_putreg32(dev, STM32_GTIM_CCER_OFFSET, ccer_val);

  /* Configure only board-defined output pins. */

  if ((priv->config->gpio_mask & (1 << channel)) != 0)
    {
      stm32_tim_gpioconfig(priv->config->gpio[channel], mode);
    }

  return OK;
}

static int stm32_tim_setcompare(struct stm32_tim_dev_s *dev,
                                uint8_t channel, uint32_t compare)
{
  struct stm32_tim_priv_s *priv = (struct stm32_tim_priv_s *)dev;
  uint8_t offset;

  DEBUGASSERT(dev != NULL);

  if (channel == 0 || channel > priv->config->channels)
    {
      return -EINVAL;
    }

  switch (channel)
    {
      case 1:
        offset = STM32_GTIM_CCR1_OFFSET;
        break;

      case 2:
        offset = STM32_GTIM_CCR2_OFFSET;
        break;

      case 3:
        offset = STM32_GTIM_CCR3_OFFSET;
        break;

      case 4:
        offset = STM32_GTIM_CCR4_OFFSET;
        break;

      case 5:
        offset = STM32_ATIM_CCR5_OFFSET;
        break;

      case 6:
        offset = STM32_ATIM_CCR6_OFFSET;
        break;

      default:
        return -EINVAL;
    }

  if (priv->config->width > 16)
    {
      stm32_putreg32(dev, offset, compare);
    }
  else
    {
      stm32_putreg16(dev, offset, (uint16_t)compare);
    }

  return OK;
}

static int stm32_tim_getcapture(struct stm32_tim_dev_s *dev,
                                uint8_t channel)
{
  struct stm32_tim_priv_s *priv = (struct stm32_tim_priv_s *)dev;
  uint8_t offset;

  DEBUGASSERT(dev != NULL);

  /* TIM1/TIM8 channels 5 and 6 are output-compare-only channels. */

  if (channel == 0 || channel > priv->config->channels || channel > 4)
    {
      return -EINVAL;
    }

  switch (channel)
    {
      case 1:
        offset = STM32_GTIM_CCR1_OFFSET;
        break;

      case 2:
        offset = STM32_GTIM_CCR2_OFFSET;
        break;

      case 3:
        offset = STM32_GTIM_CCR3_OFFSET;
        break;

      case 4:
        offset = STM32_GTIM_CCR4_OFFSET;
        break;

      default:
        return -EINVAL;
    }

  return priv->config->width > 16 ? (int)stm32_getreg32(dev, offset) :
                                   (int)stm32_getreg16(dev, offset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct stm32_tim_dev_s *stm32_tim_init(int timer)
{
  struct stm32_tim_priv_s *priv;
  struct stm32_tim_dev_s *dev;

  if (timer < 1 || (unsigned int)timer >=
      sizeof(stm32_tim_devices) / sizeof(stm32_tim_devices[0]))
    {
      return NULL;
    }

  priv = stm32_tim_devices[timer];
  if (priv == NULL)
    {
      return NULL;
    }

  /* Enable power. */

  modifyreg32(priv->config->rcc_enable, 0, priv->config->enable_mask);

  /* Is device already allocated? */

  if (priv->mode != STM32_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  dev = (struct stm32_tim_dev_s *)priv;
  stm32_tim_reset(dev);

  return dev;
}

/* TODO: Detach interrupts, and close down all TIM Channels */

int stm32_tim_deinit(struct stm32_tim_dev_s *dev)
{
  struct stm32_tim_priv_s *priv = (struct stm32_tim_priv_s *)dev;

  DEBUGASSERT(dev != NULL);

  /* Disable power and mark the timer as free. */

  modifyreg32(priv->config->rcc_enable, priv->config->enable_mask, 0);
  priv->mode = STM32_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_STM32_TIM1 || ... || TIM18) */
