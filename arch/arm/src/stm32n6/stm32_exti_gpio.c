/****************************************************************************
 * arch/arm/src/stm32n6/stm32_exti_gpio.c
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
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include <errno.h>
#include <stdint.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "hardware/stm32n6xxx_exti.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
  xcpt_t callback;
  void  *arg;
  bool   attached;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gpio_callback_s g_gpio_callbacks[16];
static spinlock_t g_gpio_exti_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_exti_gpio_isr(int irq, void *context, void *arg)
{
  struct gpio_callback_s *cb;
  xcpt_t callback;
  void *cbarg;
  unsigned int pin;
  uint32_t mask;

  pin = irq - STM32_IRQ_EXTI0;
  DEBUGASSERT(pin < 16);
  if (pin >= 16)
    {
      return -EINVAL;
    }

  mask = STM32_EXTI_GPIO_LINE_MASK(pin);

  /* Both edge-pending registers are write-one-to-clear. */

  putreg32(mask, STM32_EXTI_RPR1);
  putreg32(mask, STM32_EXTI_FPR1);

  cb       = &g_gpio_callbacks[pin];
  callback = cb->callback;
  cbarg    = cb->arg;

  if (callback != NULL)
    {
      return callback(irq, context, cbarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpiosetevent
 *
 * Description:
 *   Configure the GPIO-backed EXTI event and interrupt triggers for one pin.
 *
 ****************************************************************************/

int stm32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg)
{
  irqstate_t flags;
  uint32_t mask;
  uint32_t pin;
  uint32_t port;
  uint32_t input_pinset;
  int irq;
  int ret;
  bool attached_new = false;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  if (port >= STM32_NPORTS)
    {
      return -EINVAL;
    }

  mask = STM32_EXTI_GPIO_LINE_MASK(pin);
  irq  = STM32_IRQ_EXTI0 + pin;

  flags = spin_lock_irqsave(&g_gpio_exti_lock);

  /* Mask the peripheral interrupt and NVIC line before changing callback or
   * trigger state.  spin_lock_irqsave() also prevents an in-flight callback
   * from racing these changes.
   */

  up_disable_irq(irq);
  modifyreg32(STM32_EXTI_IMR1, mask, 0);

  if (func != NULL && !g_gpio_callbacks[pin].attached)
    {
      ret = irq_attach(irq, stm32_exti_gpio_isr, NULL);
      if (ret < 0)
        {
          if (g_gpio_callbacks[pin].callback != NULL)
            {
              modifyreg32(STM32_EXTI_IMR1, 0, mask);
              up_enable_irq(irq);
            }

          spin_unlock_irqrestore(&g_gpio_exti_lock, flags);
          return ret;
        }

      g_gpio_callbacks[pin].attached = true;
      attached_new = true;
    }

  if (event || func != NULL)
    {
      input_pinset = (pinset & ~GPIO_MODE_MASK) | GPIO_INPUT | GPIO_EXTI;
      ret = stm32_configgpio(input_pinset);
      if (ret < 0)
        {
          if (attached_new)
            {
              if (irq_detach(irq) >= 0)
                {
                  g_gpio_callbacks[pin].attached = false;
                }
            }

          if (g_gpio_callbacks[pin].callback != NULL)
            {
              modifyreg32(STM32_EXTI_IMR1, 0, mask);
              up_enable_irq(irq);
            }

          spin_unlock_irqrestore(&g_gpio_exti_lock, flags);
          return ret;
        }
    }

  if (func == NULL)
    {
      g_gpio_callbacks[pin].callback = NULL;
      g_gpio_callbacks[pin].arg      = NULL;

      if (g_gpio_callbacks[pin].attached)
        {
          ret = irq_detach(irq);
          if (ret >= 0)
            {
              g_gpio_callbacks[pin].attached = false;
            }
        }
      else
        {
          ret = OK;
        }
    }
  else
    {
      g_gpio_callbacks[pin].callback = func;
      g_gpio_callbacks[pin].arg      = arg;
      ret = OK;
    }

  modifyreg32(STM32_EXTI_RTSR1,
              risingedge ? 0 : mask, risingedge ? mask : 0);
  modifyreg32(STM32_EXTI_FTSR1,
              fallingedge ? 0 : mask, fallingedge ? mask : 0);
  modifyreg32(STM32_EXTI_EMR1, event ? 0 : mask, event ? mask : 0);

  /* Discard pending edges accumulated while the line was masked. */

  putreg32(mask, STM32_EXTI_RPR1);
  putreg32(mask, STM32_EXTI_FPR1);

  if (func != NULL)
    {
      modifyreg32(STM32_EXTI_IMR1, 0, mask);
      up_enable_irq(irq);
    }

  spin_unlock_irqrestore(&g_gpio_exti_lock, flags);
  return ret;
}
