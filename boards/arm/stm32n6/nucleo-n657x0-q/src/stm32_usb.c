/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_usb.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbmonitor.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "hardware/stm32n6xxx_ucpd.h"
#include "stm32_otg.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CDCACM_CONSOLE) || defined(CONFIG_CDCACM_COMPOSITE) || \
    defined(CONFIG_SYSTEM_CDCACM) || defined(CONFIG_EXAMPLES_USBSERIAL)
#  error "CN8 board bring-up must be the only CDC-ACM registration owner"
#endif

#if !defined(CONFIG_USBDEV_SELFPOWERED) || \
    defined(CONFIG_USBDEV_REMOTEWAKEUP)
#  error "CN8 requires self-powered descriptors without remote wakeup"
#endif

#define GPIO_TCPP_ENABLE             (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                      GPIO_SPEED_2MHZ | GPIO_OUTPUT_CLEAR | \
                                      GPIO_PORTA | GPIO_PIN7)
#define GPIO_TCPP_FLAG               (GPIO_INPUT | GPIO_FLOAT | \
                                      GPIO_PORTD | GPIO_PIN2)

/* DS13618 Rev 3, sections 6.2/6.3/7.1/7.3. The ACK power-mode encoding
 * differs from the command encoding (also specified by ST's TCPP driver).
 * Every command keeps GDP, VCONN and discharge clear and opens GDC.
 */

#define TCPP_ADDRESS                 0x34
#define TCPP_CONTROL                 0
#define TCPP_ACK                     1
#define TCPP_FLAGS                   2
#define TCPP_LOWPOWER_SINK           0x28
#define TCPP_LOWPOWER_SINK_ACK       0x18
#define TCPP_VBUS_OK                 (1u << 5)
#define TCPP_FAULTS                  0x1f
#define TCPP_TYPE_RESERVED           0xc0

#define CN8_POLL_MS                  20
#define CN8_CC_DEBOUNCE_MS           150
#define CN8_SINK_CR                  (UCPD_CR_ANAMODE_SINK | \
                                      UCPD_CR_CCENABLE_BOTH)
#define CN8_UCPD_CONFIG              UCPD_CFGR1_PSC_DIV2

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum cn8_state_e
{
  CN8_OFF,
  CN8_INITIALIZING,
  CN8_BLOCKED,
  CN8_UNATTACHED,
  CN8_DEBOUNCE_CC1,
  CN8_DEBOUNCE_CC2,
  CN8_ATTACHED_CC1,
  CN8_ATTACHED_CC2,
  CN8_FAULT
};

enum cn8_resource_e
{
  CN8RES_ENABLE = 1u << 0,
  CN8RES_CLOCK = 1u << 1,
  CN8RES_SLEEP_CLOCK = 1u << 2,
  CN8RES_UCPD = 1u << 3
};

struct cn8_usb_s
{
  enum cn8_state_e state;
  unsigned int resources;
  int result;
  clock_t candidate;
  struct i2c_master_s *i2c;
  struct work_s work;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_usb_lock = NXMUTEX_INITIALIZER;
static struct cn8_usb_s g_cn8;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cn8_read(uint8_t reg, uint8_t *value)
{
  struct i2c_msg_s msgs[2] =
  {
    {
      .frequency = I2C_SPEED_STANDARD,
      .addr = TCPP_ADDRESS,
      .flags = 0,
      .buffer = &reg,
      .length = 1
    },
    {
      .frequency = I2C_SPEED_STANDARD,
      .addr = TCPP_ADDRESS,
      .flags = I2C_M_READ,
      .buffer = value,
      .length = 1
    }
  };

  return I2C_TRANSFER(g_cn8.i2c, msgs, 2);
}

static int cn8_program(void)
{
  uint8_t data[2] =
  {
    TCPP_CONTROL, TCPP_LOWPOWER_SINK
  };

  struct i2c_msg_s msg =
  {
    .frequency = I2C_SPEED_STANDARD,
    .addr = TCPP_ADDRESS,
    .flags = 0,
    .buffer = data,
    .length = sizeof(data)
  };

  return I2C_TRANSFER(g_cn8.i2c, &msg, 1);
}

static void cn8_fault(int result)
{
  int ret;

  g_cn8.state = CN8_FAULT;
  g_cn8.result = result;
  syslog(LOG_ERR, "ERROR: CN8 USB policy fault: %d; reboot required\n",
         result);
  ret = stm32_usbdev_vbus(false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: CN8 USB disconnect failed: %d\n", ret);
    }

  /* EN low resets TCPP03 with GDP off and external dead-battery Rd active.
   * CN9 [1-2] does not select its default consumer path for board power.
   */

  if ((g_cn8.resources & CN8RES_ENABLE) != 0)
    {
      stm32_gpiowrite(GPIO_TCPP_ENABLE, false);
    }

  if ((g_cn8.resources & CN8RES_UCPD) != 0)
    {
      putreg32(0, STM32_UCPD_CFGR1);
    }

  if ((g_cn8.resources & CN8RES_SLEEP_CLOCK) != 0)
    {
      putreg32(RCC_APB1HLPENR_UCPD1LPEN, STM32_RCC_APB1HLPENCR);
    }

  if ((g_cn8.resources & CN8RES_CLOCK) != 0)
    {
      putreg32(RCC_APB1HENR_UCPD1EN, STM32_RCC_APB1HENCR);
    }

  g_cn8.resources = 0;
}

static int cn8_ucpd_initialize(void)
{
  uint32_t reg;

  if ((getreg32(STM32_RCC_SR) & RCC_SR_HSIRDY) == 0)
    {
      return -EIO;
    }

  reg = getreg32(STM32_RCC_APB1HENR);
  if ((reg & RCC_APB1HENR_UCPD1EN) == 0)
    {
      g_cn8.resources |= CN8RES_CLOCK;
      putreg32(RCC_APB1HENR_UCPD1EN, STM32_RCC_APB1HENSR);
    }

  if ((getreg32(STM32_RCC_APB1HENR) & RCC_APB1HENR_UCPD1EN) == 0)
    {
      return -EACCES;
    }

  if ((getreg32(STM32_UCPD_CFGR1) & UCPD_CFGR1_UCPDEN) != 0)
    {
      return -EBUSY;
    }

  g_cn8.resources |= CN8RES_UCPD;
  putreg32(RCC_APB1HRSTR_UCPD1RST, STM32_RCC_APB1HRSTSR);
  if ((getreg32(STM32_RCC_APB1HRSTR) & RCC_APB1HRSTR_UCPD1RST) == 0)
    {
      return -EACCES;
    }

  putreg32(RCC_APB1HRSTR_UCPD1RST, STM32_RCC_APB1HRSTCR);
  if ((getreg32(STM32_RCC_APB1HRSTR) & RCC_APB1HRSTR_UCPD1RST) != 0)
    {
      return -EACCES;
    }

  if ((getreg32(STM32_RCC_APB1HLPENR) & RCC_APB1HLPENR_UCPD1LPEN) == 0)
    {
      g_cn8.resources |= CN8RES_SLEEP_CLOCK;
      putreg32(RCC_APB1HLPENR_UCPD1LPEN, STM32_RCC_APB1HLPENSR);
    }

  /* HSI oscillator /4 /2 = 8 MHz. Only analog Type-C detection is used;
   * PD Rx, Tx, DMA, interrupts and VCONN remain disabled.
   */

  putreg32(CN8_UCPD_CONFIG, STM32_UCPD_CFGR1);
  putreg32(0, STM32_UCPD_CFGR2);
  putreg32(CN8_UCPD_CONFIG | UCPD_CFGR1_UCPDEN, STM32_UCPD_CFGR1);
  putreg32(CN8_SINK_CR, STM32_UCPD_CR);
  putreg32(0, STM32_UCPD_IMR);
  if (getreg32(STM32_UCPD_CFGR1) !=
      (CN8_UCPD_CONFIG | UCPD_CFGR1_UCPDEN) ||
      getreg32(STM32_UCPD_CR) != CN8_SINK_CR ||
      (getreg32(STM32_RCC_APB1HLPENR) & RCC_APB1HLPENR_UCPD1LPEN) == 0)
    {
      return -EIO;
    }

  return OK;
}

static int cn8_sample(void)
{
  uint32_t sr;
  unsigned int cc1;
  unsigned int cc2;
  uint8_t ack;
  uint8_t flags;
  int ret;

  ret = cn8_read(TCPP_ACK, &ack);
  if (ret < 0)
    {
      return ret;
    }

  if (ack != TCPP_LOWPOWER_SINK_ACK)
    {
      return -EIO;
    }

  ret = cn8_read(TCPP_FLAGS, &flags);
  if (ret < 0)
    {
      return ret;
    }

  if ((flags & TCPP_TYPE_RESERVED) != 0)
    {
      return -ENODEV;
    }

  if ((flags & TCPP_FAULTS) != 0)
    {
      return -EIO;
    }

  if (getreg32(STM32_UCPD_CR) != CN8_SINK_CR ||
      getreg32(STM32_UCPD_CFGR1) !=
      (CN8_UCPD_CONFIG | UCPD_CFGR1_UCPDEN))
    {
      return -EIO;
    }

  if ((flags & TCPP_VBUS_OK) == 0 || stm32_gpioread(GPIO_TCPP_FLAG))
    {
      return 0;
    }

  sr = getreg32(STM32_UCPD_SR);
  cc1 = (sr >> UCPD_SR_CC1_SHIFT) & UCPD_SR_CC_MASK;
  cc2 = (sr >> UCPD_SR_CC2_SHIFT) & UCPD_SR_CC_MASK;

  /* Exactly one Rp source is required. Reject accessory and invalid states.
   * Do not infer VBUS from CC alone or use the PHY's unwired comparator.
   */

  if (cc1 != 0 && cc2 == 0)
    {
      return 1;
    }

  if (cc2 != 0 && cc1 == 0)
    {
      return 2;
    }

  return 0;
}

static void cn8_poll(void *arg)
{
  enum cn8_state_e debounce;
  enum cn8_state_e attached;
  int ret;
  int cc;

  (void)arg;
  ret = nxmutex_lock(&g_usb_lock);
  if (ret < 0)
    {
      cn8_fault(ret);
      return;
    }

  cc = cn8_sample();
  if (cc < 0)
    {
      cn8_fault(cc);
      goto out;
    }

  debounce = cc == 1 ? CN8_DEBOUNCE_CC1 : CN8_DEBOUNCE_CC2;
  attached = cc == 1 ? CN8_ATTACHED_CC1 : CN8_ATTACHED_CC2;
  if (cc == 0 || (g_cn8.state != debounce && g_cn8.state != attached))
    {
      if (g_cn8.state == CN8_ATTACHED_CC1 ||
          g_cn8.state == CN8_ATTACHED_CC2)
        {
          ret = stm32_usbdev_vbus(false);
          if (ret < 0)
            {
              cn8_fault(ret);
              goto out;
            }

          syslog(LOG_INFO, "CN8 USB detached\n");
        }

      g_cn8.state = cc == 0 ? CN8_UNATTACHED : debounce;
      g_cn8.candidate = clock_systime_ticks();
    }
  else if (g_cn8.state == debounce &&
           (clock_t)(clock_systime_ticks() - g_cn8.candidate) >=
           MSEC2TICK(CN8_CC_DEBOUNCE_MS))
    {
      ret = stm32_usbdev_vbus(true);
      if (ret < 0)
        {
          cn8_fault(ret);
          goto out;
        }

      g_cn8.state = attached;
      syslog(LOG_INFO, "CN8 USB attached on CC%d\n", cc);
    }

  ret = work_queue(LPWORK, &g_cn8.work, cn8_poll, NULL,
                   MSEC2TICK(CN8_POLL_MS));
  if (ret < 0)
    {
      cn8_fault(ret);
    }

out:
  nxmutex_unlock(&g_usb_lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nucleo_usbdev_initialize(void)
{
  int ret;

  if (up_interrupt_context())
    {
      return -EWOULDBLOCK;
    }

  if ((getcontrol() & CONTROL_NPRIV) != 0)
    {
      return -EACCES;
    }

  ret = nxmutex_lock(&g_usb_lock);

  if (ret < 0)
    {
      return ret;
    }

  if (g_cn8.state != CN8_OFF)
    {
      ret = g_cn8.result;
      goto out;
    }

  g_cn8.state = CN8_INITIALIZING;
  ret = stm32_configgpio(GPIO_TCPP_ENABLE);
  if (ret < 0)
    {
      goto fail;
    }

  g_cn8.resources |= CN8RES_ENABLE;
  ret = stm32_usbdev_vbus(false);
  if (ret < 0)
    {
      goto fail;
    }

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      goto fail;
    }

#ifdef CONFIG_USBMONITOR
  ret = usbmonitor_start();
  if (ret < 0)
    {
      goto fail;
    }
#endif

#ifndef CONFIG_NUCLEO_N657X0_Q_USBDEV_QUALIFIED
  g_cn8.state = CN8_BLOCKED;
  g_cn8.result = -EAGAIN;
  syslog(LOG_WARNING,
         "CN8 CDC /dev/ttyACM0 registered; "
         "attachment qualification missing\n");
  ret = -EAGAIN;
  goto out;
#endif

  ret = nucleo_i2c_initialize();
  if (ret < 0)
    {
      goto fail;
    }

  g_cn8.i2c = nucleo_i2c2_bus();
  if (g_cn8.i2c == NULL)
    {
      ret = -ENODEV;
      goto fail;
    }

  ret = stm32_configgpio(GPIO_TCPP_FLAG);
  if (ret < 0)
    {
      goto fail;
    }

  ret = cn8_ucpd_initialize();
  if (ret < 0)
    {
      goto fail;
    }

  stm32_gpiowrite(GPIO_TCPP_ENABLE, true);
  ret = nxsig_usleep(2000);
  if (ret < 0)
    {
      goto fail;
    }

  ret = cn8_program();
  if (ret < 0)
    {
      goto fail;
    }

  ret = nxsig_usleep(2000);
  if (ret < 0)
    {
      goto fail;
    }

  ret = cn8_sample();
  if (ret < 0)
    {
      goto fail;
    }

  g_cn8.state = CN8_UNATTACHED;
  ret = work_queue(LPWORK, &g_cn8.work, cn8_poll, NULL, 0);
  if (ret < 0)
    {
      goto fail;
    }

  g_cn8.result = OK;
  syslog(LOG_INFO, "CN8 CDC /dev/ttyACM0 registered; static sink polling\n");
  goto out;

fail:
  cn8_fault(ret);
out:
  nxmutex_unlock(&g_usb_lock);
  return ret;
}

void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  (void)dev;
  syslog(LOG_INFO, "CN8 USB %s; clocks retained\n",
         resume ? "resume" : "suspend");
}
