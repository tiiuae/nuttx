/****************************************************************************
 * arch/arm64/src/imx9/imx9_adc.c
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

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_adc.h"
#include "hardware/imx9_adc.h"
#include "hardware/imx9_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_ADC_POWERDOWN_TIMEOUT_US 50
#define IMX9_ADC_CAL_TIMEOUT_US      2000000
#define IMX9_ADC_CONV_TIMEOUT_US     100000
#define IMX9_ADC_CAL_MAX_RETRIES          5
#define IMX9_ADC_CAL_RETRY_DELAY_US     200
#define IMX9_ADC_NCHANNELS                8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_adc_lower_s
{
  struct adc_dev_s dev;
  FAR const struct adc_callback_s *cb;
  uintptr_t base;
  bool configured;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int imx9_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void imx9_adc_reset(FAR struct adc_dev_s *dev);
static int imx9_adc_setup(FAR struct adc_dev_s *dev);
static void imx9_adc_shutdown(FAR struct adc_dev_s *dev);
static void imx9_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int imx9_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);
static int imx9_adc_power_down(FAR struct imx9_adc_lower_s *priv);
static int imx9_adc_power_up(FAR struct imx9_adc_lower_s *priv);
static int imx9_adc_clk_config(FAR struct imx9_adc_lower_s *priv);
static int imx9_adc_calibration(FAR struct imx9_adc_lower_s *priv);
static int imx9_adc_read_raw(FAR struct imx9_adc_lower_s *priv,
                             uint8_t channel, FAR uint16_t *result);
static int imx9_adc_trigger_scan(FAR struct imx9_adc_lower_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_imx9_adc_ops =
{
  .ao_bind     = imx9_adc_bind,
  .ao_reset    = imx9_adc_reset,
  .ao_setup    = imx9_adc_setup,
  .ao_shutdown = imx9_adc_shutdown,
  .ao_rxint    = imx9_adc_rxint,
  .ao_ioctl    = imx9_adc_ioctl,
};

static struct imx9_adc_lower_s g_imx9_adc =
{
  .dev =
    {
      .ad_ops  = &g_imx9_adc_ops,
      .ad_priv = NULL,
    },
  .cb         = NULL,
  .base       = IMX9_ADC1_BASE,
  .configured = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool g_adc_hw_enabled;

static int imx9_adc_require_hw_enabled(void)
{
  return g_adc_hw_enabled ? OK : -EACCES;
}

static int imx9_adc_hw_enable(void)
{
  int ret;

  if (g_adc_hw_enabled)
    {
      return OK;
    }

  ret = imx9_ccm_configure_root_clock(CCM_CR_ADC, OSC_24M, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = imx9_ccm_gate_on(CCM_LPCG_ADC1, true);
  if (ret < 0)
    {
      return ret;
    }

  g_adc_hw_enabled = true;

  return OK;
}

static int imx9_adc_hw_disable(void)
{
  int ret;

  if (!g_adc_hw_enabled)
    {
      return OK;
    }

  ret = imx9_ccm_gate_on(CCM_LPCG_ADC1, false);
  if (ret < 0)
    {
      return ret;
    }

  g_adc_hw_enabled = false;

  return OK;
}

static int imx9_adc_power_down(FAR struct imx9_adc_lower_s *priv)
{
  uint32_t mcr;
  uint32_t msr;
  uintptr_t base = priv->base;
  int timeout;
  int ret;

  ret = imx9_adc_require_hw_enabled();
  if (ret < 0)
    {
      return ret;
    }

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr |= IMX9_ADC_MCR_PWDN_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  for (timeout = 0; timeout < IMX9_ADC_POWERDOWN_TIMEOUT_US; timeout++)
    {
      msr = getreg32(base + IMX9_ADC_MSR_OFFSET);
      if ((msr & IMX9_ADC_MSR_ADCSTATUS_MASK) ==
          IMX9_ADC_MSR_ADCSTATUS_POWER_DOWN)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static int imx9_adc_power_up(FAR struct imx9_adc_lower_s *priv)
{
  uint32_t mcr;
  uintptr_t base = priv->base;
  int ret;

  ret = imx9_adc_require_hw_enabled();
  if (ret < 0)
    {
      return ret;
    }

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr &= ~IMX9_ADC_MCR_PWDN_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  return OK;
}

static int imx9_adc_clk_config(FAR struct imx9_adc_lower_s *priv)
{
  uint32_t mcr;
  uintptr_t base = priv->base;
  int ret;

  ret = imx9_adc_power_down(priv);
  if (ret < 0)
    {
      return ret;
    }

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr |= IMX9_ADC_MCR_ADCLKSE_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  return imx9_adc_power_up(priv);
}

static int imx9_adc_calibration(FAR struct imx9_adc_lower_s *priv)
{
  uint32_t mcr;
  uint32_t msr;
  uintptr_t base = priv->base;
  int ret;
  int timeout;

  ret = imx9_adc_power_down(priv);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(0, base + IMX9_ADC_NCMR0_OFFSET);
  putreg32(0, base + IMX9_ADC_CIMR0_OFFSET);
  putreg32(0, base + IMX9_ADC_IMR_OFFSET);
  putreg32(IMX9_ADC_ISR_EOC_ECH_MASK, base + IMX9_ADC_ISR_OFFSET);

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr &= ~IMX9_ADC_MCR_MODE_MASK;
  mcr |= IMX9_ADC_MCR_ADCLKSE_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  ret = imx9_adc_power_up(priv);
  if (ret < 0)
    {
      return ret;
    }

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr |= IMX9_ADC_MCR_CALSTART_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  for (timeout = 0; timeout < IMX9_ADC_CAL_TIMEOUT_US; timeout++)
    {
      msr = getreg32(base + IMX9_ADC_MSR_OFFSET);
      if ((msr & IMX9_ADC_MSR_CALBUSY_MASK) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout >= IMX9_ADC_CAL_TIMEOUT_US)
    {
      imx9_adc_power_down(priv);
      return -ETIMEDOUT;
    }

  msr = getreg32(base + IMX9_ADC_MSR_OFFSET);
  if ((msr & IMX9_ADC_MSR_CALFAIL_MASK) != 0)
    {
      imx9_adc_power_down(priv);
      return -EAGAIN;
    }

  return OK;
}

static int imx9_adc_read_raw(FAR struct imx9_adc_lower_s *priv,
                             uint8_t channel, FAR uint16_t *result)
{
  uint32_t channel_mask;
  uint32_t imr;
  uint32_t mcr;
  uint32_t isr;
  uint32_t pcdr;
  uintptr_t base;
  int timeout;
  int ret;

  if (channel > 7 || result == NULL)
    {
      return -EINVAL;
    }

  if (!priv->configured)
    {
      return -EACCES;
    }

  ret = imx9_adc_require_hw_enabled();
  if (ret < 0)
    {
      return ret;
    }

  base = priv->base;

  channel_mask = 1u << channel;
  putreg32(channel_mask, base + IMX9_ADC_NCMR0_OFFSET);

  imr = IMX9_ADC_IMR_EOC_MASK;
  putreg32(imr, base + IMX9_ADC_IMR_OFFSET);
  putreg32(channel_mask, base + IMX9_ADC_CIMR0_OFFSET);

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr &= ~IMX9_ADC_MCR_MODE_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  mcr = getreg32(base + IMX9_ADC_MCR_OFFSET);
  mcr |= IMX9_ADC_MCR_NSTART_MASK;
  putreg32(mcr, base + IMX9_ADC_MCR_OFFSET);

  for (timeout = 0; timeout < IMX9_ADC_CONV_TIMEOUT_US; timeout++)
    {
      isr = getreg32(base + IMX9_ADC_ISR_OFFSET);
      if ((isr & IMX9_ADC_ISR_EOC_ECH_MASK) != 0)
        {
          putreg32(isr & IMX9_ADC_ISR_EOC_ECH_MASK,
                   base + IMX9_ADC_ISR_OFFSET);
          pcdr = getreg32(base + IMX9_ADC_PCDR_OFFSET(channel));
          *result = (uint16_t)(pcdr & IMX9_ADC_PCDR_CDATA_MASK);
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static int imx9_adc_trigger_scan(FAR struct imx9_adc_lower_s *priv)
{
  FAR const struct adc_callback_s *cb = priv->cb;
  uint16_t result;
  int ret;
  int first_error = OK;
  uint8_t channel;

  if (cb == NULL || cb->au_receive == NULL)
    {
      return -ENOSYS;
    }

  for (channel = 0; channel < IMX9_ADC_NCHANNELS; channel++)
    {
      ret = imx9_adc_read_raw(priv, channel, &result);
      if (ret < 0)
        {
          if (first_error == OK)
            {
              first_error = ret;
            }

          continue;
        }

      ret = cb->au_receive(&priv->dev, channel, (int32_t)result);
      if (ret < 0)
        {
          return ret;
        }
    }

  return first_error;
}

static int imx9_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback)
{
  FAR struct imx9_adc_lower_s *priv =
    (FAR struct imx9_adc_lower_s *)dev->ad_priv;

  priv->cb = callback;
  return OK;
}

static void imx9_adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct imx9_adc_lower_s *priv =
    (FAR struct imx9_adc_lower_s *)dev->ad_priv;

  if (priv->configured)
    {
      imx9_adc_power_down(priv);
    }

  priv->configured = false;
}

static int imx9_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct imx9_adc_lower_s *priv =
    (FAR struct imx9_adc_lower_s *)dev->ad_priv;
  int ret;
  int retries;

  if (priv->configured)
    {
      return OK;
    }

  ret = imx9_adc_hw_enable();
  if (ret < 0)
    {
      return ret;
    }

  ret = -EAGAIN;
  retries = IMX9_ADC_CAL_MAX_RETRIES;

  while (retries-- > 0)
    {
      ret = imx9_adc_calibration(priv);
      if (ret == OK)
        {
          break;
        }

      if (ret != -EAGAIN)
        {
          break;
        }

      up_udelay(IMX9_ADC_CAL_RETRY_DELAY_US);
    }

  if (ret < 0)
    {
      imx9_adc_hw_disable();
      return ret;
    }

  ret = imx9_adc_clk_config(priv);
  if (ret < 0)
    {
      imx9_adc_power_down(priv);
      imx9_adc_hw_disable();
      return ret;
    }

  priv->configured = true;
  return OK;
}

static void imx9_adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct imx9_adc_lower_s *priv =
    (FAR struct imx9_adc_lower_s *)dev->ad_priv;

  if (priv->configured)
    {
      imx9_adc_power_down(priv);
      priv->configured = false;
    }

  imx9_adc_hw_disable();
}

static void imx9_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct imx9_adc_lower_s *priv =
    (FAR struct imx9_adc_lower_s *)dev->ad_priv;

  if (enable)
    {
      imx9_adc_trigger_scan(priv);
    }
}

static int imx9_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg)
{
  FAR struct imx9_adc_lower_s *priv =
    (FAR struct imx9_adc_lower_s *)dev->ad_priv;

  (void)arg;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        return imx9_adc_trigger_scan(priv);

      case ANIOC_GET_NCHANNELS:
        return IMX9_ADC_NCHANNELS;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct adc_dev_s *imx9_adc_initialize(uintptr_t base)
{
  if (base != 0)
    {
      g_imx9_adc.base = base;
    }

  g_imx9_adc.dev.ad_priv = &g_imx9_adc;
  return &g_imx9_adc.dev;
}

int imx9_adc_read_channel(FAR struct adc_dev_s *dev, uint8_t channel,
                          FAR uint16_t *result)
{
  FAR struct imx9_adc_lower_s *priv;

  if (dev == NULL || result == NULL)
    {
      return -EINVAL;
    }

  priv = (FAR struct imx9_adc_lower_s *)dev->ad_priv;
  if (priv == NULL)
    {
      return -EINVAL;
    }

  return imx9_adc_read_raw(priv, channel, result);
}
