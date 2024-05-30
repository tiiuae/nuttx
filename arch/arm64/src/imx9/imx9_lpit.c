/****************************************************************************
 * arch/arm64/src/imx9/imx9_lpit.c
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

#include <sys/types.h>
#include <errno.h>
#include <sys/types.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "arm64_arch.h"
#include "imx9_ccm.h"
#include "hardware/imx9_ccm.h"
#include "imx9_lpit.h"
#include "hardware/imx9_lpit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define N_TIMER_CH 4
#define AON_CLK_FREQ       133333333
#define WAKEUP_CLK_FREQ    133333333

#ifndef nitems
#define nitems(_a)    (sizeof(_a) / sizeof(0[(_a)]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_lpit_s
{
  const uintptr_t base;
  const int clk_root;
  const int clk_gate;
  const int clk;
  const int isr;
  const int trg_channel;
  bool reserved;
  int n_in_use;
  bool in_use[N_TIMER_CH];
  lpit_callback_t timer_isr[N_TIMER_CH];
  void *timer_isr_arg[N_TIMER_CH];
  mutex_t lock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imx9_lpit_s g_lpit[] =
{
#ifdef CONFIG_IMX9_LPIT1
  {
    .base = IMX9_LPIT1_BASE,
    .clk_root = CCM_CR_LPIT1,
    .clk_gate = CCM_LPCG_LPIT1,
    .clk = AON_CLK_FREQ,
    .isr = IMX9_IRQ_LPIT1,
    .trg_channel = 6,
    .lock = NXMUTEX_INITIALIZER,
  },
#endif

#ifdef CONFIG_IMX9_LPIT2
  {
    .base = IMX9_LPIT2_BASE,
    .clk_root = CCM_CR_LPIT2,
    .clk_gate = CCM_LPCG_LPIT2,
    .clk = WAKEUP_CLK_FREQ,
    .isr = IMX9_IRQ_LPIT2,
    .trg_channel = 7,
    .lock = NXMUTEX_INITIALIZER,
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void reg_write32(struct imx9_lpit_s *lpit, off_t offset, uint32_t val)
{
  putreg32(val, lpit->base + offset);
}

static uint32_t reg_read32(struct imx9_lpit_s *lpit, off_t offset)
{
  return getreg32(lpit->base + offset);
}

static void reg_modify32(struct imx9_lpit_s *lpit, off_t offset,
                         uint32_t clear, uint32_t set)
{
  uint32_t val = reg_read32(lpit, offset);

  reg_write32(lpit, offset, (val & (~clear)) | set);
}

static void imx9_lpit_enable(struct imx9_lpit_s *lpit)
{
  /* Reset the module */

  reg_write32(lpit, IMX9_LPIT_MCR_OFFSET, LPIT_MCR_SW_RST);

  /* Wait for reset, 4 functional clock periods */

  up_udelay(1);

  /* Clear reset and enable module clock */

  reg_write32(lpit, IMX9_LPIT_MCR_OFFSET,  LPIT_MCR_M_CEN);
};

static int imx9_lpit_isr(int irq, void *context, void *arg)
{
  struct imx9_lpit_s *lpit = (struct imx9_lpit_s *)arg;
  uint32_t msr;
  int ch;
  int ret = -ERROR;

  DEBUGASSERT(lpit != NULL);

  /* Check which channels generated an interrupt and perform callbacks */

  msr = reg_read32(lpit, IMX9_LPIT_MSR_OFFSET);

  /* Clear the interrupts */

  reg_write32(lpit, IMX9_LPIT_MSR_OFFSET, msr);

  for (ch = 0; ch < N_TIMER_CH; ch++)
    {
      if ((msr & (1 << ch)) && lpit->timer_isr[ch])
        {
          lpit->timer_isr[ch](lpit->timer_isr_arg[ch]);
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imx9_lpit_reserve(lpit_handle_t *handle)
{
  struct imx9_lpit_s *lpit;
  int ret;
  int i;

  ret = -ENXIO;
  for (i = 0; i < nitems(g_lpit); i++)
    {
      lpit = &g_lpit[i];
      ret = nxmutex_lock(&lpit->lock);
      if (ret < 0)
        {
          return ret;
        }

      if (lpit->n_in_use == 0)
        {
          lpit->reserved = true;
          handle->reserved = true;
          handle->lpit = i;
          break;
        }

      nxmutex_unlock(&lpit->lock);
    }

  return OK;
}

int imx9_lpit_release(lpit_handle_t *handle, int reserved_timer)
{
  struct imx9_lpit_s *lpit;
  int ret;
  int ch;

  /* Sanity check: caller has reserved this timer */

  if (!handle->reserved || reserved_timer != handle->lpit)
    {
      return -EINVAL;
    }

  lpit = &g_lpit[reserved_timer];
  ret = nxmutex_lock(&lpit->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Release all channels within the timer */

  for (ch = 0; ch < N_TIMER_CH; ch++)
    {
      if (lpit->in_use[ch])
        {
          imx9_lpit_release_ch(handle, ch);
        }
    }

  /* Leave module in reset */

  reg_write32(lpit, IMX9_LPIT_MCR_OFFSET, LPIT_MCR_SW_RST);

  /* Disable peripheral clock. */

  imx9_ccm_gate_on(lpit->clk_gate , false);

  handle->lpit = -1;
  handle->ch_mask = 0;
  handle->reserved = false;

  nxmutex_unlock(&lpit->lock);

  return OK;
}

int imx9_lpit_reserve_periodic_ch(lpit_handle_t *handle, uint32_t freq,
                                  lpit_callback_t isr, void *arg)
{
  struct imx9_lpit_s *lpit;
  int ret;
  int timer = -1;
  int i;
  int ch;

  /* Frequency of 0 is not allowed */

  if (freq == 0)
    {
      return -EINVAL;
    }

  /* Find a free timer channel, either from the reserved timer or
   * any free one.
   */

  i = handle->lpit >= 0 && g_lpit[handle->lpit].reserved ? handle->lpit : 0;

  do
    {
      lpit = &g_lpit[i];

      ret = nxmutex_lock(&lpit->lock);
      if (ret < 0)
        {
          return ret;
        }

      /* Skip this timer if it is reserved for someone else */

      if (!lpit->reserved || handle->reserved)
        {
          /* Find a free channel within the timer */

          for (ch = 0; ch < N_TIMER_CH; ch++)
            {
              if (!lpit->in_use[ch])
                {
                  timer = i;
                  break;
                }
            }
        }

      if (timer >= 0)
        {
          /* Timer was found, break out and leave timer locked */

          break;
        }

      nxmutex_unlock(&lpit->lock);
    }
  while (++i < nitems(g_lpit) && !handle->reserved);

  /* Return error if free timer was not found */

  if (timer < 0)
    {
      return -ENODEV;
    }

  /* If this is the first timer on this LPIT, configure timer global
   * registers
   */

  if (lpit->n_in_use == 0)
    {
      /* Enable peripheral clock. It is always AON for LPIT1 and WAKEUP
       * for LPIT2
       */

      imx9_ccm_gate_on(lpit->clk_gate , true);

      /* Reset and enable module */

      imx9_lpit_enable(lpit);
    }

  if (isr)
    {
      ret = irq_attach(lpit->isr, imx9_lpit_isr, lpit);

      if (ret < 0)
        {
          nxmutex_unlock(&lpit->lock);

          return ret;
        }
    }

  lpit->timer_isr[ch] = isr;
  lpit->timer_isr_arg[ch] = arg;
  lpit->in_use[ch] = true;
  lpit->n_in_use++;

  /* Clear any pending channel interrupt in MSR */

  reg_modify32(lpit, IMX9_LPIT_MSR_OFFSET, 0, 1 << ch);

  /* Enable channel interrupt in MIER */

  reg_modify32(lpit, IMX9_LPIT_MIER_OFFSET, 0, 1 << ch);

  /* Set timer channel value */

  reg_write32(lpit, IMX9_LPIT_TVAL_OFFSET(ch), lpit->clk / freq);

  /* Set to internal trigger, and enable. This setting
   * provides periodic trigger with automatic reload:
   * MODE = 0, TSOT = 0, TROT = 0, TSOI  = 0, CHAIN = 0
   */

  reg_write32(lpit, IMX9_LPIT_TCTRL_OFFSET(ch),
              LPIT_TCTRL_TRG_SRC_INTER | LPIT_TCTRL_T_EN);

  nxmutex_unlock(&lpit->lock);

  /* Update handle */

  handle->lpit = timer;
  handle->ch_mask |= 1 << ch;

  /* Enable interrupt */

  if (isr)
    {
      up_enable_irq(lpit->isr);
    }

  /* Return the hardware trigger channel */

  return lpit->trg_channel;
}

int imx9_lpit_release_ch(lpit_handle_t *handle, int ch)
{
  /* TODO: This is unimplemented */

  /* stop channel */

  handle->ch_mask &= ~(1 << ch);

  /* TODO: interrupt source tracking, disable interrupt if no longer in use */

  return OK;
}
