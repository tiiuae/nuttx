/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_corepwm.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <time.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "hardware/mpfs_corepwm.h"

#include "riscv_arch.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef OK
#  define OK 0
#endif

/* This module only compiles if at least one CorePWM instance
 * is configured to the FPGA
 */

#ifndef CONFIG_MPFS_HAVE_COREPWM
#error This should not be compiled as CorePWM block is not defined/configured
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpfs_pwmchan_s
{
  uint8_t channel;                     /* Timer output channel: {1,..16} */
};

/* This structure represents the state of one PWM timer */

struct mpfs_pwmtimer_s
{
  FAR const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t nchannels;                   /* Number of channels on this PWM block */
  uint8_t pwmid;                       /* PWM ID {1,...} */
  struct mpfs_pwmchan_s channels[MPFS_MAX_PWM_CHANNELS];
  uint32_t frequency;                  /* Current frequency setting */
  uintptr_t base;                      /* The base address of the pwm block */
  uint32_t pwmclk;                     /* The frequency of the pwm clock */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct mpfs_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct mpfs_pwmtimer_s *priv, int offset, uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct mpfs_pwmtimer_s *priv, FAR const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_timer(FAR struct mpfs_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info);

/* PWM driver methods */

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

#ifdef CONFIG_MPFS_COREPWM0
static struct mpfs_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
  .nchannels   = CONFIG_MPFS_COREPWM0_NCHANNELS,
  .pwmid       = 0,
  .channels    =
  {
    {
      .channel = 1,
    }
  },
  .base        = CONFIG_MPFS_COREPWM0_BASE,
  .pwmclk      = CONFIG_MPFS_COREPWM0_PWMCLK,
};
#endif

#ifdef CONFIG_MPFS_COREPWM1
static struct mpfs_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
  .nchannels   = CONFIG_MPFS_COREPWM1_NCHANNELS,
  .pwmid       = 1,
  .channels    =
  {
    {
      .channel = 1,
    }
  },
  .base        = CONFIG_MPFS_COREPWM1_BASE,
  .pwmclk      = CONFIG_MPFS_COREPWM1_PWMCLK,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pwm_getreg(struct mpfs_pwmtimer_s *priv, int offset)
{
  pwminfo("pwm_getreg: %p, %d\n", priv->base, offset);
  uint32_t uu;
  uu = *((volatile uint32_t *)(0x44000000));
  pwminfo("uu=0x%x\n", uu);
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_putreg(struct mpfs_pwmtimer_s *priv, int offset,
                       uint32_t value)
{
  /* TODO: 8,16 & 32 bit reg width consideration
    * a 32 bit access is required for a 32 bit register
    */

  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *
 * Returned Value:
 *   None
 *
 * TODO: Add TACH* register if tachometer feature is taken in use
 * TODO: Add DAC* register if DA feature is taken in use
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct mpfs_pwmtimer_s *priv, FAR const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  PRESCALE: %08x PERIOD: %08x\n",
          pwm_getreg(priv, MPFS_COREPWM_PRESCALE_OFFSET),
          pwm_getreg(priv, MPFS_COREPWM_PERIOD_OFFSET));
  pwminfo("  SYNC_UPDATE: %02x\n",
          pwm_getreg(priv, MPFS_COREPWM_SYNC_UPDATE_OFFSET));
  pwminfo("  PWM_ENABLE_0_7: %02x PWM_ENABLE_8_15: %02x\n",
          pwm_getreg(priv, MPFS_COREPWM_PWM_ENABLE_0_7_OFFSET),
          pwm_getreg(priv, MPFS_COREPWM_PWM_ENABLE_8_15_OFFSET));

  for (int i = 0; i < 16; i++)
  {
    pwminfo("  PWM%d_POSEDGE: %08x PWM%d_NEGEDGE: %08x\n",
          i + 1,
          pwm_getreg(priv, MPFS_COREPWM_PWM1_POS_EDGE_OFFSET + (i * 4)),
          i + 1,
          pwm_getreg(priv, MPFS_COREPWM_PWM1_NEG_EDGE_OFFSET + (i * 4)));
  }
}
#endif

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_timer(FAR struct mpfs_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info)
{
  int      i;

  /* Calculated values */

  /* TODO: We might need to calculate prescaler on some rare cases,
   * for now hardcoded to 0
   */
  uint32_t prescaler = 0;

  uint32_t period;

  DEBUGASSERT(priv != NULL && info != NULL);
  DEBUGASSERT(info->frequency > 0);


  /* CorePWM FPGA block can be configured to be with either 8, 16, or 32 bit
   * registers width. Minimally PWM functionality is set up by two registers:
   * PRESCALE and PERIOD which are common to all channels. Up to 16 channels
   * may be configured in use. Clock used by the block may be selected in design
   * phase an on Icicle Kit reference design version 21.04 has at least the
   * following clock signals to choose from the Clocks_and_Resets block:
   * 125MHz, 100MHz, 75MHz, 62.5MHz, 50MHz, and 25MHz.
   *
   * For now only 32 the bit configuration is supported.
   * TODO: Add 8 and 16 bit width support
   *
   * There are many combinations of prescaler and period registers, but the best
   * will be the one that has the smallest prescaler value. That is the solution
   * that should give us the most accuracy in the pwm control.
   *
   * Example for clk = 25MHz and using 32 bit wide registers:
   *   PWM period granularity PWM_PG = (PRESCALE + 1) / pwmclk =
   *   40 ns × 1 = 40 ns, so the smallest step with PRESCALE = 0 is 40ns
   *   pwmclk = clk / (PRESCALE + 1) = 25,000,000 / (PRESCALE + 1)
   *
   *    For desired output frequency of 50Hz and using PRESCALE = 0
   *    PERIOD = pwmclk / frequency = 25,000,000 / 50 = 500,000
   */

  pwminfo("PWM%u frequency: %u PWMCLK: %u prescaler: %u\n",
          priv->pwmid, info->frequency, priv->pwmclk, prescaler);

  /* Set the reload and prescaler values */

  period = priv->pwmclk / info->frequency;

  pwm_putreg(priv, MPFS_COREPWM_PERIOD_OFFSET, period);
  pwm_putreg(priv, MPFS_COREPWM_PRESCALE_OFFSET, prescaler);

  /* Handle channel specific setup */

	for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
  {
    ub32_t    duty;
    uint8_t   channel;
    uint32_t  neg_edge;

    channel   = info->channels[i].channel;

    /* Duty defined as fraction of 65536, i.e. a value of 1 to 65535
     * corresponding to a duty cycle of 0.000015 - 0.999984
     */
    duty      = ub16toub32(info->channels[i].duty);
    neg_edge  = b32toi(duty * period + b32HALF);

    if (channel == 0)   /* A value of zero means to skip this channel */
    {
      continue;
    }

    if (channel > MPFS_MAX_PWM_CHANNELS)
    {
      pwmerr("ERROR: No such PWM channel: %u\n", channel);
      return -EINVAL;
    }

    pwminfo("duty: %u, neg_edge: %u\n",
      info->channels[i].duty, neg_edge);

    /* Set the channels duty cycle by writing to the NEG_EDGE register
      * for this channel
      */
    const int neg_edge_reg_offset =
      MPFS_COREPWM_PWM1_NEG_EDGE_OFFSET +
      (MPFS_COREPWM_PWM2_NEG_EDGE_OFFSET -
        MPFS_COREPWM_PWM1_NEG_EDGE_OFFSET) * (channel - 1);

    pwm_putreg(priv, neg_edge_reg_offset, neg_edge);

    /* Enable the channel */

    if (channel <= 8)
    {
      uint32_t reg = pwm_getreg(priv, MPFS_COREPWM_PWM_ENABLE_0_7_OFFSET);
      pwm_putreg(priv, MPFS_COREPWM_PWM_ENABLE_0_7_OFFSET,
                 reg | (1 << (channel - 1)));
    }
    else
    {
      uint32_t reg = pwm_getreg(priv, MPFS_COREPWM_PWM_ENABLE_8_15_OFFSET);
      pwm_putreg(priv, MPFS_COREPWM_PWM_ENABLE_8_15_OFFSET,
                 reg | (1 << (channel - 9)));
    }
  }

  pwm_dumpregs(priv, "After starting");

  return OK;
}

/****************************************************************************
 * Name: pwm_update_duty
 *
 * Description:
 *   Try to change only channel duty.
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_update_duty(FAR struct mpfs_pwmtimer_s *priv,
				    uint8_t channel, ub16_t duty16)
{
  uint32_t              period;
  uint32_t              neg_edge;
  ub32_t                duty = ub16toub32(duty16);

  DEBUGASSERT(priv != NULL);

  if (channel == 0 || channel > priv->nchannels)
  {
    pwmerr("ERROR: PWM%d has no such channel: %u\n", priv->pwmid, channel);
    return -EINVAL;
  }

  pwminfo("PWM%u channel: %u duty: %08x\n", priv->pwmid, channel, duty16);

  period = pwm_getreg(priv, MPFS_COREPWM_PERIOD_OFFSET);
  neg_edge = b32toi(duty * period + b32HALF);

  pwminfo("duty: %u, period: %u neg_edge: %u\n", duty16, period, neg_edge);

  /* Set the channels duty cycle by writing to the NEG_EDGE register
    * for this channel
    */
  const uint8_t neg_edge_reg_offset =
    MPFS_COREPWM_PWM1_NEG_EDGE_OFFSET +
    (MPFS_COREPWM_PWM2_NEG_EDGE_OFFSET -
     MPFS_COREPWM_PWM1_NEG_EDGE_OFFSET) * (channel - 1);

  pwm_putreg(priv, neg_edge_reg_offset, neg_edge);

  return OK;
}


/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 * Note:
 *   On a MPFS CorePWM block no setting up is needed
 *
 ****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct mpfs_pwmtimer_s *priv = (FAR struct mpfs_pwmtimer_s *)dev;

  pwminfo("PWMID%u\n", priv->pwmid);
  pwm_dumpregs(priv, "Initially");

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct mpfs_pwmtimer_s *priv = (FAR struct mpfs_pwmtimer_s *)dev;

  pwminfo("PWM%u\n", priv->pwmid);

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  int ret = OK;
  FAR struct mpfs_pwmtimer_s *priv = (FAR struct mpfs_pwmtimer_s *)dev;

  /* if frequency has not changed we just update duty */
  if (info->frequency == priv->frequency)
  {
    int i;

    for (i = 0; ret == OK && i < MPFS_MAX_PWM_CHANNELS; i++)
    {
      /* Set output if channel configured */

      if (info->channels[i].channel != 0)
      {
        ret = pwm_update_duty(priv, info->channels[i].channel,
                              info->channels[i].duty);
      }
    }
  }
  else
  {
    ret = pwm_timer(priv, info);

    /* Save current frequency */

    if (ret == OK)
      {
        priv->frequency = info->frequency;
      }
  }

  return ret;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.
 *
 ****************************************************************************/

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct mpfs_pwmtimer_s *priv = (FAR struct mpfs_pwmtimer_s *)dev;

  pwminfo("PWM%u\n", priv->pwmid);

  /* Determine which timer to reset */

  switch (priv->pwmid)
  {
#ifdef CONFIG_MPFS_COREPWM0
    case 1:
      break;
#endif
#ifdef CONFIG_MPFS_COREPWM1
    case 2:
      break;
#endif
    default:
      return -EINVAL;
  }

  /* Stopped so frequency is zero */

  priv->frequency = 0;

  /* No resetting on CorePWM block so just stop the output and
   * it into a state where pwm_start() can be called.
   */

  pwm_putreg(priv, MPFS_COREPWM_PWM_ENABLE_0_7_OFFSET,  0x00);
  pwm_putreg(priv, MPFS_COREPWM_PWM_ENABLE_8_15_OFFSET, 0x00);

  pwm_dumpregs(priv, "After stop");

  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  FAR struct mpfs_pwmtimer_s *priv = (FAR struct mpfs_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("PWM%u\n", priv->pwmid);
#endif

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_corepwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   pwmid - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the FPGA design.
 *
 * Returned Value:
 *   On success, a pointer to the mpfs lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *mpfs_corepwminitialize(int pwmid)
{
  FAR struct mpfs_pwmtimer_s *lower;

  pwminfo("PWM%u\n", pwmid);

  switch (pwmid)
  {
#ifdef CONFIG_MPFS_COREPWM0
    case 0:
      lower = &g_pwm1dev;
      break;
#endif
#ifdef CONFIG_MPFS_COREPWM1
    case 1:
      lower = &g_pwm2dev;
      break;
#endif
    default:
      pwmerr("ERROR: No such timer configured\n");
      return NULL;
  }

  return (FAR struct pwm_lowerhalf_s *)lower;
}

