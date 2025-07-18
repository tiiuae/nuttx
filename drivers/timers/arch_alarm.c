/****************************************************************************
 * drivers/timers/arch_alarm.c
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
#include <nuttx/clock.h>
#include <nuttx/timers/arch_alarm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct oneshot_lowerhalf_s *g_oneshot_lower;

#ifndef CONFIG_SCHED_TICKLESS
static clock_t g_current_tick;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void udelay_coarse(useconds_t microseconds)
{
  volatile int i;

  /* We'll do this a little at a time because we expect that the
   * CONFIG_BOARD_LOOPSPERUSEC is very inaccurate during to truncation in
   * the divisions of its calculation.  We'll use the largest values that
   * we can in order to prevent significant error buildup in the loops.
   */

  while (microseconds > 1000)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERMSEC; i++)
        {
        }

      microseconds -= 1000;
    }

  while (microseconds > 100)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER100USEC; i++)
        {
        }

      microseconds -= 100;
    }

  while (microseconds > 10)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER10USEC; i++)
        {
        }

      microseconds -= 10;
    }

  while (microseconds > 0)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERUSEC; i++)
        {
        }

      microseconds--;
    }
}

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                             FAR void *arg)
{
  clock_t now = 0;

  ONESHOT_TICK_CURRENT(g_oneshot_lower, &now);
#ifdef CONFIG_SCHED_TICKLESS
  nxsched_alarm_tick_expiration(now);
#else
  /* Start the next tick first, in order to minimize latency. Ideally
   * the ONESHOT_TICK_START would also return the current tick so that
   * the retriving the current tick and starting the new one could be done
   * atomically w. respect to a HW timer
   */

  ONESHOT_TICK_START(g_oneshot_lower, oneshot_callback, NULL, 1);

  /* It is always an error if this progresses more than 1 tick at a time.
   * That would break any timer based on wdog; such timers might timeout
   * early. Add a DEBUGASSERT here to catch those errors. It is not added
   * here by default, since it would break debugging. These errors
   * would occur due to HW timers possibly running while CPU is being halted.
   */

  /* DEBUGASSERT(now - g_current_tick <= 1); */

  while (now - g_current_tick > 0)
    {
      g_current_tick++;
      nxsched_process_timer();
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_alarm_set_lowerhalf(FAR struct oneshot_lowerhalf_s *lower)
{
#ifdef CONFIG_SCHED_TICKLESS
  clock_t ticks = 0;
#endif

  g_oneshot_lower = lower;

#ifdef CONFIG_SCHED_TICKLESS
  ONESHOT_TICK_MAX_DELAY(g_oneshot_lower, &ticks);
  g_oneshot_maxticks = ticks < UINT32_MAX ? ticks : UINT32_MAX;
#else
  ONESHOT_TICK_CURRENT(g_oneshot_lower, &g_current_tick);
  ONESHOT_TICK_START(g_oneshot_lower, oneshot_callback, NULL, 1);
#endif
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   the architecture-specific timer was initialized).  This function is
 *   functionally equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

void weak_function up_timer_getmask(FAR clock_t *mask)
{
  *mask = 0;

  if (g_oneshot_lower != NULL)
    {
      clock_t maxticks = 0;

      ONESHOT_TICK_MAX_DELAY(g_oneshot_lower, &maxticks);

      for (; ; )
        {
          clock_t next = (*mask << 1) | 1;
          if (next > maxticks)
            {
              break;
            }

          *mask = next;
        }
    }
}

int weak_function up_timer_gettick(FAR clock_t *ticks)
{
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      ret = ONESHOT_TICK_CURRENT(g_oneshot_lower, ticks);
    }

  return ret;
}

int weak_function up_timer_gettime(struct timespec *ts)
{
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      ret = ONESHOT_CURRENT(g_oneshot_lower, ts);
    }

  return ret;
}

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_alarm_expiration() will not be called unless the alarm is
 *   restarted with up_alarm_start().
 *
 *   If, as a race condition, the alarm has already expired when this
 *   function is called, then time returned is the current time.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the expiration time.  The current time should
 *        returned if the alarm is not active.  ts may be NULL in which
 *        case the time is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_alarm_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int weak_function up_alarm_tick_cancel(FAR clock_t *ticks)
{
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      ret = ONESHOT_TICK_CANCEL(g_oneshot_lower, ticks);
      ONESHOT_TICK_CURRENT(g_oneshot_lower, ticks);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm.  nxsched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur. When the
 *        alarm occurs the timer logic will call nxsched_alarm_expiration().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int weak_function up_alarm_tick_start(clock_t ticks)
{
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      clock_t now = 0;
      clock_t delta;

      ONESHOT_TICK_CURRENT(g_oneshot_lower, &now);
      delta = ticks - now;
      if ((sclock_t)delta < 0)
        {
          delta = 0;
        }

      ret = ONESHOT_TICK_START(g_oneshot_lower, oneshot_callback,
                               NULL, delta);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_perf_*
 *
 * Description:
 *   The first interface simply provides the current time value in unknown
 *   units.  NOTE:  This function may be called early before the timer has
 *   been initialized.  In that event, the function should just return a
 *   start time of zero.
 *
 *   Nothing is assumed about the units of this time value.  The following
 *   are assumed, however: (1) The time is an unsigned integer value, (2)
 *   the time is monotonically increasing, and (3) the elapsed time (also
 *   in unknown units) can be obtained by subtracting a start time from
 *   the current time.
 *
 *   The second interface simple converts an elapsed time into well known
 *   units.
 ****************************************************************************/

#ifndef CONFIG_ARCH_HAVE_PERF_EVENTS
void up_perf_init(FAR void *arg)
{
  UNUSED(arg);
}

clock_t up_perf_gettime(void)
{
  clock_t ret = 0;

  if (g_oneshot_lower != NULL)
    {
      struct timespec ts;

      ONESHOT_CURRENT(g_oneshot_lower, &ts);
      ret = clock_time2nsec(&ts);
    }

  return ret;
}

unsigned long up_perf_getfreq(void)
{
  return NSEC_PER_SEC;
}

void up_perf_convert(clock_t elapsed, FAR struct timespec *ts)
{
  clock_nsec2time(ts, elapsed);
}
#endif /* CONFIG_ARCH_PERF_EVENTS */

/****************************************************************************
 * Name: up_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds.
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void weak_function up_mdelay(unsigned int milliseconds)
{
  up_udelay(USEC_PER_MSEC * milliseconds);
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void weak_function up_udelay(useconds_t microseconds)
{
  up_ndelay(NSEC_PER_USEC * microseconds);
}

/****************************************************************************
 * Name: up_ndelay
 *
 * Description:
 *   Delay inline for the requested number of nanoseconds.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void weak_function up_ndelay(unsigned long nanoseconds)
{
  udelay_coarse((nanoseconds + NSEC_PER_USEC - 1) / NSEC_PER_USEC);
}
