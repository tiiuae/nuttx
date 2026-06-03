/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_ticks2time
 ****************************************************************************/

void clock_ticks2time(FAR struct timespec *ts, clock_t tick)
{
  ts->tv_sec = div_const(tick, TICK_PER_SEC);
  tick -= ts->tv_sec * TICK_PER_SEC;
  ts->tv_nsec = tick * NSEC_PER_TICK;
}

/****************************************************************************
 * Name:  clock_time2ticks
 ****************************************************************************/

clock_t clock_time2ticks(FAR const struct timespec *ts)
{
  return ts->tv_sec * TICK_PER_SEC +
         div_const_roundup(ts->tv_nsec, NSEC_PER_TICK);
}

/****************************************************************************
 * Name:  clock_time2ticks_floor
 ****************************************************************************/

clock_t clock_time2ticks_floor(FAR const struct timespec *ts)
{
  return ts->tv_sec * TICK_PER_SEC + div_const(ts->tv_nsec, NSEC_PER_TICK);
}

/****************************************************************************
 * Name:  clock_msec2time
 ****************************************************************************/

void clock_msec2time(FAR struct timespec *ts, int64_t msec)
{
  ts->tv_sec = div_const(msec, MSEC_PER_SEC);
  msec -= ts->tv_sec * MSEC_PER_SEC;
  ts->tv_nsec = msec * NSEC_PER_MSEC;
}

/****************************************************************************
 * Name:  clock_time2msec
 ****************************************************************************/

int64_t clock_time2msec(FAR const struct timespec *ts)
{
  return ts->tv_sec * MSEC_PER_SEC +
         div_const(ts->tv_nsec, NSEC_PER_MSEC);
}

/****************************************************************************
 * Name:  clock_usec2time
 ****************************************************************************/

void clock_usec2time(FAR struct timespec *ts, int64_t usec)
{
  ts->tv_sec = div_const(usec, USEC_PER_SEC);
  usec -= ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec = usec * NSEC_PER_USEC;
}

/****************************************************************************
 * Name:  clock_time2usec
 ****************************************************************************/

int64_t clock_time2usec(FAR const struct timespec *ts)
{
  return ts->tv_sec * USEC_PER_SEC +
         div_const(ts->tv_nsec, NSEC_PER_USEC);
}

/****************************************************************************
 * Name:  clock_nsec2time
 ****************************************************************************/

void clock_nsec2time(FAR struct timespec *ts, int64_t nsec)
{
  ts->tv_sec = div_const(nsec, NSEC_PER_SEC);
  nsec -= ts->tv_sec * NSEC_PER_SEC;
  ts->tv_nsec = nsec;
}

/****************************************************************************
 * Name:  clock_time2nsec
 ****************************************************************************/

int64_t clock_time2nsec(FAR const struct timespec *ts)
{
  return ts->tv_sec * NSEC_PER_SEC + ts->tv_nsec;
}
