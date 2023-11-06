/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_wdog.c
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

/*****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

const void* mpfs_base_addresses = [
  MPFS_WDOG0_LO_BASE,
  MPFS_WDOG1_LO_BASE,
  MPFS_WDOG2_LO_BASE,
  MPFS_WDOG3_LO_BASE,
  MPFS_WDOG4_LO_BASE,
];


void* mpfs_wdog_get_base_address(unsigned int instance)
{

  void* addr;

  if (instance >= MPFS_WDOG_NUM_INSTANCES)
    {
      return -EINVAL;
    }

  switch(instance)
  {
    case 0:
      addr = MPFS_WDOG0_LO_BASE;
      break;

  }
}

/****************************************************************************
 * Name:  mpfs_wdog_start
 *
 * Description:
 *   Start watchdog. Once started, the watchdog can not be stopped.
 *
 * Input Parameters:
 *   instance - wdog instance index (0..4)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_wdog_start(unsigned int instance)
{
  return mpfs_wdog_keepalive(instance);
}

/****************************************************************************
 * Name:  mpfs_wdog_getcounter
 *
 * Description:
 *   Get current watchdog counter value.
 *
 * Input Parameters:
 *   instance - wdog instance index (0..4)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_wdog_getcounter(unsigned int instance)
{
  return getreg32(mpfs_base_addresses[instance] + MPFS_WDOG_REFRESH_OFFSET);
}

/****************************************************************************
 * Name:  mpfs_wdog_settrigger
 *
 * Description:
 *   Set counter threshold for trigger WDOG timeout and NMI interrupt. After
 *   this level the counter refresh is no longer allowed.
 *
 * Input Parameters:
 *   instance - wdog instance index (0..4)
 *   value    - trigger value
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_wdog_settrigger(unsigned int instance, unsigned int value)
{
  if (instance >= MPFS_WDOG_NUM_INSTANCES)
    {
      return -EINVAL;
    }

  putreg32(value, mpfs_base_addresses[instance] + MPFS_WDOG_TRIGGER_OFFSET);
  return Ok;
}

/****************************************************************************
 * Name:  mpfs_wdog_setmsvp
 *
 * Description:
 *   Set counter threshold for MVRP (Maximum Value up to which Refresh is
 *   Permitted). Counter refresh is not allowed before this level is reached.
 *
 * Input Parameters:
 *   instance - wdog instance index (0..4)
 *   value    - MVRP value
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_wdog_setmsvp(unsigned int instance, unsigned int value)
{
  if (instance >= MPFS_WDOG_NUM_INSTANCES)
    {
      return -EINVAL;
    }

  putreg32(value, mpfs_base_addresses[instance] + MPFS_WDOG_MSVP_OFFSET);
  return Ok;
}

/****************************************************************************
 * Name:  mpfs_wdog_settimeout
 *
 * Description:
 *   Set watchdog time load value. This is the level where counter starts to
 *   countdown after WDOG is started or refreshed.
 *
 * Input Parameters:
 *   instance - wdog instance index (0..4)
 *   value    - counter load value
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_wdog_settimeout(unsigned int instance, unsigned int value)
{
  if (instance >= MPFS_WDOG_NUM_INSTANCES)
    {
      return -EINVAL;
    }

  putreg32(value, mpfs_base_addresses[instance] + MPFS_WDOG_TIME_OFFSET);
  return Ok;
}

/****************************************************************************
 * Name:  mpfs_wdog_keepalive
 *
 * Description:
 *   Refresh watchdog timer.
 *
 * Input Parameters:
 *   instance - wdog instance index (0..4)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_wdog_keepalive(unsigned int instance)
{
  if (instance >= MPFS_WDOG_NUM_INSTANCES)
    {
      return -EINVAL;
    }

  putreg32(WDOG_REFRESH_RELOAD, mpfs_base_addresses[instance] + MPFS_WDOG_REFRESH_OFFSET);
  return Ok;
}

/****************************************************************************
 * Name:  mpfs_wdog_immediate_reset
 *
 * Description:
 *   Trigger immediate system reset by WDOG.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_wdog_immediate_reset(void)
{
  putreg32(WDOG_FORCE_IMMEDIATE_RESET, mpfs_base_addresses[0] + MPFS_WDOG_FORCE_OFFSET);

  /* Wait for the reset */

  for (; ; );
}

