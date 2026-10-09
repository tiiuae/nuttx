/****************************************************************************
 * arch/arm/src/stm32n6/stm32_pwr.c
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

#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "stm32_pwr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables write access to the backup domain (RTC registers, RTC backup
 *   data registers and backup SRAM).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool stm32_pwr_enablebkp(bool writable)
{
  uint32_t regval;
  bool waswritable;

  regval = getreg32(STM32_PWR_DBPCR);
  waswritable = ((regval & PWR_DBPCR_DBP) != 0);

  if (writable)
    {
      regval |= PWR_DBPCR_DBP;
    }
  else
    {
      regval &= ~PWR_DBPCR_DBP;
    }

  putreg32(regval, STM32_PWR_DBPCR);

  return waswritable;
}

/****************************************************************************
 * Name: stm32_pwr_configvddio
 *
 * Description:
 *   Configure I/O supply-valid and voltage-range bits in PWR SVMCR3.
 *
 * Input Parameters:
 *   setmask   - PWR_SVMCR3 bits to set.
 *   clearmask - PWR_SVMCR3 bits to clear.
 *
 ****************************************************************************/

void stm32_pwr_configvddio(uint32_t setmask, uint32_t clearmask)
{
  modifyreg32(STM32_PWR_SVMCR3, clearmask, setmask);
}
