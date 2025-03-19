/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_rcc.c
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

#include <nuttx/spinlock.h>

#include <errno.h>
#include <sys/types.h>

#include "mpfs_rcc.h"
#include "mpfs_memorymap.h"

#include "hardware/mpfs_fpga_sysreg.h"

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uintptr_t g_reset_reg[] =
{
  [MPFS_RCC_CAN]  = MPFS_FPGA_SYSREG_CAN,
  [MPFS_RCC_I2C]  = MPFS_FPGA_SYSREG_I2C,
  [MPFS_RCC_SPI]  = MPFS_FPGA_SYSREG_SPI,
  [MPFS_RCC_PWM]  = MPFS_FPGA_SYSREG_PWM,
  [MPFS_RCC_UART] = MPFS_FPGA_SYSREG_UART,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_set_reset
 *
 * Description:
 *   Enable / disable peripheral reset.
 *
 * Input Parameters:
 *   rcc_id   - Device id.
 *   instance - Optional instance number for device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int mpfs_set_reset(int rcc_id, int instance, bool state)
{
  uintptr_t reg;
  UNUSED(instance); /* Unused, for now */

  if (rcc_id >= MPFS_RCC_LAST)
    {
      return -ENODEV;
    }

  reg = g_reset_reg[rcc_id];
  if (reg == 0)
    {
      return -ENODEV;
    }

  if (state)
    {
      modifyreg32(reg, 0, MPFS_FPGA_SYSREG_SOFT_RESET_CR(instance));
    }
  else
    {
      modifyreg32(reg, MPFS_FPGA_SYSREG_SOFT_RESET_CR(instance), 0);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_set_clock
 *
 * Description:
 *   Enable / disable peripheral clock.
 *
 * Input Parameters:
 *   rcc_id   - Device id.
 *   instance - Optional instance number for device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int mpfs_set_clock(int rcc_id, int instance, bool state)
{
  return -ENODEV;
}
