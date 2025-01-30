/****************************************************************************
 * arch/arm64/src/imx9/imx9_pmic.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_PMIC_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_PMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>
#include "arm64_internal.h"
#include "hardware/imx9_memorymap.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define IMX9_PMIC_RESET_CTRL_DEFAULT                0x21

/* WDOG_B_CFG [7:6]: 11b (Cold Reset) */

#define IMX9_PMIC_RESET_CTRL_WDOG_COLD_RESET_MASK   0xC0

/****************************************************************************
 * Name: imx9_pmic_get_reset_reason
 *
 * Description:
 *  Read reset reason from pmic via i2c
 *
 * Input Parameters:
 *   Pointer to reset reason value
 *
 * Returned Value:
 *   Zero on success or error code
 *
 ****************************************************************************/

int imx9_pmic_get_reset_reason(uint8_t *value);

/****************************************************************************
 * Name: imx9_pmic_reset
 *
 * Description:
 *  Reset entire Soc
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success or error code
 *
 ****************************************************************************/

int imx9_pmic_reset(void);

/****************************************************************************
 * Name: imx9_pmic_get_reset_ctrl
 *
 * Description:
 *  Read reset control register value
 *
 * Input Parameters:
 *   Pointer to reset reason value
 *
 * Returned Value:
 *   Zero on success or error code
 *
 ****************************************************************************/

int imx9_pmic_get_reset_ctrl(uint8_t *value);

/****************************************************************************
 * Name: imx9_pmic_set_reset_ctrl
 *
 * Description:
 *  Set reset control register value
 *
 * Input Parameters:
 *   Register value
 *
 * Returned Value:
 *   Zero on success or error code
 *
 ****************************************************************************/

int imx9_pmic_set_reset_ctrl(uint8_t val);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_PMIC_H */
