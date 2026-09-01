/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig_ver3.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/rt118x/imxrt118x_ccm.h"
#include "imxrt_clockconfig_ver3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fixed clock source frequencies programmed by the Cortex-M33 boot ROM
 * before the M7 is released.  NuttX does not reprogram these PLLs, so
 * their frequencies are treated as static constants; if a future use
 * case needs to reconfigure the PLLs, extend imxrt_get_clock() to read
 * the ANADIG registers instead.
 */

#define IMXRT_OSC_24M_HZ         (24000000u)
#define IMXRT_OSC_RC_24M_HZ      (24000000u)
#define IMXRT_OSC_RC_400M_HZ     (400000000u)
#define IMXRT_ARM_PLL_HZ         (800000000u)
#define IMXRT_SYS_PLL1_HZ        (1000000000u)
#define IMXRT_SYS_PLL2_HZ        (528000000u)
#define IMXRT_SYS_PLL3_HZ        (480000000u)

/* Standard PFD fractions used by the boot ROM (IMXRT1180RM Ch. 20.5 default
 * configuration).  Output = (PLL * 18) / frac.
 */

#define IMXRT_SYS_PLL2_PFD0_HZ   (352000000u)  /* frac = 27 */
#define IMXRT_SYS_PLL2_PFD1_HZ   (594000000u)  /* frac = 16 */
#define IMXRT_SYS_PLL2_PFD2_HZ   (396000000u)  /* frac = 24 */
#define IMXRT_SYS_PLL2_PFD3_HZ   (297000000u)  /* frac = 32 */
#define IMXRT_SYS_PLL3_PFD0_HZ   (664615384u)  /* frac = 13 */
#define IMXRT_SYS_PLL3_PFD1_HZ   (508235294u)  /* frac = 17 */
#define IMXRT_SYS_PLL3_PFD2_HZ   (392727272u)  /* frac = 22 */
#define IMXRT_SYS_PLL3_PFD3_HZ   (392727272u)  /* frac = 22 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ccm_configure_root_clock
 ****************************************************************************/

int imxrt_ccm_configure_root_clock(int root, int src, uint32_t div)
{
  uint32_t regval;
  int i;

  if (root >= CCM_CR_COUNT || div == 0 || div > 256)
    {
      return -EINVAL;
    }

  /* Find the mux index that selects the requested source. */

  for (i = 0; i < ROOT_MUX_MAX; i++)
    {
      if (g_ccm_root_mux[root][i] == src)
        {
          break;
        }
    }

  if (i == ROOT_MUX_MAX)
    {
      return -EINVAL;
    }

  regval = getreg32(IMXRT_CCM_CR_CTRL(root));
  regval &= ~(CCM_CR_CTRL_MUX_MASK | CCM_CR_CTRL_DIV_MASK);
  regval |= CCM_CR_CTRL_MUX_SRCSEL(i) | CCM_CR_CTRL_DIV(div);
  putreg32(regval, IMXRT_CCM_CR_CTRL(root));

  while (getreg32(IMXRT_CCM_CR_STAT0(root)) & CCM_CR_STAT0_CHANGING);

  return OK;
}

/****************************************************************************
 * Name: imxrt_ccm_gate_on
 ****************************************************************************/

int imxrt_ccm_gate_on(int gate, bool enabled)
{
  uint32_t value;

  if (gate >= CCM_LPCG_COUNT)
    {
      return -EINVAL;
    }

  value = enabled ? CCM_LPCG_DIR_ON : 0u;
  putreg32(value, IMXRT_CCM_LPCG_DIR(gate));

  while ((getreg32(IMXRT_CCM_LPCG_STAT0(gate)) & CCM_LPCG_STAT0_ON) !=
         value);

  return OK;
}

/****************************************************************************
 * Name: imxrt_periphclk_configure
 *
 * Description:
 *   Thin wrapper that lets legacy callers pass CCM_LPCG_DIR_ON /
 *   CCM_LPCG_DIR_OFF as the second argument.
 *
 ****************************************************************************/

void imxrt_periphclk_configure(unsigned int index, unsigned int value)
{
  imxrt_ccm_gate_on((int)index, (value & CCM_LPCG_DIR_ON) != 0);
}

/****************************************************************************
 * Name: imxrt_get_clock
 ****************************************************************************/

int imxrt_get_clock(int clkname, uint32_t *frequency)
{
  switch (clkname)
    {
      case OSC_RC_24M:
        *frequency = IMXRT_OSC_RC_24M_HZ;
        break;

      case OSC_RC_400M:
        *frequency = IMXRT_OSC_RC_400M_HZ;
        break;

      case OSC_24M:
        *frequency = IMXRT_OSC_24M_HZ;
        break;

      case ARM_PLL_OUT:
        *frequency = IMXRT_ARM_PLL_HZ;
        break;

      case SYS_PLL1_OUT:
        *frequency = IMXRT_SYS_PLL1_HZ;
        break;

      case SYS_PLL1_DIV2:
        *frequency = IMXRT_SYS_PLL1_HZ / 2;
        break;

      case SYS_PLL1_DIV5:
        *frequency = IMXRT_SYS_PLL1_HZ / 5;
        break;

      case SYS_PLL2_OUT:
        *frequency = IMXRT_SYS_PLL2_HZ;
        break;

      case SYS_PLL2_PFD0:
        *frequency = IMXRT_SYS_PLL2_PFD0_HZ;
        break;

      case SYS_PLL2_PFD1:
        *frequency = IMXRT_SYS_PLL2_PFD1_HZ;
        break;

      case SYS_PLL2_PFD2:
        *frequency = IMXRT_SYS_PLL2_PFD2_HZ;
        break;

      case SYS_PLL2_PFD3:
        *frequency = IMXRT_SYS_PLL2_PFD3_HZ;
        break;

      case SYS_PLL3_OUT:
        *frequency = IMXRT_SYS_PLL3_HZ;
        break;

      case SYS_PLL3_DIV2:
        *frequency = IMXRT_SYS_PLL3_HZ / 2;
        break;

      case SYS_PLL3_PFD0:
        *frequency = IMXRT_SYS_PLL3_PFD0_HZ;
        break;

      case SYS_PLL3_PFD1:
        *frequency = IMXRT_SYS_PLL3_PFD1_HZ;
        break;

      case SYS_PLL3_PFD2:
        *frequency = IMXRT_SYS_PLL3_PFD2_HZ;
        break;

      case SYS_PLL3_PFD3:
        *frequency = IMXRT_SYS_PLL3_PFD3_HZ;
        break;

      case AUDIO_PLL_OUT:
        *frequency = 0;
        break;

      default:
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_get_rootclock
 ****************************************************************************/

int imxrt_get_rootclock(uint32_t clkroot, uint32_t *frequency)
{
  uint32_t reg;
  uint32_t mux;
  uint32_t div;
  int clkname;

  if (clkroot >= CCM_CR_COUNT)
    {
      return -ENODEV;
    }

  reg = getreg32(IMXRT_CCM_CR_CTRL(clkroot));

  if ((reg & CCM_CR_CTRL_OFF) != 0)
    {
      *frequency = 0;
      return OK;
    }

  mux     = (reg & CCM_CR_CTRL_MUX_MASK) >> CCM_CR_CTRL_MUX_SHIFT;
  clkname = g_ccm_root_mux[clkroot][mux];
  imxrt_get_clock(clkname, frequency);
  div     = ((reg & CCM_CR_CTRL_DIV_MASK) >> CCM_CR_CTRL_DIV_SHIFT) + 1;
  *frequency = *frequency / div;

  return OK;
}

/****************************************************************************
 * Name: imxrt_clockconfig
 *
 * Description:
 *   Called to initialize the i.MX RT.  This does whatever setup is needed to
 *   put the SoC in a usable state.  The Cortex-M33 boot ROM has already
 *   started essential PLLs and set the FlexSPI1 clock, so only the roots
 *   required by NuttX are configured here.  CCM_CR_FLEXSPI1 and its LPCG
 *   MUST NOT be touched while executing XIP from FlexSPI1.
 *
 ****************************************************************************/

void imxrt_clockconfig(void)
{
  /* AON bus (LPUART1/2 pclk): SYS_PLL2 (528 MHz) / 4 = 132 MHz */

  imxrt_ccm_configure_root_clock(CCM_CR_BUS_AON, SYS_PLL2_OUT, 4);

  /* M7 SysTick reference: 24 MHz / 240 = 100 kHz */

  imxrt_ccm_configure_root_clock(CCM_CR_M7_SYSTICK, OSC_24M, 240);

  /* LPUART1/2 functional clock: SYS_PLL3_DIV2 (240 MHz) / 10 = 24 MHz */

  imxrt_ccm_configure_root_clock(CCM_CR_LPUART0102, SYS_PLL3_DIV2, 10);

  /* Turn on the LPCGs used by the minimal port */

  imxrt_ccm_gate_on(CCM_LPCG_IOMUXC1, true);
  imxrt_ccm_gate_on(CCM_LPCG_IOMUXC2, true);
  imxrt_ccm_gate_on(CCM_LPCG_LPUART1, true);
}
