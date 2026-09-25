/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_mx25um51245g.c
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

#include <nuttx/mtd/mtd.h>

#include "arm_internal.h"
#include "stm32_xspi.h"
#include "hardware/stm32n6xxx_memorymap.h"
#include "hardware/stm32n6xxx_xspi.h"
#include "nucleo-n657x0-q.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MX25UM_OCTA_DTR_READ       0xee11
#define MX25UM_READ_DUMMY          20
#define MX25UM_DEVSIZE             25
#define MX25UM_SETUP_PRESCALER     3
#define MX25UM_XIP_FREQUENCY       100000000ul

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xspi_flash_initialize
 *
 * Description:
 *   Initialize XSPI2 and the board's MX25UM51245G flash, then enter
 *   memory-mapped octal DTR mode at 100 MHz with DQS.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_flash_initialize(void)
{
  struct stm32_xspi_config_s config;
  struct qspi_meminfo_s readinfo;
  struct mtd_dev_s *mtd;
  struct qspi_dev_s *qspi;
  volatile uint32_t probe;
  int ret;

  config.memtype      = STM32_XSPI_MEMTYPE_MACRONIX;
  config.iomport      = STM32_XSPI_IOM_PORT2;
  config.cs_override  = STM32_XSPI_CS_NCS1;
  config.devsize      = MX25UM_DEVSIZE;
  config.csht         = 2;
  config.prescaler    = MX25UM_SETUP_PRESCALER;
  config.fthres       = 4;
  config.wrapsize     = 0;
  config.maxtran      = 0;
  config.csbound      = 0;
  config.req2ack      = 1;
  config.refresh      = 0;
  config.clock_mode3  = false;
  config.free_running = false;
  config.sample_shift = false;
  config.data_strobe  = true;

  qspi = stm32_xspi_initialize(2, &config);
  if (qspi == NULL)
    {
      return -ENODEV;
    }

  mtd = mx25um51245g_initialize(qspi);
  if (mtd == NULL)
    {
      return -ENODEV;
    }

  if (QSPI_SETFREQUENCY(qspi, MX25UM_XIP_FREQUENCY) !=
      MX25UM_XIP_FREQUENCY)
    {
      return -ERANGE;
    }

  readinfo.flags   = QSPIMEM_IOCTAL | QSPIMEM_OCTALIO | QSPIMEM_DTR;
  readinfo.addrlen = 4;
  readinfo.dummies = MX25UM_READ_DUMMY;
  readinfo.cmd     = MX25UM_OCTA_DTR_READ;
  readinfo.buflen  = 0;
  readinfo.addr    = 0;
  readinfo.buffer  = NULL;

  ret = stm32_xspi_enter_memorymapped(qspi, &readinfo, NULL);
  if (ret < 0)
    {
      return ret;
    }

  /* The first mapped access starts the N6 high-speed-interface automatic
   * calibration requested by the final DCR2/CCR writes.  The access stalls
   * until calibration and the read have completed.
   */

  probe = getreg32(STM32_XSPI2_BANK);
  UNUSED(probe);

  if ((getreg32(STM32_XSPI2_CALFCR) & XSPI_CALFCR_CALMAX) != 0)
    {
      return -ERANGE;
    }

  return OK;
}
