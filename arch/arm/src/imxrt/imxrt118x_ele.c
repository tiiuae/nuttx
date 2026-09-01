/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_ele.c
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
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "hardware/rt118x/imxrt118x_memorymap.h"
#include "imxrt118x_ele.h"
#include "imxrt118x_trdc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ele_msg g_msg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt118x_ele_sendmsg
 *
 * Description:
 *   Send a message to the EdgeLock Enclave over the System 3 Message Unit A.
 *
 ****************************************************************************/

static void imxrt118x_ele_sendmsg(struct ele_msg *msg_ptr)
{
  /* Check that ele is ready to receive */

  while (!((1) & getreg32(ELE_MU_TSR)));

  /* write header to slot 0 */

  putreg32(msg_ptr->header.data, ELE_MU_TR(0));

  /* write data */

  for (int i = 1; i < msg_ptr->header.size; i++)
    {
      int tx_channel;

      tx_channel = i % ELE_TR_NUM;
      while (!((1 << tx_channel) & getreg32(ELE_MU_TSR)));

      /* Write data */

      putreg32(msg_ptr->data[i - 1], ELE_MU_TR(tx_channel));
    }
}

/****************************************************************************
 * Name: imxrt118x_ele_receivemsg
 *
 * Description:
 *   Receive a response message from the EdgeLock Enclave.
 *
 ****************************************************************************/

static void imxrt118x_ele_receivemsg(struct ele_msg *msg_ptr)
{
  /* Check if data ready */

  while (!((1) & getreg32(ELE_MU_RSR)));

  /* Read Header from slot 0 */

  msg_ptr->header.data = getreg32(ELE_MU_RR(0));

  for (int i = 1; i < msg_ptr->header.size; i++)
    {
      /* Check if empty */

      int rx_channel = (i) % ELE_RR_NUM;
      while (!((1 << rx_channel) & getreg32(ELE_MU_RSR)));

      /* Read data */

      msg_ptr->data[i - 1] = getreg32(ELE_MU_RR(rx_channel));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imxrt118x_ele_init(void)
{
  int ret;

  /* Bring the S3MUA message unit up (clear the tx/rx control regs so we
   * don't accidentally fire an ELE-side interrupt before we're ready).
   */

  putreg32(0, ELE_MU_TCR);
  putreg32(0, ELE_MU_RCR);

  /* Load the ELE firmware.  The RT118x ROM only ships a minimal ELE stub
   * that understands LOAD_FW; the other commands below require the real
   * firmware to be running.  The FW blob is embedded in the driver image
   * (imxrt118x_ele_fw.c / CONFIG_IMXRT_ELE_FW_PATH).
   */

  ret = imxrt118x_ele_load_fw((uint32_t)(uintptr_t)imxrt118x_ele_fw);
  if (ret < 0)
    {
      _err("ELE: LOAD_FW failed (%d) - aborting handshake\n", ret);
      return;
    }

  /* Hand TRDC AON, MEGA, WAKEUP ownership to this core (CM33) so we can
   * program permissive access-control words.  Mirrors NXP MCUXpresso SDK
   * BOARD_RequestTRDC() for RT1180.  AON first, then MEGA before WAKEUP:
   * releasing WAKEUP first would drop the caller's own access to the
   * MEGA release path.
   */

  ret = imxrt118x_ele_release_rdc(ELE_TRDC_AON_ID, ELE_CORE_CM33_ID);
  if (ret < 0)
    {
      _err("ELE: RELEASE_RDC AON failed (%d)\n", ret);
    }

  ret = imxrt118x_ele_release_rdc(ELE_TRDC_MEGA_ID, ELE_CORE_CM33_ID);
  if (ret < 0)
    {
      _err("ELE: RELEASE_RDC MEGA failed (%d)\n", ret);
    }

  ret = imxrt118x_ele_release_rdc(ELE_TRDC_WAKEUP_ID, ELE_CORE_CM33_ID);
  if (ret < 0)
    {
      _err("ELE: RELEASE_RDC WAKEUP failed (%d)\n", ret);
    }

#ifdef CONFIG_IMXRT_TRDC
  /* Grant every domain full R/W/X in all TRDCs so peripheral accesses
   * from the M7 / eDMA / etc. don't take TRDC bus faults.
   */

  imxrt118x_trdc_grant_full_access(IMXRT_TRDC1_BASE);
  imxrt118x_trdc_grant_full_access(IMXRT_TRDC2_BASE);
  imxrt118x_trdc_grant_full_access(IMXRT_TRDC3_BASE);
#endif
}

int imxrt118x_ele_load_fw(uint32_t fw_addr)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 4;
  g_msg.header.command = ELE_LOAD_FW_REQ;
  g_msg.data[0] = fw_addr;
  g_msg.data[1] = 0;
  g_msg.data[2] = fw_addr;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_release_rdc(uint32_t rdc_id, uint32_t core_id)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 2;
  g_msg.header.command = ELE_RELEASE_RDC_REQ;
  g_msg.data[0] = (rdc_id << 8) | (core_id & 0xff);

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

int imxrt118x_ele_enable_apc(void)
{
  g_msg.header.version = ELE_VERSION;
  g_msg.header.tag = ELE_CMD_TAG;
  g_msg.header.size = 1;
  g_msg.header.command = ELE_ENABLE_APC_REQ;

  imxrt118x_ele_sendmsg(&g_msg);
  imxrt118x_ele_receivemsg(&g_msg);

  if ((g_msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}
