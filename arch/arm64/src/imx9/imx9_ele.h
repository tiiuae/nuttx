/****************************************************************************
 * arch/arm64/src/imx9/imx9_ele.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_S3MUA_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_S3MUA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "chip.h"
#include "hardware/imx9_mua.h"

/****************************************************************************
 * Public Functions Prototypes
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

struct imx9_s3muadev_s; /* Forward reference */

#define S3MUA_MSG_SIZE  64
#define S3MUA_RESP_SIZE  31
#define INIT_FW_REQ      0x17
#define PING_FW_REQ      0x1
#define ELE_START_RNG_REQ 0xA3
#define ELE_GENERATE_KEY_BLOB_REQ 0xAF
#define ELE_LOAD_KEY_BLOB_REQ 0xA7
#define ELE_DERIVE_KEY_REQ 0xA9
#define ELE_GET_FW_STATUS_REQ 0xC5
#define ELE_GET_INFO_REQ 0xDA
#define GET_RANDOM_REQ	0xCD
#define MESSAGE_VER_6     0x6
#define MESSAGE_VER_7     0x7
#define TX_TAG          0x17
#define RX_TAG          0x1e
#define ELE_OK          0xd6
#define RNG_OK          0x2

/* Keyblob algorithms */
 #define AES_CBC  0x03
 #define AES_CTR  0x04
 #define AES_XTS  0x37
 #define SM4_CBC  0x2B
 #define KEYBLOB_TAG  0x81
 #define KEYBLOB_VERSION  0x00



#define S3MUA_TX        IMX9_MUA_TR_OFFSET
#define S3MUA_RX        IMX9_MUA_RR_OFFSET





/****************************************************************************
 * Name: imx9_ele_initialize
 *
 * Description:
 *   Initialize s3mua interface
 *
 * Input Parameters:
 *   -
 *
 * Returned Value:
 *   0 on success; a error value on failure
 *
 ****************************************************************************/


int imx9_ele_initialize(void);




#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_LPSPI_H */
