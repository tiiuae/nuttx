/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_can.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_CAN_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Check if MSS CAN support is enabled. */

#ifdef CONFIG_MPFS_HAVE_MSS_CAN

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include "hardware/mpfs_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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

/****************************************************************************
 * Name: mpfs_fpga_canfd_init
 *
 * Description:
 *    The mpfs_can_init() function initializes the CAN driver as well as the
 *    CAN controller. The basic_can_rx_mb and basic_can_tx_mb are used to
 *    configure the number of receive and transmit buffers in basic CAN
 *    operation. This function configures the CAN channel speed as per the
 *    “bitrate” parameter. It initializes all receive buffers and make it
 *    ready for configuration. This is the first function to be called before
 *    using any other function.
 *
 * Input Parameters:
 *    ncan  - The ncan parameter is a number identifying the CAN block.
 *    bitrate  - The bitRate parameter is used to configure CAN speed.
 *    The following standard preset definitions are provided for systems with
 *    a PCLK1 of 8MHz, 16MHz or 32MHz:
 *       +-------------------+--------------------+--------------------+
 *       | 8MHz PCLK1        | 16MHz PCLK1        | 32MHz PCLK1        |
 *       +-------------------+--------------------+--------------------+
 *       | CAN_SPEED_8M_5K   | CAN_SPEED_16M_5K   | CAN_SPEED_32M_5K   |
 *       | CAN_SPEED_8M_10K  | CAN_SPEED_16M_10K  | CAN_SPEED_32M_10K  |
 *       | CAN_SPEED_8M_20K  | CAN_SPEED_16M_20K  | CAN_SPEED_32M_20K  |
 *       | CAN_SPEED_8M_50K  | CAN_SPEED_16M_50K  | CAN_SPEED_32M_50K  |
 *       | CAN_SPEED_8M_100K | CAN_SPEED_16M_100K | CAN_SPEED_32M_100K |
 *       | CAN_SPEED_8M_125K | CAN_SPEED_16M_125K | CAN_SPEED_32M_125K |
 *       | CAN_SPEED_8M_250K | CAN_SPEED_16M_250K | CAN_SPEED_32M_250K |
 *       | CAN_SPEED_8M_500K | CAN_SPEED_16M_500K | CAN_SPEED_32M_500K |
 *       | CAN_SPEED_8M_1M   | CAN_SPEED_16M_1M   | CAN_SPEED_32M_1M   |
 *       +-------------------+--------------------+--------------------+
 *
 * Returned Value:
 *    This function returns CAN_OK on successful execution, otherwise it will
 *    returns CAN_ERR indicating error condition.
 *
 ****************************************************************************/

int mpfs_can_init(int ncan, uint32_t bitrate);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_MPFS_HAVE_MSS_CAN */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_CAN_H */
