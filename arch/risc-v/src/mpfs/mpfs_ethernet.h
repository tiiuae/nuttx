/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_ethernet.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_ETHERNET_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/mpfs_ethernet.h"

#define MSS_MAC0_BASE     (MPFS_GEM0_LO_BASE)
#define MSS_EMAC0_BASE    (MPFS_GEM0_LO_BASE + 0x1000)
#define MSS_MAC1_BASE     (MPFS_GEM1_LO_BASE)
#define MSS_EMAC1_BASE    (MPFS_GEM1_LO_BASE + 0x1000)

#define MSS_MAC_QUEUE_COUNT (4)

#define MSS_MAC_TX_RING_SIZE (16U)
#define MSS_MAC_RX_RING_SIZE (16U)

//#define CONFIG_MPFS_ETH_USE_JUMBO_FRAMES

/* Maximum packet size that the hardware supports */
#define MSS_MAC_JUMBO_MAX (10240U)

#define MSS_MAC_DEFAULT_PACKET_SIZE (1518U)

#ifdef CONFIG_MPFS_ETH_USE_JUMBO_FRAMES
  #define MSS_MAC_MAX_PACKET_SIZE MSS_MAC_JUMBO_MAX
#else
  #define MSS_MAC_MAX_PACKET_SIZE MSS_MAC_DEFAULT_PACKET_SIZE
#endif


#define MSS_MAC_RX_BUF_VALUE    ((MSS_MAC_MAX_PACKET_SIZE + 63U) / 64U)

#define MSS_MAC_AMBA_BURST_256  (0U)
#define MSS_MAC_AMBA_BURST_1    (1U)
#define MSS_MAC_AMBA_BURST_4    (4U)
#define MSS_MAC_AMBA_BURST_8    (8U)
#define MSS_MAC_AMBA_BURST_16   (16U)
#define MSS_MAC_AMBA_BURST_MASK (31U)

#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_ETHERNET_H */