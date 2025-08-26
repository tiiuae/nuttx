/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_i2c_ext.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_I2C_EXT_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_I2C_EXT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C extension IP registers */

#define MPFS_COREI2C_EXT_CTR_OFFSET     0x20
#define MPFS_COREI2C_EXT_STA_OFFSET     0x24
#define MPFS_COREI2C_EXT_DAT_OFFSET     0x28
#define MPFS_COREI2C_EXT_ADR_OFFSET     0x2c
#define MPFS_COREI2C_EXT_BTT_OFFSET     0x30
#define MPFS_COREI2C_EXT_CNF_OFFSET     0x34
#define MPFS_COREI2C_EXT_TMR_OFFSET     0x38
#define MPFS_COREI2C_EXT_IRQ_OFFSET     0x3c

/* I2C extension IP register bits */

#define MPFS_COREI2C_EXT_CTR_ENABLE     (1 << 0)
#define MPFS_COREI2C_EXT_CTR_START      (1 << 1)
#define MPFS_COREI2C_EXT_CTR_STOP       (1 << 2)

#define MPFS_COREI2C_EXT_STA_ENABLE     (1 << 0)
#define MPFS_COREI2C_EXT_STA_ACTIVE     (1 << 1)
#define MPFS_COREI2C_EXT_STA_INT        (1 << 2)
#define MPFS_COREI2C_EXT_STA_TXCOUNT(x) (((x) >> 8)  & 0xff)
#define MPFS_COREI2C_EXT_STA_RXCOUNT(x) (((x) >> 16) & 0xff)

#define MPFS_COREI2C_EXT_BTT_TX(x)      ((x) & 0x3f)
#define MPFS_COREI2C_EXT_BTT_RX(x)      (((x) & 0x3f) << 8)

#define MPFS_COREI2C_EXT_CNF_M_READ     (1 << 0)
#define MPFS_COREI2C_EXT_CNF_M_TEN      (1 << 1)
#define MPFS_COREI2C_EXT_CNF_M_CONT     (1 << 5)
#define MPFS_COREI2C_EXT_CNF_M_NOSTOP   (1 << 6)
#define MPFS_COREI2C_EXT_CNF_M_NOSTART  (1 << 7)
#define MPFS_COREI2C_EXT_CNF_IRQ_ENABLE (1 << 8)
#define MPFS_COREI2C_EXT_CNF_IRQ_MASK   (1 << 9)

#define MPFS_COREI2C_EXT_IRQ_COMPLETE   (1 << 0)
#define MPFS_COREI2C_EXT_IRQ_ERROR      (1 << 1)
#define MPFS_COREI2C_EXT_IRQ_BUS_ERROR  (1 << 2)
#define MPFS_COREI2C_EXT_IRQ_TX_OVF     (1 << 3)
#define MPFS_COREI2C_EXT_IRQ_TX_UDR     (1 << 4)
#define MPFS_COREI2C_EXT_IRQ_RX_OVF     (1 << 5)
#define MPFS_COREI2C_EXT_IRQ_RX_UDR     (1 << 6)
#define MPFS_COREI2C_EXT_IRQ_STATUS(x)  (((x) >> 8) & 0xff)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_I2C_EXT_H */
