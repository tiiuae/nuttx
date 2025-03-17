/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_fpga_sysreg.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_SYSREG_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_SYSREG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_FPGA_SYSREG_BASE 0x42000000

/* 0x10 : 32 bit Reset 1 control registers ->
 *         0x00 - I2C reset
 *         0x04 - SPI reset
 *         0x08 - UART reset
 *         0x0C - PWM reset
 *  0x20 : 32 bit Reset 2 control registers ->
 *         0x00 - CAN reset
 */

#define MPFS_FPGA_SYSREG_I2C    (MPFS_FPGA_SYSREG_BASE + 0x10)
#define MPFS_FPGA_SYSREG_SPI    (MPFS_FPGA_SYSREG_BASE + 0x14)
#define MPFS_FPGA_SYSREG_UART   (MPFS_FPGA_SYSREG_BASE + 0x18)
#define MPFS_FPGA_SYSREG_PWM    (MPFS_FPGA_SYSREG_BASE + 0x1C)
#define MPFS_FPGA_SYSREG_CAN    (MPFS_FPGA_SYSREG_BASE + 0x20)

/* Soft reset control register per instance. Applies to all peripherals. */

#define MPFS_FPGA_SYSREG_SOFT_RESET_CR(x) (1 << (x))

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_SYSREG_H */
