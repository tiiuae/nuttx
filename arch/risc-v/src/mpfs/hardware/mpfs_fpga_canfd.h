/****************************************************************************
 * arch/riscv/src/mpfs/hardware/mpfs_fpga_can.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_CAN_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_CAN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets ********************************************************/

#define MPFS_CANFD_DEVICE_ID_OFFSET                             (0x00)

#define MPFS_CANFD_MODE_REG_OFFSET                              (0x04)
#define MPFS_CANFD_MODE_OFFSET                                  (MPFS_CANFD_MODE_REG_OFFSET + 0x00)
#define MPFS_CANFD_COMMAND_OFFSET                               (MPFS_CANFD_MODE_REG_OFFSET + 0x01)
#define MPFS_CANFD_STATUS_OFFSET                                (MPFS_CANFD_MODE_REG_OFFSET + 0x02)
#define MPFS_CANFD_SETTINGS_OFFSET                              (MPFS_CANFD_MODE_REG_OFFSET + 0x03)

#define MPFS_CANFD_INTERRUPT_REG_OFFSET                         (0x08)
#define MPFS_CANFD_INT_OFFSET                                   (MPFS_CANFD_INTERRUPT_REG_OFFSET + 0x00)                      
#define MPFS_CANFD_INT_ENA_OFFSET                               (MPFS_CANFD_INTERRUPT_REG_OFFSET + 0x02)

#define MPFS_CANFD_TIMING_REG_OFFSET                            (0x0C)
#define MPFS_CANFD_BTR_OFFSET                                   (MPFS_CANFD_TIMING_REG_OFFSET + 0x00)
#define MPFS_CANFD_BTR_FD_OFFSET                                (MPFS_CANFD_TIMING_REG_OFFSET + 0x02)

#define MPFS_CANFD_ALC_PRESC_OFFSET                             (0x10)
#define MPFS_CANFD_ALC_OFFSET                                   (MPFS_CANFD_ALC_PRESC_OFFSET + 0x00)
#define MPFS_CANFD_SJW_OFFSET                                   (MPFS_CANFD_ALC_PRESC_OFFSET + 0x01)
#define MPFS_CANFD_BRP_OFFSET                                   (MPFS_CANFD_ALC_PRESC_OFFSET + 0x02)
#define MPFS_CANFD_BRP_FD_OFFSET                                (MPFS_CANFD_ALC_PRESC_OFFSET + 0x03)

#define MPFS_CANFD_ERROR_TH_OFFSET                              (0x14)
#define MPFS_CANFD_EWL_OFFSET                                   (MPFS_CANFD_ERROR_TH_OFFSET + 0x00)
#define MPFS_CANFD_ERP_OFFSET                                   (MPFS_CANFD_ERROR_TH_OFFSET + 0x11)
#define MPFS_CANFD_FAULT_STATE_OFFSET                           (MPFS_CANFD_ERROR_TH_OFFSET + 0x12)

#define MPFS_CANFD_ERR_COUNTERS_OFFSET                          (0x18)
#define MPFS_CANFD_RXC_OFFSET                                   (MPFS_CANFD_ERR_COUNTERS_OFFSET + 0x00)
#define MPFS_CANFD_TXC_OFFSET                                   (MPFS_CANFD_ERR_COUNTERS_OFFSET + 0x02)

#define MPFS_CANFD_ERR_COUNTERS_SP_OFFSET                       (0x1C)
#define MPFS_CANFD_ERR_NORM_OFFSET                              (MPFS_CANFD_ERR_COUNTERS_SP_OFFSET + 0x00)
#define MPFS_CANFD_ERR_FD_OFFSET                                (MPFS_CANFD_ERR_COUNTERS_SP_OFFSET + 0x02)

#define MPFS_CANFD_FILTER_A_MASK_OFFSET                         (0x20)
#define MPFS_CANFD_FITLER_A_VAL_OFFSET                          (0x24)
#define MPFS_CANFD_FILTER_B_MASK_OFFSET                         (0x28)
#define MPFS_CANFD_FILTER_B_VAL_OFFSET                          (0x2C)
#define MPFS_CANFD_FILTER_C_MASK_OFFSET                         (0x30)
#define MPFS_CANFD_FILTER_C_VAL_OFFSET                          (0x34)
#define MPFS_CANFD_FILTER_RAN_LOW_OFFSET                        (0x38)
#define MPFS_CANFD_FILTER_RAN_HIGH_OFFSET                       (0x3C)
#define MPFS_CANFD_FILTER_CONTROL_OFFSET                        (0x40)

#define MPFS_CANFD_RX_INFO_1_OFFSET                             (0x44)
#define MPFS_CANFD_RX_STATUS_OFFSET                             (MPFS_CANFD_RX_INFO_1_OFFSET + 0x00)                     
#define MPFS_CANFD_RX_MC_OFFSET                                 (MPFS_CANFD_RX_INFO_1_OFFSET + 0x01)
#define MPFS_CANFD_RX_MF_OFFSET                                 (MPFS_CANFD_RX_INFO_1_OFFSET + 0x02)

#define MPFS_CANFD_RX_INFO_2_OFFSET                             (0x48)
#define MPFS_CANFD_RX_BUFF_SIZE_OFFSET                          (MPFS_CANFD_RX_INTO_2_OFFSET + 0x00)
#define MPFS_CANFD_RX_WPP_OFFSET                                (MPFS_CANFD_RX_INTO_2_OFFSET + 0x01)
#define MPFS_CANFD_RX_RPP_OFFSET                                (MPFS_CANFD_RX_INTO_2_OFFSET + 0x02)

#define MPFS_CANFD_RX_DATA_OFFSET                               (0x4C)
#define MPFS_CANFD_TRV_DELAY_OFFSET                             (0x50)

#define MPFS_CANFD_TX_STATUS_OFFSET                             (0x54)
#define MPFS_CANFD_TX_STAT_OFFSET                               (MPFS_CANFD_TX_STATUS_OFFSET + 0x00)

#define MPFS_CANFD_TX_SETTINGS_OFFSET                           (0x58)
#define MPFS_CANFD_TX_SET_OFFSET                                (MPFS_CANFD_TX_SETTINGS_OFFSET + 0x00)

#define MPFS_CANFD_TX_DATA_1_OFFSET                             (0x5C)
#define MPFS_CANFD_TX_DATA_2_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x04)
#define MPFS_CANFD_TX_DATA_3_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x08)
#define MPFS_CANFD_TX_DATA_4_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x0C)
#define MPFS_CANFD_TX_DATA_5_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x10)
#define MPFS_CANFD_TX_DATA_6_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x14)
#define MPFS_CANFD_TX_DATA_7_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x18)
#define MPFS_CANFD_TX_DATA_8_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x1C)
#define MPFS_CANFD_TX_DATA_9_OFFSET                             (MPFS_CANFD_TX_DATA_1_OFFSET + 0x20)
#define MPFS_CANFD_TX_DATA_10_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x24)
#define MPFS_CANFD_TX_DATA_11_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x28)
#define MPFS_CANFD_TX_DATA_12_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x2C)
#define MPFS_CANFD_TX_DATA_13_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x30)
#define MPFS_CANFD_TX_DATA_14_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x34)
#define MPFS_CANFD_TX_DATA_15_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x38)
#define MPFS_CANFD_TX_DATA_16_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x3C)
#define MPFS_CANFD_TX_DATA_17_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x40)
#define MPFS_CANFD_TX_DATA_18_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x44)
#define MPFS_CANFD_TX_DATA_19_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x48)
#define MPFS_CANFD_TX_DATA_20_OFFSET                            (MPFS_CANFD_TX_DATA_1_OFFSET + 0x4C)

#define MPFS_CANFD_RX_COUNTER_OFFSET                            (0xAC)
#define MPFS_CANFD_TX_COUNTER_OFFSET                            (0xB0)

#define MPFS_CANFD_LOG_TRIGGER_CONFIG_OFFSET                    (0xB8)
#define MPFS_CANFD_LOG_CAPT_CONFIG_OFFSET                       (0xC0)

#define MPFS_CANFD_LOG_STATUS_OFFSET                            (0xC4)
#define MPFS_CANFD_LOG_STAT_OFFSET                              (MPFS_CANFD_LOG_STATUS_OFFSET + 0x00)
#define MPFS_CANFD_LOG_WPP_OFFSET                               (MPFS_CANFD_LOG_STATUS_OFFSET + 0x02)
#define MPFS_CANFD_LOG_RPP_OFFSET                               (MPFS_CANFD_LOG_STATUS_OFFSET + 0x03)

#define MPFS_CANFD_LOG_COMMAND_OFFSET                           (0xC8)
#define MPFS_CANFD_LOG_CMD_OFFSET                               (MPFS_CANFD_LOG_COMMAND_OFFSET + 0x00)

#define MPFS_CANFD_LOG_CAPT_EVENT_1_OFFSET                      (0xCC)
#define MPFS_CANFD_LOG_EVENT_TIME_STAMP_UPPER_OFFSET            (MPFS_CANFD_LOG_CAPT_EVENT_1_OFFSET + 0x00)

#define MPFS_CANFD_LOG_CAPT_EVENT_2_OFFSET                      (0xD0)
#define MPFS_CANFD_LOG_CAPT_EVENT_INFO_OFFSET                   (MPFS_CANFD_LOG_CAPT_EVENT_2_OFFSET + 0x00)
#define MPFS_CANFD_LOG_EVENT_TIME_STAMP_LOWER_OFFSET            (MPFS_CANFD_LOG_CAPT_EVENT_2_OFFSET + 0x02)


#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_CAN_H */
