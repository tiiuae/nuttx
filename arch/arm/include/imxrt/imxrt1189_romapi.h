/****************************************************************************
 * arch/arm/include/imxrt/imxrt1189_romapi.h
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

#ifndef __ARCH_ARM_INCLUDE_IMXRT_IMXRT1189_ROMAPI_H
#define __ARCH_ARM_INCLUDE_IMXRT_IMXRT1189_ROMAPI_H

#include <stdbool.h>
#include <stdint.h>

typedef int32_t status_t;

#ifndef kStatus_Success
#  define kStatus_Success ((status_t)0)
#endif

#ifndef kStatus_InvalidArgument
#  define kStatus_InvalidArgument ((status_t)4)
#endif

#define FSL_ROM_HAS_FLEXSPINOR_API 1
#define FSL_ROM_HAS_RUNBOOTLOADER_API 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_GET_CONFIG 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_FLASH_INIT 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE_SECTOR 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE_BLOCK 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_ERASE_ALL 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_READ 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_UPDATE_LUT 1
#define FSL_ROM_FLEXSPINOR_API_HAS_FEATURE_CMD_XFER 1

#define FSL_ROM_ROMAPI_VERSION              0x00010103u
#define FSL_ROM_FLEXSPINOR_DRIVER_VERSION  0x00010700u

#define FLEXSPI_CFG_BLK_TAG     0x42464346UL
#define FLEXSPI_CFG_BLK_VERSION 0x56010400UL

typedef struct
{
  union
  {
    struct
    {
      uint32_t max_freq : 4;
      uint32_t misc_mode : 4;
      uint32_t quad_mode_setting : 4;
      uint32_t cmd_pads : 4;
      uint32_t query_pads : 4;
      uint32_t device_type : 4;
      uint32_t option_size : 4;
      uint32_t tag : 4;
    } B;
    uint32_t U;
  } option0;

  union
  {
    struct
    {
      uint32_t dummy_cycles : 8;
      uint32_t status_override : 8;
      uint32_t pinmux_group : 4;
      uint32_t dqs_pinmux_group : 4;
      uint32_t drive_strength : 4;
      uint32_t flash_connection : 4;
    } B;
    uint32_t U;
  } option1;
} serial_nor_config_option_t;

typedef struct
{
  uint8_t seqNum;
  uint8_t seqId;
  uint16_t reserved;
} flexspi_lut_seq_t;

typedef struct
{
  uint8_t time_100ps;
  uint8_t delay_cells;
} flexspi_dll_time_t;

typedef struct
{
  uint32_t tag;
  uint32_t version;
  uint32_t reserved0;
  uint8_t readSampleClkSrc;
  uint8_t csHoldTime;
  uint8_t csSetupTime;
  uint8_t columnAddressWidth;
  uint8_t deviceModeCfgEnable;
  uint8_t deviceModeType;
  uint16_t waitTimeCfgCommands;
  flexspi_lut_seq_t deviceModeSeq;
  uint32_t deviceModeArg;
  uint8_t configCmdEnable;
  uint8_t configModeType[3];
  flexspi_lut_seq_t configCmdSeqs[3];
  uint32_t reserved1;
  uint32_t configCmdArgs[3];
  uint32_t reserved2;
  uint32_t controllerMiscOption;
  uint8_t deviceType;
  uint8_t sflashPadType;
  uint8_t serialClkFreq;
  uint8_t lutCustomSeqEnable;
  uint32_t reserved3[2];
  uint32_t sflashA1Size;
  uint32_t sflashA2Size;
  uint32_t sflashB1Size;
  uint32_t sflashB2Size;
  uint32_t csPadSettingOverride;
  uint32_t sclkPadSettingOverride;
  uint32_t dataPadSettingOverride;
  uint32_t dqsPadSettingOverride;
  uint32_t timeoutInMs;
  uint32_t commandInterval;
  flexspi_dll_time_t dataValidTime[2];
  uint16_t busyOffset;
  uint16_t busyBitPolarity;
  uint32_t lookupTable[64];
  flexspi_lut_seq_t lutCustomSeq[12];
  uint32_t reserved4[4];
} flexspi_mem_config_t;

typedef struct
{
  flexspi_mem_config_t memConfig;
  uint32_t pageSize;
  uint32_t sectorSize;
  uint8_t ipcmdSerialClkFreq;
  uint8_t isUniformBlockSize;
  uint8_t isDataOrderSwapped;
  uint8_t reserved0;
  uint8_t serialNorType;
  uint8_t needExitNoCmdMode;
  uint8_t halfClkForNonReadCmd;
  uint8_t needRestoreNoCmdMode;
  uint32_t blockSize;
  uint32_t reserve2[11];
} flexspi_nor_config_t;

typedef enum
{
  kFLEXSPIOperation_Command,
  kFLEXSPIOperation_Config,
  kFLEXSPIOperation_Write,
  kFLEXSPIOperation_Read
} flexspi_operation_t;

typedef struct
{
  flexspi_operation_t operation;
  uint32_t baseAddress;
  uint32_t seqId;
  uint32_t seqNum;
  bool isParallelModeEnable;
  uint32_t *txBuffer;
  uint32_t txSize;
  uint32_t *rxBuffer;
  uint32_t rxSize;
} flexspi_xfer_t;

/* ROM API entry points.  The RT1189 adapter uses the function table directly,
 * but these declarations keep the public ROM interface available to NuttX. */

status_t ROM_FLEXSPI_NorFlash_GetConfig(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        serial_nor_config_option_t *option);
status_t ROM_FLEXSPI_NorFlash_Init(uint32_t instance,
                                   flexspi_nor_config_t *config);
status_t ROM_FLEXSPI_NorFlash_ProgramPage(uint32_t instance,
                                          flexspi_nor_config_t *config,
                                          uint32_t address,
                                          const uint32_t *src);
status_t ROM_FLEXSPI_NorFlash_Read(uint32_t instance,
                                   flexspi_nor_config_t *config,
                                   uint32_t *dst, uint32_t address,
                                   uint32_t size);
status_t ROM_FLEXSPI_NorFlash_Erase(uint32_t instance,
                                    flexspi_nor_config_t *config,
                                    uint32_t address, uint32_t size);
status_t ROM_FLEXSPI_NorFlash_EraseSector(uint32_t instance,
                                          flexspi_nor_config_t *config,
                                          uint32_t address);
status_t ROM_FLEXSPI_NorFlash_EraseBlock(uint32_t instance,
                                         flexspi_nor_config_t *config,
                                         uint32_t address);
status_t ROM_FLEXSPI_NorFlash_EraseAll(uint32_t instance,
                                       flexspi_nor_config_t *config);
status_t ROM_FLEXSPI_NorFlash_CommandXfer(uint32_t instance,
                                           flexspi_xfer_t *xfer);

#endif /* __ARCH_ARM_INCLUDE_IMXRT_IMXRT1189_ROMAPI_H */
