/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_gpdma.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_GPDMA_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_GPDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/stm32n6xxx_dma.h"
#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPDMA1: 16 channels, dual 32-bit AHB master (RM0486 section 19). Channels
 * 0-11 have an 8-byte FIFO, channels 12-15 a 32-byte FIFO and 2D registers.
 * Channel interrupts are contiguous from STM32_IRQ_GPDMA1_CH0.
 */

#define STM32_GPDMA1_NCHANNELS         16
#define STM32_GPDMA1_CHANNEL_MAX       15
#define STM32_GPDMA1_2D_FIRST_CHANNEL  12
#define STM32_GPDMA1_MAX_WIDTH         4
#define STM32_GPDMA1_FIFO_SIZE_SMALL   8
#define STM32_GPDMA1_FIFO_SIZE_LARGE   32
#define STM32_GPDMA1_FIFO_LEVEL_SHIFT  16
#define STM32_GPDMA1_FIFO_LEVEL_MASK   (0xffu << STM32_GPDMA1_FIFO_LEVEL_SHIFT)

#define STM32_GPDMA1_SECCFGR           (STM32_GPDMA1_BASE + STM32_DMA_SECCFGR_OFFSET)
#define STM32_GPDMA1_PRIVCFGR          (STM32_GPDMA1_BASE + STM32_DMA_PRIVCFGR_OFFSET)
#define STM32_GPDMA1_RCFGLOCKR         (STM32_GPDMA1_BASE + STM32_DMA_RCFGLOCKR_OFFSET)
#define STM32_GPDMA1_MISR              (STM32_GPDMA1_BASE + STM32_DMA_MISR_OFFSET)
#define STM32_GPDMA1_SMISR             (STM32_GPDMA1_BASE + STM32_DMA_SMISR_OFFSET)

#define STM32_GPDMA1_CXLBAR(ch)        (STM32_GPDMA1_BASE + STM32_DMA_CXLBAR_OFFSET(ch))
#define STM32_GPDMA1_CXFCR(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXFCR_OFFSET(ch))
#define STM32_GPDMA1_CXSR(ch)          (STM32_GPDMA1_BASE + STM32_DMA_CXSR_OFFSET(ch))
#define STM32_GPDMA1_CXCR(ch)          (STM32_GPDMA1_BASE + STM32_DMA_CXCR_OFFSET(ch))
#define STM32_GPDMA1_CXTR1(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXTR1_OFFSET(ch))
#define STM32_GPDMA1_CXTR2(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXTR2_OFFSET(ch))
#define STM32_GPDMA1_CXBR1(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXBR1_OFFSET(ch))
#define STM32_GPDMA1_CXSAR(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXSAR_OFFSET(ch))
#define STM32_GPDMA1_CXDAR(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXDAR_OFFSET(ch))
#define STM32_GPDMA1_CXTR3(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXTR3_OFFSET(ch))
#define STM32_GPDMA1_CXBR2(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXBR2_OFFSET(ch))
#define STM32_GPDMA1_CXLLR(ch)         (STM32_GPDMA1_BASE + STM32_DMA_CXLLR_OFFSET(ch))

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_GPDMA_H */
