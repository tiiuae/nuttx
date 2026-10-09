/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_dmasigmap.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMASIGMAP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMASIGMAP_H

/* RM0486 Tables 86 and 98 assign the same REQSEL values to requests 0-128
 * listed here on HPDMA1 and GPDMA1. The controller must still be selected
 * explicitly; equal request numbers do not make the controllers or their
 * bus paths interchangeable.
 */

#define STM32_DMA_REQ_ADC1             7
#define STM32_DMA_REQ_ADC2             8

#define STM32_DMA_REQ_TIM1_CC1          14
#define STM32_DMA_REQ_TIM1_CC2          15
#define STM32_DMA_REQ_TIM1_CC3          16
#define STM32_DMA_REQ_TIM1_CC4          17
#define STM32_DMA_REQ_TIM1_UPD          18
#define STM32_DMA_REQ_TIM1_TRG          19
#define STM32_DMA_REQ_TIM1_COM          20
#define STM32_DMA_REQ_TIM2_CC1          21
#define STM32_DMA_REQ_TIM2_CC2          22
#define STM32_DMA_REQ_TIM2_CC3          23
#define STM32_DMA_REQ_TIM2_CC4          24
#define STM32_DMA_REQ_TIM2_UPD          25
#define STM32_DMA_REQ_TIM2_TRG          26
#define STM32_DMA_REQ_TIM3_CC1          27
#define STM32_DMA_REQ_TIM3_CC2          28
#define STM32_DMA_REQ_TIM3_CC3          29
#define STM32_DMA_REQ_TIM3_CC4          30
#define STM32_DMA_REQ_TIM3_UPD          31
#define STM32_DMA_REQ_TIM3_TRG          32
#define STM32_DMA_REQ_TIM4_CC1          33
#define STM32_DMA_REQ_TIM4_CC2          34
#define STM32_DMA_REQ_TIM4_CC3          35
#define STM32_DMA_REQ_TIM4_CC4          36
#define STM32_DMA_REQ_TIM4_UPD          37
#define STM32_DMA_REQ_TIM4_TRG          38
#define STM32_DMA_REQ_TIM5_CC1          39
#define STM32_DMA_REQ_TIM5_CC2          40
#define STM32_DMA_REQ_TIM5_CC3          41
#define STM32_DMA_REQ_TIM5_CC4          42
#define STM32_DMA_REQ_TIM5_UPD          43
#define STM32_DMA_REQ_TIM5_TRG          44
#define STM32_DMA_REQ_TIM6_UPD          45
#define STM32_DMA_REQ_TIM7_UPD          46
#define STM32_DMA_REQ_TIM8_CC1          47
#define STM32_DMA_REQ_TIM8_CC2          48
#define STM32_DMA_REQ_TIM8_CC3          49
#define STM32_DMA_REQ_TIM8_CC4          50
#define STM32_DMA_REQ_TIM8_UPD          51
#define STM32_DMA_REQ_TIM8_TRG          52
#define STM32_DMA_REQ_TIM8_COM          53
#define STM32_DMA_REQ_TIM15_CC1         56
#define STM32_DMA_REQ_TIM15_CC2         57
#define STM32_DMA_REQ_TIM15_UPD         58
#define STM32_DMA_REQ_TIM15_TRG         59
#define STM32_DMA_REQ_TIM15_COM         60
#define STM32_DMA_REQ_TIM16_CC1         61
#define STM32_DMA_REQ_TIM16_UPD         62
#define STM32_DMA_REQ_TIM16_COM         63
#define STM32_DMA_REQ_TIM17_CC1         64
#define STM32_DMA_REQ_TIM17_UPD         65
#define STM32_DMA_REQ_TIM17_COM         66
#define STM32_DMA_REQ_TIM18_CC1         67
#define STM32_DMA_REQ_TIM18_UPD         68
#define STM32_DMA_REQ_TIM18_COM         69
#define STM32_DMA_REQ_LPTIM1_IC1        70
#define STM32_DMA_REQ_LPTIM1_IC2        71
#define STM32_DMA_REQ_LPTIM1_UE         72
#define STM32_DMA_REQ_LPTIM2_IC1        73
#define STM32_DMA_REQ_LPTIM2_IC2        74
#define STM32_DMA_REQ_LPTIM2_UE         75
#define STM32_DMA_REQ_LPTIM3_IC1        76
#define STM32_DMA_REQ_LPTIM3_IC2        77
#define STM32_DMA_REQ_LPTIM3_UE         78

#define STM32_DMA_REQ_SPI1_RX           79
#define STM32_DMA_REQ_SPI1_TX           80
#define STM32_DMA_REQ_SPI2_RX           81
#define STM32_DMA_REQ_SPI2_TX           82
#define STM32_DMA_REQ_SPI3_RX           83
#define STM32_DMA_REQ_SPI3_TX           84
#define STM32_DMA_REQ_SPI4_RX           85
#define STM32_DMA_REQ_SPI4_TX           86
#define STM32_DMA_REQ_SPI5_RX           87
#define STM32_DMA_REQ_SPI5_TX           88
#define STM32_DMA_REQ_SPI6_RX           89
#define STM32_DMA_REQ_SPI6_TX           90

#define STM32_DMA_REQ_I2C1_RX           95
#define STM32_DMA_REQ_I2C1_TX           96
#define STM32_DMA_REQ_I2C2_RX           97
#define STM32_DMA_REQ_I2C2_TX           98
#define STM32_DMA_REQ_I2C3_RX           99
#define STM32_DMA_REQ_I2C3_TX           100
#define STM32_DMA_REQ_I2C4_RX           101
#define STM32_DMA_REQ_I2C4_TX           102

#define STM32_DMA_REQ_USART1_RX         107
#define STM32_DMA_REQ_USART1_TX         108
#define STM32_DMA_REQ_USART2_RX         109
#define STM32_DMA_REQ_USART2_TX         110
#define STM32_DMA_REQ_USART3_RX         111
#define STM32_DMA_REQ_USART3_TX         112
#define STM32_DMA_REQ_UART4_RX          113
#define STM32_DMA_REQ_UART4_TX          114
#define STM32_DMA_REQ_UART5_RX          115
#define STM32_DMA_REQ_UART5_TX          116
#define STM32_DMA_REQ_USART6_RX         117
#define STM32_DMA_REQ_USART6_TX         118
#define STM32_DMA_REQ_UART7_RX          119
#define STM32_DMA_REQ_UART7_TX          120
#define STM32_DMA_REQ_UART8_RX          121
#define STM32_DMA_REQ_UART8_TX          122
#define STM32_DMA_REQ_UART9_RX          123
#define STM32_DMA_REQ_UART9_TX          124
#define STM32_DMA_REQ_USART10_RX        125
#define STM32_DMA_REQ_USART10_TX        126
#define STM32_DMA_REQ_LPUART1_RX        127
#define STM32_DMA_REQ_LPUART1_TX        128

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMASIGMAP_H */
