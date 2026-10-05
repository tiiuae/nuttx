# ##############################################################################
# boards/arm/stm32n6/nucleo-n657x0-q/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS stm32_boot.c stm32_bringup.c stm32_bringup_test.c)

if(CONFIG_NUCLEO_N657X0_Q_TIMER_CLOCKTEST)
  list(APPEND SRCS stm32_timer_test.c)
endif()

if(CONFIG_NUCLEO_N657X0_Q_DMA_POLICYTEST)
  list(APPEND SRCS stm32_dma_test.c)
endif()

if(CONFIG_NUCLEO_N657X0_Q_GPIO_EXTI_TEST)
  list(APPEND SRCS stm32_gpio_exti_test.c)
endif()

if(CONFIG_NUCLEO_N657X0_Q_I2C2)
  list(APPEND SRCS stm32_i2c_board.c)
endif()

if(CONFIG_STM32_SPI5)
  list(APPEND SRCS stm32_spi5_bmp280_test.c)
endif()

if(CONFIG_NUCLEO_N657X0_Q_BOOTLOADER)
  list(APPEND SRCS stm32_bootloader.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

target_sources(board PRIVATE ${SRCS})

# The production application executes in place from XSPI2 after the SRAM2
# bootloader configures the flash interface.

if(CONFIG_NUCLEO_N657X0_Q_BOOTLOADER)
  set(LDSCRIPT bl_flash.ld)
elseif(CONFIG_NUCLEO_N657X0_Q_BOOT_XSPI)
  set(LDSCRIPT flash.ld)
else()
  set(LDSCRIPT sram.ld)
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/${LDSCRIPT}")
