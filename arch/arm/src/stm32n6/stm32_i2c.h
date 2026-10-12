/****************************************************************************
 * arch/arm/src/stm32n6/stm32_i2c.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_I2C_H
#define __ARCH_ARM_SRC_STM32N6_STM32_I2C_H

#include <nuttx/config.h>

#include "hardware/stm32n6xxx_i2c.h"

struct i2c_master_s;

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s *stm32_i2cbus_initialize(int port);
int stm32_i2cbus_uninitialize(struct i2c_master_s *dev);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_I2C_H */
