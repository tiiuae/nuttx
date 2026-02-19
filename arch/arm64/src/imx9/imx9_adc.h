/****************************************************************************
 * arch/arm64/src/imx9/imx9_adc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_ADC_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>

#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Name: imx9_adc_initialize
 *
 * Description:
 *   Initialize i.MX9 ADC lower-half instance implementing adc_ops_s.
 *
 * Input Parameters:
 *   base    - ADC base address.
 *
 * Returned Value:
 *   Valid ADC device structure on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_ADC

FAR struct adc_dev_s *imx9_adc_initialize(uintptr_t base);

/****************************************************************************
 * Name: imx9_adc_read_channel
 *
 * Description:
 *   Perform a single conversion on the selected channel and return the
 *   raw conversion result.
 *
 * Input Parameters:
 *   dev     - ADC lower-half device returned by imx9_adc_initialize().
 *   channel - ADC channel index (0-7).
 *   result  - Pointer to storage for the raw conversion value.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imx9_adc_read_channel(FAR struct adc_dev_s *dev, uint8_t channel,
                          FAR uint16_t *result);

#else

static inline FAR struct adc_dev_s *imx9_adc_initialize(uintptr_t base)
{
	(void)base;
	return NULL;
}

static inline int imx9_adc_read_channel(FAR struct adc_dev_s *dev,
                                        uint8_t channel,
                                        FAR uint16_t *result)
{
	(void)dev;
	(void)channel;
	(void)result;
	return -ENODEV;
}

#endif

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_ADC_H */
