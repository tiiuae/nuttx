/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_adc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_ADC_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx9_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define IMX9_ADC_MCR_OFFSET           (0x0000) /* Module configuration */
#define IMX9_ADC_MSR_OFFSET           (0x0004) /* Module status */
#define IMX9_ADC_ISR_OFFSET           (0x0010) /* Interrupt status */
#define IMX9_ADC_IMR_OFFSET           (0x0020) /* Interrupt mask */
#define IMX9_ADC_CIMR0_OFFSET         (0x0024) /* Channel interrupt mask 0 */
#define IMX9_ADC_CTR0_OFFSET          (0x0094) /* Channel sample time 0 */
#define IMX9_ADC_NCMR0_OFFSET         (0x00a4) /* Normal conversion mask 0 */
#define IMX9_ADC_PCDR0_OFFSET         (0x0100) /* Port conversion data 0 */
#define IMX9_ADC_PCDR1_OFFSET         (0x0104) /* Port conversion data 1 */
#define IMX9_ADC_PCDR2_OFFSET         (0x0108) /* Port conversion data 2 */
#define IMX9_ADC_PCDR3_OFFSET         (0x010c) /* Port conversion data 3 */
#define IMX9_ADC_PCDR4_OFFSET         (0x0110) /* Port conversion data 4 */
#define IMX9_ADC_PCDR5_OFFSET         (0x0114) /* Port conversion data 5 */
#define IMX9_ADC_PCDR6_OFFSET         (0x0118) /* Port conversion data 6 */
#define IMX9_ADC_PCDR7_OFFSET         (0x011c) /* Port conversion data 7 */
#define IMX9_ADC_CALSTAT_OFFSET       (0x039c) /* Calibration status */

#define IMX9_ADC_PCDR_OFFSET(n)       (0x0100 + ((n) << 2))

/* Register addresses */

#define IMX9_ADC_MCR                  (IMX9_ADC1_BASE + IMX9_ADC_MCR_OFFSET)
#define IMX9_ADC_MSR                  (IMX9_ADC1_BASE + IMX9_ADC_MSR_OFFSET)
#define IMX9_ADC_ISR                  (IMX9_ADC1_BASE + IMX9_ADC_ISR_OFFSET)
#define IMX9_ADC_IMR                  (IMX9_ADC1_BASE + IMX9_ADC_IMR_OFFSET)
#define IMX9_ADC_CIMR0                (IMX9_ADC1_BASE + IMX9_ADC_CIMR0_OFFSET)
#define IMX9_ADC_CTR0                 (IMX9_ADC1_BASE + IMX9_ADC_CTR0_OFFSET)
#define IMX9_ADC_NCMR0                (IMX9_ADC1_BASE + IMX9_ADC_NCMR0_OFFSET)
#define IMX9_ADC_PCDR0                (IMX9_ADC1_BASE + IMX9_ADC_PCDR0_OFFSET)
#define IMX9_ADC_PCDR1                (IMX9_ADC1_BASE + IMX9_ADC_PCDR1_OFFSET)
#define IMX9_ADC_PCDR2                (IMX9_ADC1_BASE + IMX9_ADC_PCDR2_OFFSET)
#define IMX9_ADC_PCDR3                (IMX9_ADC1_BASE + IMX9_ADC_PCDR3_OFFSET)
#define IMX9_ADC_PCDR4                (IMX9_ADC1_BASE + IMX9_ADC_PCDR4_OFFSET)
#define IMX9_ADC_PCDR5                (IMX9_ADC1_BASE + IMX9_ADC_PCDR5_OFFSET)
#define IMX9_ADC_PCDR6                (IMX9_ADC1_BASE + IMX9_ADC_PCDR6_OFFSET)
#define IMX9_ADC_PCDR7                (IMX9_ADC1_BASE + IMX9_ADC_PCDR7_OFFSET)
#define IMX9_ADC_CALSTAT              (IMX9_ADC1_BASE + IMX9_ADC_CALSTAT_OFFSET)

#define IMX9_ADC_PCDR(n)              (IMX9_ADC1_BASE + IMX9_ADC_PCDR_OFFSET(n))

/* MCR bit fields */

#define IMX9_ADC_MCR_MODE_MASK        (1UL << 29)
#define IMX9_ADC_MCR_NSTART_MASK      (1UL << 24)
#define IMX9_ADC_MCR_CALSTART_MASK    (1UL << 14)
#define IMX9_ADC_MCR_ADCLKSE_MASK     (1UL << 8)
#define IMX9_ADC_MCR_PWDN_MASK        (1UL << 0)

/* MSR bit fields */

#define IMX9_ADC_MSR_CALFAIL_MASK     (1UL << 30)
#define IMX9_ADC_MSR_CALBUSY_MASK     (1UL << 29)
#define IMX9_ADC_MSR_ADCSTATUS_MASK   (0x7)

/* ISR bit fields */

#define IMX9_ADC_ISR_ECH_MASK         (1UL << 0)
#define IMX9_ADC_ISR_EOC_MASK         (1UL << 1)
#define IMX9_ADC_ISR_EOC_ECH_MASK     (IMX9_ADC_ISR_EOC_MASK | IMX9_ADC_ISR_ECH_MASK)

/* IMR bit fields */

#define IMX9_ADC_IMR_JEOC_MASK        (1UL << 3)
#define IMX9_ADC_IMR_JECH_MASK        (1UL << 2)
#define IMX9_ADC_IMR_EOC_MASK         (1UL << 1)
#define IMX9_ADC_IMR_ECH_MASK         (1UL << 0)

/* PCDR bit fields */

#define IMX9_ADC_PCDR_CDATA_MASK      (0x0fff)

/* ADC status */

#define IMX9_ADC_MSR_ADCSTATUS_IDLE                  (0)
#define IMX9_ADC_MSR_ADCSTATUS_POWER_DOWN            (1)
#define IMX9_ADC_MSR_ADCSTATUS_WAIT_STATE            (2)
#define IMX9_ADC_MSR_ADCSTATUS_BUSY_IN_CALIBRATION   (3)
#define IMX9_ADC_MSR_ADCSTATUS_SAMPLE                (4)
#define IMX9_ADC_MSR_ADCSTATUS_CONVERSION            (6)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_ADC_H */
