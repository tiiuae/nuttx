/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_adc.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_ADC_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/nxxx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC Register Offsets *****************************************************/

#define NXXX_ADC_VERID_OFFSET           0x0000  /* Version ID Register */
#define NXXX_ADC_PARAM_OFFSET           0x0004  /* Parameter Register */
#define NXXX_ADC_CTRL_OFFSET            0x0010  /* Control Register */
#define NXXX_ADC_STAT_OFFSET            0x0014  /* Status Register */
#define NXXX_ADC_IE_OFFSET              0x0018  /* Interrupt Enable Register */
#define NXXX_ADC_DE_OFFSET              0x001C  /* DMA Enable Register */
#define NXXX_ADC_CFG_OFFSET             0x0020  /* Configuration Register */
#define NXXX_ADC_PAUSE_OFFSET           0x0024  /* Pause Register */
#define NXXX_ADC_SWTRIG_OFFSET          0x0034  /* Software Trigger Register */
#define NXXX_ADC_TSTAT_OFFSET           0x0038  /* Trigger Status Register */
#define NXXX_ADC_OFSTRIM_OFFSET         0x0040  /* Offset Trim Register */
#define NXXX_ADC_TCTRL0_OFFSET          0x00A0  /* Trigger Control 0 Register */
#define NXXX_ADC_TCTRL1_OFFSET          0x00A4  /* Trigger Control 1 Register */
#define NXXX_ADC_TCTRL2_OFFSET          0x00A8  /* Trigger Control 2 Register */
#define NXXX_ADC_TCTRL3_OFFSET          0x00AC  /* Trigger Control 3 Register */
#define NXXX_ADC_FCTRL0_OFFSET          0x00E0  /* FIFO Control 0 Register */
#define NXXX_ADC_FCTRL1_OFFSET          0x00E4  /* FIFO Control 1 Register */
#define NXXX_ADC_GCC0_OFFSET            0x00F0  /* Gain Calibration Control 0 Register */
#define NXXX_ADC_GCC1_OFFSET            0x00F4  /* Gain Calibration Control 1 Register */
#define NXXX_ADC_GCR0_OFFSET            0x00F8  /* Gain Calibration Result 0 Register */
#define NXXX_ADC_GCR1_OFFSET            0x00FC  /* Gain Calibration Result 1 Register */
#define NXXX_ADC_CMDL_OFFSET            0x0100  /* Command Low Buffer Register */
#define NXXX_ADC_CMDH_OFFSET            0x0104  /* Command Low Buffer Register */
#define NXXX_ADC_CV_OFFSET              0x0200  /* Compare Value Register */
#define NXXX_ADC_RESFIFO0_OFFSET        0x0300  /* Result FIFO 0 Register */
#define NXXX_ADC_RESFIFO1_OFFSET        0x0304  /* Result FIFO 1 Register */
#define NXXX_ADC_CAL_GAR_OFFSET         0x0400  /* Calibration General A-Side Registers */
#define NXXX_ADC_CAL_GBR_OFFSET         0x0500  /* Calibration General B-Side Registers */

#define NXXX_ADC_CMDL(n)                (NXXX_ADC_CMDL_OFFSET + (n) * 8)
#define NXXX_ADC_CMDH(n)                (NXXX_ADC_CMDH_OFFSET + (n) * 8)
#define NXXX_ADC_CV(n)                  (NXXX_ADC_CV_OFFSET + (n) * 4)
#define NXXX_ADC_CAL_GAR(n)             (NXXX_ADC_CAL_GAR_OFFSET + (n) * 4)
#define NXXX_ADC_CAL_GBR(n)             (NXXX_ADC_CAL_GBR_OFFSET + (n) * 4)

/* CTRL - Control Register */

#define ADC_CTRL_ADCEN                  (1 << 0)
#define ADC_CTRL_RST                    (1 << 1)
#define ADC_CTRL_DOZEN                  (1 << 2)
#define ADC_CTRL_CAL_REQ                (1 << 3)
#define ADC_CTRL_CALOFS                 (1 << 4)
#define ADC_CTRL_RSTFIFO0               (1 << 8)
#define ADC_CTRL_RSTFIFO1               (1 << 9)

#define ADC_CTRL_CAL_AVGS_SHIFT         (16)
#define ADC_CTRL_CAL_AVGS_MASK          (0xf << ADC_CTRL_CAL_AVGS_SHIFT)
#define ADC_CTRL_CAL_AVGS(x)            (((x) << ADC_CTRL_CAL_AVGS_SHIFT) & ADC_CTRL_CAL_AVGS_MASK)
#define ADC_CTRL_CAL_AVG1               (0)
#define ADC_CTRL_CAL_AVG2               (1)
#define ADC_CTRL_CAL_AVG4               (2)
#define ADC_CTRL_CAL_AVG8               (3)
#define ADC_CTRL_CAL_AVG16              (4)
#define ADC_CTRL_CAL_AVG32              (5)
#define ADC_CTRL_CAL_AVG64              (6)
#define ADC_CTRL_CAL_AVG128             (7)
#define ADC_CTRL_CAL_AVG256             (8)
#define ADC_CTRL_CAL_AVG512             (9)
#define ADC_CTRL_CAL_AVG1024            (10)

/* STAT - Status Register */

#define ADC_STAT_RDY0                   (1 << 0)
#define ADC_STAT_FOF0                   (1 << 1)
#define ADC_STAT_RDY1                   (1 << 2)
#define ADC_STAT_FOF1                   (1 << 3)
#define ADC_STAT_TEXC_INT               (1 << 8)
#define ADC_STAT_TCOMP_INT              (1 << 9)
#define ADC_STAT_CAL_RDY                (1 << 10)
#define ADC_STAT_ADC_ACTIVE             (1 << 11)

#define ADC_STAT_TRGACT_SHIFT           (16)
#define ADC_STAT_TRGACT_MASK            (0x3 << ADC_STAT_TRGACT_SHIFT)
#define ADC_STAT_TRGACT(x)              (((x) << ADC_STAT_TRGACT_SHIFT) & ADC_STAT_TRGACT_MASK)

#define ADC_STAT_CMDACT_SHIFT           (24)
#define ADC_STAT_CMDACT_MASK            (0xf << ADC_STAT_CMDACT_SHIFT)
#define ADC_STAT_CMDACT(x)              (((x) << ADC_STAT_CMDACT_SHIFT) & ADC_STAT_CMDACT_MASK)

/* IE - Interrupt Enable Register */

#define ADC_IE_FWMIE0                   (1 << 0)
#define ADC_IE_FOFIE0                   (1 << 1)
#define ADC_IE_FWMIE1                   (1 << 2)
#define ADC_IE_FOFIE1                   (1 << 3)
#define ADC_IE_TEXC_IE                  (1 << 8)

#define ADC_IE_TCOMP_IE_SHIFT           (16)
#define ADC_IE_TCOMP_IE_MASK            (0xf << ADC_IE_TCOMP_IE_SHIFT)
#define ADC_IE_TCOMP_IE(x)              (((x) << ADC_IE_TCOMP_IE_SHIFT) & ADC_IE_TCOMP_IE_MASK)

/* DE - DMA Enable Register */

#define ADC_DE_FWMDE0                   (1 << 0)
#define ADC_DE_FWMDE1                   (1 << 1)

/* CFG - Configuration Register */

#define ADC_CFG_TPRICTRL_SHIFT          (0)
#define ADC_CFG_TPRICTRL_MASK           (0x3 << ADC_CFG_TPRICTRL_SHIFT)
#define ADC_CFG_TPRICTRL(x)             (((x) << ADC_CFG_TPRICTRL_SHIFT) & ADC_CFG_TPRICTRL_MASK)
#define ADC_CFG_TPRICTRL_PREEMPT        (0)
#define ADC_CFG_TPRICTRL_WAIT_CURRENT   (1)
#define ADC_CFG_TPRICTRL_NOPREEMT       (2)

#define ADC_CFG_PWRSEL_SHIFT            (4)
#define ADC_CFG_PWRSEL_MASK             (0x3 << ADC_CFG_PWRSEL_SHIFT)
#define ADC_CFG_PWRSEL(x)               (((x) << ADC_CFG_PWRSEL_SHIFT) & ADC_CFG_PWRSEL_MASK)
#define ADC_CFG_PWRSEL_LOW              (0)
#define ADC_CFG_PWRSEL_HIGH             (1)

/*
 * The ADC voltage references are:
 * - CFG[REFSEL]=00, VREFH reference pin
 * - CFG[REFSEL]=01, ANA_7(VREFI/VREFO) pin
 * - CFG[REFSEL]=10, VDD_ANA supply pin
 */

#define ADC_CFG_REFSEL_SHIFT            (6)
#define ADC_CFG_REFSEL_MASK             (0x3 << ADC_CFG_REFSEL_SHIFT)
#define ADC_CFG_REFSEL(x)               (((x) << ADC_CFG_REFSEL_SHIFT) & ADC_CFG_REFSEL_MASK)
#define ADC_CFG_REFSEL_VREFH            (0)
#define ADC_CFG_REFSEL_ANA7             (1)
#define ADC_CFG_REFSEL_VDDANA           (2)

#define ADC_CFG_TRES                    (1 << 8)
#define ADC_CFG_TCMDRES                 (1 << 9)
#define ADC_CFG_HPT_EXDI                (1 << 10)

#define ADC_CFG_PUDLY_SHIFT             (16)
#define ADC_CFG_PUDLY_MASK              (0xff << ADC_CFG_PUDLY_SHIFT)
#define ADC_CFG_PUDLY(x)                (((x) << ADC_CFG_PUDLY_SHIFT) & ADC_CFG_PUDLY_MASK)

#define ADC_CFG_PWREN                   (1 << 28)

/* PAUSE - Pause Register */

#define ADC_PAUSE_PAUSEDLY_SHIFT        (0)
#define ADC_PAUSE_PAUSEDLY_MASK         (0x1ff << ADC_PAUSE_PAUSEDLY_SHIFT)
#define ADC_PAUSE_PAUSEDLY(x)           (((x) << ADC_PAUSE_PAUSEDLY_SHIFT) & ADC_PAUSE_PAUSEDLY_MASK)

#define ADC_PAUSE_PAUSEEN               (1 << 31)

/* SWTRIG - Software Trigger Register */

#define ADC_SWTRIG_SWT0                 (1 << 0)
#define ADC_SWTRIG_SWT1                 (1 << 1)
#define ADC_SWTRIG_SWT2                 (1 << 2)
#define ADC_SWTRIG_SWT3                 (1 << 3)

/* TSTAT - Trigger Status Register */

#define ADC_TSTAT_TEXC_NUM_SHIFT        (0)
#define ADC_TSTAT_TEXC_NUM_MASK         (0xf << ADC_TSTAT_TEXC_NUM_SHIFT)
#define ADC_TSTAT_TEXC_NUM(x)           (((x) << ADC_TSTAT_TEXC_NUM_SHIFT) & ADC_TSTAT_TEXC_NUM_MASK)

#define ADC_TSTAT_TCOMP_FLAG_SHIFT      (16)
#define ADC_TSTAT_TCOMP_FLAG_MASK       (0xf << ADC_TSTAT_TCOMP_FLAG_SHIFT)
#define ADC_TSTAT_TCOMP_FLAG(x)         (((x) << ADC_TSTAT_TCOMP_FLAG_SHIFT) & ADC_TSTAT_TCOMP_FLAG_MASK)

/* OFSTRIM - Offset Trim Register */

#define ADC_OFSTRIM_OFSTRIM_A_SHIFT     (0)
#define ADC_OFSTRIM_OFSTRIM_A_MASK      (0x1f << ADC_OFSTRIM_OFSTRIM_A_SHIFT)
#define ADC_OFSTRIM_OFSTRIM_A(x)        (((x) << ADC_OFSTRIM_OFSTRIM_A_SHIFT) & ADC_OFSTRIM_OFSTRIM_A_MASK)

#define ADC_OFSTRIM_OFSTRIM_B_SHIFT     (16)
#define ADC_OFSTRIM_OFSTRIM_B_MASK      (0x1f << ADC_OFSTRIM_OFSTRIM_B_SHIFT)
#define ADC_OFSTRIM_OFSTRIM_B(x)        (((x) << ADC_OFSTRIM_OFSTRIM_B_SHIFT) & ADC_OFSTRIM_OFSTRIM_B_MASK)

/* TCTRL - Trigger Control Register */

#define ADC_TCTRL_HTEN                  (1 << 0)
#define ADC_TCTRL_FIFO_SEL_A            (1 << 1)
#define ADC_TCTRL_FIFO_SEL_B            (1 << 2)

#define ADC_TCTRL_TPRI_SHIFT            (8)
#define ADC_TCTRL_TPRI_MASK             (0x3 << ADC_TCTRL_TPRI_SHIFT)
#define ADC_TCTRL_TPRI(x)               (((x) << ADC_TCTRL_TPRI_SHIFT) & ADC_TCTRL_TPRI_MASK)

#define ADC_TCTRL_RSYNC                 (1 << 15)

#define ADC_TCTRL_TDLY_SHIFT            (16)
#define ADC_TCTRL_TDLY_MASK             (0xf << ADC_TCTRL_TDLY_SHIFT)
#define ADC_TCTRL_TDLY(x)               (((x) << ADC_TCTRL_TDLY_SHIFT) & ADC_TCTRL_TDLY_MASK)

#define ADC_TCTRL_TCMD_SHIFT            (24)
#define ADC_TCTRL_TCMD_MASK             (0xf << ADC_TCTRL_TCMD_SHIFT)
#define ADC_TCTRL_TCMD(x)               (((x) << ADC_TCTRL_TCMD_SHIFT) & ADC_TCTRL_TCMD_MASK)

/* FCTRL - FIFO Control Register */

#define ADC_FCTRL_FCOUNT_SHIFT          (0)
#define ADC_FCTRL_FCOUNT_MASK           (0x1f << ADC_FCTRL_FCOUNT_SHIFT)
#define ADC_FCTRL_FCOUNT(x)             (((x) << ADC_FCTRL_FCOUNT_SHIFT) & ADC_FCTRL_FCOUNT_MASK)

#define ADC_FCTRL_FWMARK_SHIFT          (16)
#define ADC_FCTRL_FWMARK_MASK           (0xf << ADC_FCTRL_FWMARK_SHIFT)
#define ADC_FCTRL_FWMARK(x)             (((x) << ADC_FCTRL_FWMARK_SHIFT) & ADC_FCTRL_FWMARK_MASK)

/* GCC - Gain Calibration Control */

#define ADC_GCC_GAIN_CAL_SHIFT          (0)
#define ADC_GCC_GAIN_CAL_MASK           (0xffff << ADC_GCC_GAIN_CAL_SHIFT)
#define ADC_GCC_GAIN_CAL(x)             (((x) << ADC_GCC_GAIN_CAL_SHIFT) & ADC_GCC_GAIN_CAL_MASK)

#define ADC_GCC_RDY                     (1 << 24)

/* GCR - Gain Calculation Result */

#define ADC_GCR_GCALR_SHIFT             (0)
#define ADC_GCR_GCALR_MASK              (0xffff << ADC_GCR_GCALR_SHIFT)
#define ADC_GCR_GCALR(x)                (((x) << ADC_GCR_GCALR_SHIFT) & ADC_GCR_GCALR_MASK)

#define ADC_GCR_RDY                     (1 << 24)

/* CMDL - Command Low Buffer Register */

#define ADC_CMDL_ADCH_SHIFT             (0)
#define ADC_CMDL_ADCH_MASK              (0x1f << ADC_CMDL_ADCH_SHIFT)
#define ADC_CMDL_ADCH(x)                (((x) << ADC_CMDL_ADCH_SHIFT) & ADC_CMDL_ADCH_MASK)

#define ADC_CMDL_CTYPE_SHIFT            (5)
#define ADC_CMDL_CTYPE_MASK             (0x3 << ADC_CMDL_CTYPE_SHIFT)
#define ADC_CMDL_CTYPE(x)               (((x) << ADC_CMDL_CTYPE_SHIFT) & ADC_CMDL_CTYPE_MASK)
#define ADC_CMDL_CTYPE_SINGLEA          (0)
#define ADC_CMDL_CTYPE_SINGLEB          (1)
#define ADC_CMDL_CTYPE_DIFFAB           (2)
#define ADC_CMDL_CTYPE_DUALAB           (3)

#define ADC_CMDL_MODE_12BIT             (0 << 7)
#define ADC_CMDL_MODE_16BIT             (1 << 7)

#define ADC_CMDL_ALTB_ADCH_SHIFT        (16)
#define ADC_CMDL_ALTB_ADCH_MASK         (0x1f << ADC_CMDL_ALTB_ADCH_SHIFT)
#define ADC_CMDL_ALTB_ADCH(x)           (((x) << ADC_CMDL_ALTB_ADCH_SHIFT) & ADC_CMDL_ALTB_ADCH_MASK)

#define ADC_CMDL_ALTBEN                 (1 << 21)

/* CMDH - Command High Buffer Register */

#define ADC_CMDH_CMPEN_SHIFT            (0)
#define ADC_CMDH_CMPEN_MASK             (0x3 << ADC_CMDH_CMPEN_SHIFT)
#define ADC_CMDH_CMPEN(x)               (((x) << ADC_CMDH_CMPEN_SHIFT) & ADC_CMDH_CMPEN_MASK)

#define ADC_CMDH_WAIT_TRIG              (1 << 2)
#define ADC_CMDH_LWI                    (1 << 7)

#define ADC_CMDH_STS_SHIFT              (8)
#define ADC_CMDH_STS_MASK               (0x7 << ADC_CMDH_STS_SHIFT)
#define ADC_CMDH_STS(x)                 (((x) << ADC_CMDH_STS_SHIFT) & ADC_CMDH_STS_MASK)
#define ADC_CMDH_STS_ADCK3              (0)
#define ADC_CMDH_STS_ADCK5              (1)
#define ADC_CMDH_STS_ADCK7              (2)
#define ADC_CMDH_STS_ADCK11             (3)
#define ADC_CMDH_STS_ADCK19             (4)
#define ADC_CMDH_STS_ADCK35             (5)
#define ADC_CMDH_STS_ADCK67             (6)
#define ADC_CMDH_STS_ADCK131            (7)

#define ADC_CMDH_AVGS_SHIFT             (12)
#define ADC_CMDH_AVGS_MASK              (0xf << ADC_CMDH_AVGS_SHIFT)
#define ADC_CMDH_AVGS(x)                (((x) << ADC_CMDH_AVGS_SHIFT) & ADC_CMDH_AVGS_MASK)
#define ADC_CMDH_AVGS1                  (0)
#define ADC_CMDH_AVGS2                  (1)
#define ADC_CMDH_AVGS4                  (2)
#define ADC_CMDH_AVGS8                  (3)
#define ADC_CMDH_AVGS16                 (4)
#define ADC_CMDH_AVGS32                 (5)
#define ADC_CMDH_AVGS64                 (6)
#define ADC_CMDH_AVGS128                (7)
#define ADC_CMDH_AVGS256                (8)
#define ADC_CMDH_AVGS512                (9)
#define ADC_CMDH_AVGS1024               (10)

#define ADC_CMDH_LOOP_SHIFT             (16)
#define ADC_CMDH_LOOP_MASK              (0xf << ADC_CMDH_LOOP_SHIFT)
#define ADC_CMDH_LOOP(x)                (((x) << ADC_CMDH_LOOP_SHIFT) & ADC_CMDH_LOOP_MASK)

#define ADC_CMDH_NEXT_SHIFT             (24)
#define ADC_CMDH_NEXT_MASK              (0xf << ADC_CMDH_NEXT_SHIFT)
#define ADC_CMDH_NEXT(x)                (((x) << ADC_CMDH_NEXT_SHIFT) & ADC_CMDH_NEXT_MASK)

/* CV - Compare Value Register */

#define ADC_CV_CVL_SHIFT                (0)
#define ADC_CV_CVL_MASK                 (0xffff << ADC_CV_CVL_SHIFT)
#define ADC_CV_CVL(x)                   (((x) << ADC_CV_CVL_SHIFT) & ADC_CV_CVL_MASK)

#define ADC_CV_CVH_SHIFT                (16)
#define ADC_CV_CVH_MASK                 (0xffff << ADC_CV_CVH_SHIFT)
#define ADC_CV_CVH(x)                   (((x) << ADC_CV_CVH_SHIFT) & ADC_CV_CVH_MASK)

/* RESFIFO - Data Result FIFO Register */

#define ADC_RESFIFO_D_SHIFT             (0)
#define ADC_RESFIFO_D_MASK              (0xffff << ADC_RESFIFO_D_SHIFT)
#define ADC_RESFIFO_D(x)                (((x) << ADC_RESFIFO_D_SHIFT) & ADC_RESFIFO_D_MASK)

#define ADC_RESFIFO_TSRC_SHIFT          (16)
#define ADC_RESFIFO_TSRC_MASK           (0x3 << ADC_RESFIFO_TSRC_SHIFT)
#define ADC_RESFIFO_TSRC(x)             (((x) << ADC_RESFIFO_TSRC_SHIFT) & ADC_RESFIFO_TSRC_MASK)

#define ADC_RESFIFO_LOOPCNT_SHIFT       (20)
#define ADC_RESFIFO_LOOPCNT_MASK        (0xf << ADC_RESFIFO_LOOPCNT_SHIFT)
#define ADC_RESFIFO_LOOPCNT(x)          (((x) << ADC_RESFIFO_LOOPCNT_SHIFT) & ADC_RESFIFO_LOOPCNT_MASK)

#define ADC_RESFIFO_CMDSRC_SHIFT        (24)
#define ADC_RESFIFO_CMDSRC_MASK         (0xf << ADC_RESFIFO_CMDSRC_SHIFT)
#define ADC_RESFIFO_CMDSRC(x)           (((x) << ADC_RESFIFO_CMDSRC_SHIFT) & ADC_RESFIFO_CMDSRC_MASK)

#define ADC_RESFIFO_VALID               (1 << 31)

/* CAL_GARX - Calibration General A-Side Registers */

#define ADC_CAL_GAR_CAL_GAR_VAL_SHIFT   (0)
#define ADC_CAL_GAR_CAL_GAR_VAL_MASK    (0xffff << ADC_CAL_GAR_CAL_GAR_VAL_SHIFT)
#define ADC_CAL_GAR_CAL_GAR_VAL(x)      (((x) << ADC_CAL_GAR_CAL_GAR_VAL_SHIFT) & ADC_CAL_GAR_CAL_GAR_VAL_MASK)

/* CAL_GBRX - Calibration General B-Side Registers */

#define ADC_CAL_GBR_CAL_GBR_VAL_SHIFT   (0)
#define ADC_CAL_GBR_CAL_GBR_VAL_MASK    (0xffff << ADC_CAL_GBR_CAL_GBR_VAL_SHIFT)
#define ADC_CAL_GBR_CAL_GBR_VAL(x)      (((x) << ADC_CAL_GBR_CAL_GBR_VAL_SHIFT) & ADC_CAL_GBR_CAL_GBR_VAL_MASK)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_ADC_H */
