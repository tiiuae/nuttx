/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_hyperram_setup.c
 *
 * Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <syslog.h>

#include "imxrt_hyperram.h"
#include "imxrt_iomuxc.h"
#include "hardware/imxrt_flexspi.h"
#include "imxrt1180-evk.h"

#ifndef CONFIG_IMXRT_HYPERRAM_FLEXSPI2
#  error Board HyperRAM requires FlexSPI2
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HR_LUT(a, x, b, y) \
  FLEXSPI_LUT_SEQ(a, FLEXSPI_8PAD, x, b, FLEXSPI_8PAD, y)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const imxrt_pinset_t g_hr_pins[] =
{
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_30_FLEXSPI2_BUS2BIT_B_DATA03, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_31_FLEXSPI2_BUS2BIT_B_DATA02, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_32_FLEXSPI2_BUS2BIT_B_DATA01, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_33_FLEXSPI2_BUS2BIT_B_DATA00, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_34_FLEXSPI2_BUS2BIT_B_SCLK, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_35_FLEXSPI2_BUS2BIT_A_DATA00, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_36_FLEXSPI2_BUS2BIT_A_DATA01, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_37_FLEXSPI2_BUS2BIT_A_DATA02, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_38_FLEXSPI2_BUS2BIT_A_DATA03, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_39_FLEXSPI2_BUS2BIT_A_SS0_B, 0, 0),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_40_FLEXSPI2_BUS2BIT_A_DQS, 0,
            IOMUXC_MUX_SION_ON),
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B1_41_FLEXSPI2_BUS2BIT_A_SCLK, 0, 0),
};

static const uint32_t g_hr_lut[16] =
{
  [0] = HR_LUT(FLEXSPI_COMMAND_DDR, 0xa0,
               FLEXSPI_COMMAND_RADDR_DDR, 24),
  [1] = HR_LUT(FLEXSPI_COMMAND_CADDR_DDR, 16,
               FLEXSPI_COMMAND_DUMMY_RWDS_DDR, 7),
  [2] = HR_LUT(FLEXSPI_COMMAND_READ_DDR, 4,
               FLEXSPI_COMMAND_STOP, 0),
  [4] = HR_LUT(FLEXSPI_COMMAND_DDR, 0x20,
               FLEXSPI_COMMAND_RADDR_DDR, 24),
  [5] = HR_LUT(FLEXSPI_COMMAND_CADDR_DDR, 16,
               FLEXSPI_COMMAND_DUMMY_RWDS_DDR, 7),
  [6] = HR_LUT(FLEXSPI_COMMAND_WRITE_DDR, 4,
               FLEXSPI_COMMAND_STOP, 0),
  [8] = HR_LUT(FLEXSPI_COMMAND_DDR, 0xe0,
               FLEXSPI_COMMAND_RADDR_DDR, 24),
  [9] = HR_LUT(FLEXSPI_COMMAND_CADDR_DDR, 16,
               FLEXSPI_COMMAND_DUMMY_RWDS_DDR, 7),
  [10] = HR_LUT(FLEXSPI_COMMAND_READ_DDR, 2,
                FLEXSPI_COMMAND_STOP, 0),
  [12] = HR_LUT(FLEXSPI_COMMAND_DDR, 0x60,
                FLEXSPI_COMMAND_RADDR_DDR, 24),
  [13] = HR_LUT(FLEXSPI_COMMAND_CADDR_DDR, 16,
                FLEXSPI_COMMAND_WRITE_DDR, 2),
};

static const struct imxrt_hyperram_config_s g_hr_config =
{
  .size = 8u * 1024u * 1024u,
  .mcr0 = FLEXSPI_MCR0_RXCLKSRC(3) |
          FLEXSPI_MCR0_COMBINATIONEN_MASK |
          FLEXSPI_MCR0_IPGRANTWAIT(0xff) |
          FLEXSPI_MCR0_AHBGRANTWAIT(0xff),
  .mcr1 = FLEXSPI_MCR1_SEQWAIT(0x100) |
          FLEXSPI_MCR1_AHBBUSWAIT(0x100),
  .mcr2 = FLEXSPI_MCR2_SCKBDIFFOPT_MASK |
          FLEXSPI_MCR2_RESUMEWAIT(0x20),
  .ahbcr = FLEXSPI_AHBCR_PREFETCHEN_MASK |
           FLEXSPI_AHBCR_CACHABLEEN_MASK |
           FLEXSPI_AHBCR_READADDROPT_MASK,
  .flshcr1 = FLEXSPI_FLSHCR1_TCSS(3) |
             FLEXSPI_FLSHCR1_TCSH(3) |
             FLEXSPI_FLSHCR1_CAS(3) |
             FLEXSPI_FLSHCR1_WA_MASK |
             FLEXSPI_FLSHCR1_CSINTERVAL(2),
  .flshcr2 = FLEXSPI_FLSHCR2_ARDSEQID(0) |
             FLEXSPI_FLSHCR2_AWRSEQID(1),
  .flshcr4 = FLEXSPI_FLSHCR4_WMENA_MASK,
  .dllcr =
  {
    FLEXSPI_DLLCR_OVRDEN_MASK | FLEXSPI_DLLCR_OVRDVAL(14),
    FLEXSPI_DLLCR_OVRDEN_MASK,
  },
  .lut = g_hr_lut,
  .lut_words = nitems(g_hr_lut),
  .cr0_address = 0x1000,
  .powerup_us = 200,
  .cr0_value = 0x8f27,          /* Variable latency, 7 clocks at 200 MHz */
  .device_id = 0x0c86,
  .read_seq = 2,
  .write_seq = 3,
};

#ifdef CONFIG_ARCH_RAMFUNCS
static const struct flexspi_prefetch_config_s g_hr_prefetch[] =
{
#ifdef CONFIG_ARCH_CORTEXM33
  { .slot = 2, .size = 512, .region_start = 0, .region_end = 0,
    .master_id = 8 }
#else
  { .slot = 0, .size = 512, .region_start = 0, .region_end = 0,
    .master_id = 9 }
#endif
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_hyperram_setup(void)
{
  unsigned int i;
  int ret;

  for (i = 0; i < nitems(g_hr_pins); i++)
    {
      ret = imxrt_iomux_configure(g_hr_pins[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "HyperRAM: pin configuration failed: %d\n", ret);
          return ret;
        }
    }

  ret = imxrt_hyperram_initialize(&g_hr_config);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ARCH_RAMFUNCS
  ret = imxrt_flexspi_prefetch_initialize(
    (struct flexspi_type_s *)IMXRT_HYPERRAM_CTRL_BASE,
    g_hr_prefetch, nitems(g_hr_prefetch));
  if (ret < 0)
    {
      syslog(LOG_ERR, "HyperRAM: prefetch configuration failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
