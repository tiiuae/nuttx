/****************************************************************************
 * arch/arm/src/imxrt/imxrt_hyperram.c
 *
 * Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <sys/param.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <arch/barriers.h>

#include "mpu.h"
#include "imxrt_hyperram.h"
#include "imxrt_periphclks.h"
#include "hardware/imxrt_flexspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HR_POLL_US       10000u

#define HR_IP_ERRORS     (FLEXSPI_INTR_IPCMDGE_MASK | \
                          FLEXSPI_INTR_IPCMDERR_MASK | \
                          FLEXSPI_INTR_SEQTIMEOUT_MASK)

/* Follow the core's cache configuration: write-back when the data cache is
 * write-back, write-through when it is write-through or when only the
 * instruction cache is enabled for executable HyperRAM, else noncacheable.
 * On ARMv7-M, TEX=0 C=1 B=0 is Normal write-through; TEX=1 C=1 B=0 is not
 * a valid Normal memory encoding.
 */

#ifdef CONFIG_ARCH_ARMV8M
#  if defined(CONFIG_ARMV8M_DCACHE) && \
      !defined(CONFIG_ARMV8M_DCACHE_WRITETHROUGH)
#    define HR_MPU_CACHE   MPU_RLAR_WRITE_BACK
#  elif defined(CONFIG_ARMV8M_DCACHE) || \
        (defined(CONFIG_ARMV8M_ICACHE) && \
         defined(CONFIG_IMXRT_HYPERRAM_EXECUTABLE))
#    define HR_MPU_CACHE   MPU_RLAR_WRITE_THROUGH
#  else
#    define HR_MPU_CACHE   MPU_RLAR_NONCACHEABLE
#  endif
#else
#  if defined(CONFIG_ARMV7M_DCACHE) && \
      !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
#    define HR_MPU_CACHE   (MPU_RASR_TEX_NOR | MPU_RASR_C | MPU_RASR_B)
#  elif defined(CONFIG_ARMV7M_DCACHE) || \
        (defined(CONFIG_ARMV7M_ICACHE) && \
         defined(CONFIG_IMXRT_HYPERRAM_EXECUTABLE))
#    define HR_MPU_CACHE   (MPU_RASR_TEX_SO | MPU_RASR_C)
#  else
#    define HR_MPU_CACHE   MPU_RASR_TEX_NOR
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hr_wait(struct flexspi_type_s *base, volatile uint32_t *reg,
                   uint32_t mask, uint32_t value)
{
  unsigned int i;

  for (i = 0; i < HR_POLL_US; i++)
    {
      if ((*reg & mask) == value)
        {
          return OK;
        }

      up_udelay(1);
    }

  syslog(LOG_ERR, "HyperRAM: register wait timed out at +0x%03" PRIxPTR
         ": value=%08" PRIx32 " mask=%08" PRIx32
         " expected=%08" PRIx32 "\n",
         (uintptr_t)reg - (uintptr_t)base, *reg, mask, value);
  return -ETIMEDOUT;
}

static int hr_reset(struct flexspi_type_s *g_hr)
{
  /* MDIS gates the clocks needed to complete the software reset. */

  g_hr->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
  g_hr->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
  return hr_wait(g_hr, &g_hr->MCR0, FLEXSPI_MCR0_SWRESET_MASK, 0);
}

/* Only two-byte register transfers are needed here. Keep polling bounded
 * so a missing or incompatible RAM cannot silently stall the bootloader.
 */

static int hr_register(const struct imxrt_hyperram_config_s *config,
                       bool write, uint32_t address, uint32_t *data)
{
  struct flexspi_type_s *g_hr =
    (struct flexspi_type_s *)IMXRT_HYPERRAM_CTRL_BASE;
  unsigned int sequence = write ? config->write_seq : config->read_seq;
  unsigned int i;
  uint32_t status;
  int ret;

  ret = hr_wait(g_hr, &g_hr->STS0,
                FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK,
                FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK);
  if (ret < 0)
    {
      return ret;
    }

  g_hr->INTR = UINT32_MAX;
  g_hr->IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
  g_hr->IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
  g_hr->IPCR0 = address;
  g_hr->IPCR1 = FLEXSPI_IPCR1_ISEQID(sequence) |
                FLEXSPI_IPCR1_IDATSZ(2);

  if (write)
    {
      /* HyperBus registers are MSB-first; FIFO bytes are LSB-first. */

      g_hr->TFDR[0] = ((*data & 0xffu) << 8) | ((*data >> 8) & 0xffu);
    }

  g_hr->IPCMD = FLEXSPI_IPCMD_TRG_MASK;

  for (i = 0; i < HR_POLL_US; i++)
    {
      status = g_hr->INTR;
      if ((status & HR_IP_ERRORS) != 0)
        {
          syslog(LOG_ERR, "HyperRAM: IP sequence %u address=%08" PRIx32
                 " failed INTR=%08" PRIx32 " STS1=%08" PRIx32 "\n",
                 sequence, address, status, g_hr->STS1);
          return -EIO;
        }

      if ((status & FLEXSPI_INTR_IPTXWE_MASK) != 0 &&
          write)
        {
          g_hr->INTR = FLEXSPI_INTR_IPTXWE_MASK;
        }

      if ((status & FLEXSPI_INTR_IPCMDDONE_MASK) != 0)
        {
          if (!write)
            {
              uint32_t raw = g_hr->RFDR[0];

              *data = ((raw & 0xffu) << 8) | ((raw >> 8) & 0xffu);
            }

          g_hr->INTR = FLEXSPI_INTR_IPCMDDONE_MASK;
          return OK;
        }

      up_udelay(1);
    }

  syslog(LOG_ERR, "HyperRAM: IP command timed out\n");
  return -ETIMEDOUT;
}

#ifdef CONFIG_IMXRT_HYPERRAM_TEST
static int hr_memory_test(const struct imxrt_hyperram_config_s *config)
{
  struct flexspi_type_s *g_hr =
    (struct flexspi_type_s *)IMXRT_HYPERRAM_CTRL_BASE;
  const uint32_t offsets[] =
  {
    0, 0x1000, config->size / 2, config->size - 128,
  };

  unsigned int pass;
  unsigned int block;
  unsigned int word;
  uint32_t pattern;
  uint32_t actual;
  volatile uint32_t *memory;
  volatile uint8_t *bytes;
  volatile uint16_t *halfwords;
  int ret;

  /* The MPU mapping may be cacheable. Clean the written lines and reset
   * the controller's receive buffers between writes and reads, then
   * invalidate the lines so the check reaches the external device.
   */

  for (pass = 0; pass < 2; pass++)
    {
      for (block = 0; block < nitems(offsets); block++)
        {
          memory = (volatile uint32_t *)(IMXRT_HYPERRAM_BASE +
                                         offsets[block]);
          for (word = 0; word < 32; word++)
            {
              pattern = 0x5aa50000u ^ offsets[block] ^ (word * 0x01010101u);
              memory[word] = pass == 0 ? pattern : ~pattern;
            }

          /* Check masked byte/halfword writes as well as full words. */

          pattern = 0x5aa50000u ^ offsets[block];
          pattern = pass == 0 ? pattern : ~pattern;
          bytes = (volatile uint8_t *)memory;
          halfwords = (volatile uint16_t *)memory;
          bytes[1] = (uint8_t)((pattern >> 8) ^ 0xffu);
          halfwords[1] = (uint16_t)((pattern >> 16) ^ 0xffffu);
          up_clean_dcache((uintptr_t)memory, (uintptr_t)memory + 128);
        }

      UP_DSB();
      ret = hr_wait(g_hr, &g_hr->STS0,
                    FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK,
                    FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK);
      if (ret < 0)
        {
          return ret;
        }

      ret = hr_reset(g_hr);
      if (ret < 0)
        {
          return ret;
        }

      for (block = 0; block < nitems(offsets); block++)
        {
          memory = (volatile uint32_t *)(IMXRT_HYPERRAM_BASE +
                                         offsets[block]);
          up_invalidate_dcache((uintptr_t)memory, (uintptr_t)memory + 128);
          for (word = 0; word < 32; word++)
            {
              pattern = 0x5aa50000u ^ offsets[block] ^ (word * 0x01010101u);
              pattern = pass == 0 ? pattern : ~pattern;
              if (word == 0)
                {
                  pattern ^= 0xffffff00u;
                }

              actual = memory[word];
              if (actual != pattern)
                {
                  syslog(LOG_ERR, "HyperRAM: test failed at %08" PRIxPTR
                         " expected=%08" PRIx32 " actual=%08" PRIx32 "\n",
                         (uintptr_t)memory + word * 4u,
                         pattern, actual);
                  return -EIO;
                }
            }
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_hyperram_initialize(const struct imxrt_hyperram_config_s *config)
{
  struct flexspi_type_s *g_hr;
  uint32_t data;
  uint32_t flags;
  unsigned int i;
  int ret;

  if (config == NULL || config->lut == NULL ||
      config->lut_words == 0 || config->lut_words > 64 ||
      config->size < 1024 || (config->size & 1023u) != 0 ||
      (config->size & (config->size - 1u)) != 0 ||
      (IMXRT_HYPERRAM_BASE & (config->size - 1u)) != 0 ||
      IMXRT_HYPERRAM_BASE > UINTPTR_MAX - config->size ||
      config->size / 1024 > FLEXSPI_FLSHCR0_FLSHSZ_MASK ||
      config->read_seq >= config->lut_words / 4 ||
      config->write_seq >= config->lut_words / 4 ||
      (config->lut_words & 3u) != 0 ||
      (config->mcr0 & (FLEXSPI_MCR0_SWRESET_MASK |
                      FLEXSPI_MCR0_MDIS_MASK)) != 0)
    {
      syslog(LOG_ERR, "HyperRAM: invalid controller configuration\n");
      return -EINVAL;
    }

#ifdef CONFIG_IMXRT_HYPERRAM_TEST
  if (config->size < 8192)
    {
      syslog(LOG_ERR, "HyperRAM: invalid memory test configuration\n");
      return -EINVAL;
    }
#endif

#ifdef CONFIG_IMXRT_HYPERRAM_FLEXSPI1
  imxrt_clockall_flexspi();
#else
  imxrt_clockall_flexspi2();
#endif

  g_hr = (struct flexspi_type_s *)IMXRT_HYPERRAM_CTRL_BASE;
  ret = hr_reset(g_hr);
  if (ret < 0)
    {
      return ret;
    }

  g_hr->MCR0 = config->mcr0 | FLEXSPI_MCR0_MDIS_MASK;
  g_hr->MCR1 = config->mcr1;
  g_hr->MCR2 = config->mcr2;
  g_hr->AHBCR = config->ahbcr;
  g_hr->INTEN = 0;

  for (i = 0; i < 4; i++)
    {
      g_hr->FLSHCR0[i] = 0;
      g_hr->FLSHCR1[i] = 0;
      g_hr->FLSHCR2[i] = 0;
    }

  g_hr->FLSHCR0[0] = config->size / 1024;
  g_hr->FLSHCR1[0] = config->flshcr1;
  g_hr->FLSHCR2[0] = config->flshcr2;
  g_hr->FLSHCR4 = config->flshcr4;
  g_hr->DLLCR[0] = config->dllcr[0];
  g_hr->DLLCR[1] = config->dllcr[1];

  /* LUT and FIFO accesses are unavailable while MDIS is set. */

  g_hr->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
  g_hr->LUTKEY = FLEXSPI_LUT_KEY_VAL;
  g_hr->LUTCR = FLEXSPI_LUTCR_UNLOCK_MASK;
  for (i = 0; i < 64; i++)
    {
      g_hr->LUT[i] = i < config->lut_words ? config->lut[i] : 0;
    }

  g_hr->LUTKEY = FLEXSPI_LUT_KEY_VAL;
  g_hr->LUTCR = FLEXSPI_LUTCR_LOCK_MASK;
  for (i = 0; i < 64; i++)
    {
      uint32_t expected = i < config->lut_words ? config->lut[i] : 0;
      uint32_t actual = g_hr->LUT[i];

      if (actual != expected)
        {
          syslog(LOG_ERR, "HyperRAM: LUT[%u] readback=%08" PRIx32
                 " expected=%08" PRIx32 "\n", i, actual, expected);
          return -EIO;
        }
    }

  ret = hr_reset(g_hr);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(config->powerup_us);
  ret = hr_register(config, false, 0, &data);
  if (ret < 0)
    {
      return ret;
    }

  if (data != config->device_id)
    {
      syslog(LOG_ERR, "HyperRAM: unexpected device ID=%04" PRIx32 "\n",
             data);
      return -ENODEV;
    }

  data = config->cr0_value;

  /* Register writes have no latency phase and must not drive RWDS. */

  g_hr->FLSHCR4 = 0;
  ret = hr_register(config, true, config->cr0_address, &data);
  g_hr->FLSHCR4 = config->flshcr4;
  if (ret < 0)
    {
      return ret;
    }

  ret = hr_register(config, false, config->cr0_address, &data);
  if (ret < 0)
    {
      return ret;
    }

  if (data != config->cr0_value)
    {
      syslog(LOG_ERR, "HyperRAM: CR0 readback failed: %04" PRIx32 "\n",
             data);
      return -EIO;
    }

  ret = hr_register(config, false, 0, &data);
  if (ret < 0)
    {
      return ret;
    }

  if (data != config->device_id)
    {
      syslog(LOG_ERR, "HyperRAM: unexpected device ID=%04" PRIx32 "\n",
             data);
      return -ENODEV;
    }

  up_invalidate_dcache(IMXRT_HYPERRAM_BASE,
                       IMXRT_HYPERRAM_BASE + config->size);
#ifdef CONFIG_ARCH_ARMV8M
#  ifdef CONFIG_IMXRT_HYPERRAM_WRITABLE
  flags = MPU_RBAR_AP_RWRW;
#  else
  flags = MPU_RBAR_AP_RORO;
#  endif
#  ifndef CONFIG_IMXRT_HYPERRAM_EXECUTABLE
  flags |= MPU_RBAR_XN;
#  endif
  mpu_configure_region(IMXRT_HYPERRAM_BASE, config->size,
                       flags, HR_MPU_CACHE);
#else
#  ifdef CONFIG_IMXRT_HYPERRAM_WRITABLE
  flags = MPU_RASR_AP_RWRW;
#  else
  flags = MPU_RASR_AP_RORO;
#  endif

  flags |= HR_MPU_CACHE;
#  ifndef CONFIG_IMXRT_HYPERRAM_EXECUTABLE
  flags |= MPU_RASR_XN;
#  endif
  mpu_configure_region(IMXRT_HYPERRAM_BASE, config->size,
                       flags);
#endif
  UP_DSB();
  UP_ISB();

#ifdef CONFIG_IMXRT_HYPERRAM_TEST
  return hr_memory_test(config);
#else
  return OK;
#endif
}
