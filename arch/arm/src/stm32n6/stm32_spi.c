/****************************************************************************
 * arch/arm/src/stm32n6/stm32_spi.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "hardware/stm32n6xxx_memorymap.h"
#include "dwt.h"
#include "nvic.h"

#if defined(CONFIG_STM32_SPI1_DMA) || defined(CONFIG_STM32_SPI2_DMA) || \
    defined(CONFIG_STM32_SPI3_DMA) || defined(CONFIG_STM32_SPI4_DMA) || \
    defined(CONFIG_STM32_SPI5_DMA) || defined(CONFIG_STM32_SPI6_DMA)
#  error "STM32N6 SPI DMA support is not implemented"
#endif

#ifdef CONFIG_STM32_SPI_INTERRUPTS
#  error "STM32N6 SPI interrupt support is not implemented"
#endif

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3) || defined(CONFIG_STM32_SPI4) || \
    defined(CONFIG_STM32_SPI5) || defined(CONFIG_STM32_SPI6)

#ifndef STM32_HSI_FREQUENCY
#  error "STM32_HSI_FREQUENCY must be defined by the board"
#endif

#ifndef STM32_CPUCLK_FREQUENCY
#  error "STM32_CPUCLK_FREQUENCY must be defined by the board"
#endif

#define SPI_TIMEOUT_MARGIN_US       100000u
#define SPI_ABORT_TIMEOUT_US        100000u

#define SPI_IFCR_ERRORS             (SPI_IFCR_MODFC | SPI_IFCR_TIFREC | \
                                     SPI_IFCR_CRCEC | SPI_IFCR_OVRC | \
                                     SPI_IFCR_UDRC)
#define SPI_IFCR_CLEARABLE          (SPI_IFCR_SUSPC | SPI_IFCR_ERRORS | \
                                     SPI_IFCR_TXTFC | SPI_IFCR_EOTC)
#define SPI_SR_ERRORS               (SPI_SR_MODF | SPI_SR_TIFRE | \
                                     SPI_SR_CRCE | SPI_SR_OVR | SPI_SR_UDR)

struct stm32_spi_priv_s
{
  struct spi_dev_s dev;
  mutex_t lock;
  uintptr_t base;
  uintptr_t rcc_enable;
  uintptr_t rcc_reset_set;
  uintptr_t rcc_reset_clear;
  uint32_t rcc_enable_mask;
  uint32_t rcc_reset_mask;
  uint32_t ccipr_mask;
  uint32_t ccipr_source;
  uint32_t frequency;
  uint8_t bus;
  uint8_t nbits;
  enum spi_mode_e mode;
  int last_error;
  bool lock_initialized;
  bool reset_done;
  bool initialized;
  bool faulted;
};

struct spi_deadline_s
{
  uint32_t start;
  uint32_t cycles;
};

static int spi_lock(struct spi_dev_s *dev, bool lock);
static void spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected);
static uint8_t spi_status(struct spi_dev_s *dev, uint32_t devid);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t word);
static int spi_transfer(struct stm32_spi_priv_s *priv,
                        const void *txbuffer, void *rxbuffer, size_t nwords);

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features);
#endif

#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_setdelay(struct spi_dev_s *dev, uint32_t a, uint32_t b,
                        uint32_t c, uint32_t i);
#endif

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev);
#endif

#ifdef CONFIG_SPI_EXCHANGE
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords);
#else
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords);
static void spi_recvblock(struct spi_dev_s *dev, void *buffer, size_t nwords);
#endif

static const struct spi_ops_s g_spi_ops =
{
  .lock            = spi_lock,
  .select          = spi_select,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay        = spi_setdelay,
#endif
  .setfrequency    = spi_setfrequency,
  .setmode         = spi_setmode,
  .setbits         = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures      = spi_hwfeatures,
#endif
  .status          = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata         = spi_cmddata,
#endif
  .send            = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange        = spi_exchange,
#else
  .sndblock        = spi_sndblock,
  .recvblock       = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger         = spi_trigger,
#endif
  .registercallback = NULL
};

#define SPI_PRIV_INITIALIZER(n, b, en, rstset, rstclr, enmask, rstmask, \
                             selmask, selval)                            \
  {                                                                      \
    .dev             = { &g_spi_ops },                                   \
    .base            = (b),                                              \
    .rcc_enable      = (en),                                             \
    .rcc_reset_set   = (rstset),                                         \
    .rcc_reset_clear = (rstclr),                                         \
    .rcc_enable_mask = (enmask),                                         \
    .rcc_reset_mask  = (rstmask),                                        \
    .ccipr_mask      = (selmask),                                        \
    .ccipr_source    = (selval),                                         \
    .frequency       = STM32_HSI_FREQUENCY / 256u,                        \
    .bus             = (n),                                              \
    .nbits           = 8,                                                \
    .mode            = SPIDEV_MODE0                                       \
  }

#ifdef CONFIG_STM32_SPI1
static struct stm32_spi_priv_s g_spi1 =
  SPI_PRIV_INITIALIZER(1, STM32_SPI1_BASE, STM32_RCC_APB2ENSR,
                       STM32_RCC_APB2RSTSR, STM32_RCC_APB2RSTCR,
                       RCC_APB2ENSR_SPI1ENS, RCC_APB2RSTSR_SPI1RSTS,
                       RCC_CCIPR9_SPI1SEL_MASK, RCC_CCIPR9_SPI1SEL_HSI_DIV_CK);
#endif

#ifdef CONFIG_STM32_SPI2
static struct stm32_spi_priv_s g_spi2 =
  SPI_PRIV_INITIALIZER(2, STM32_SPI2_BASE, STM32_RCC_APB1LENSR,
                       STM32_RCC_APB1LRSTSR, STM32_RCC_APB1LRSTCR,
                       RCC_APB1LENSR_SPI2ENS, RCC_APB1LRSTSR_SPI2RSTS,
                       RCC_CCIPR9_SPI2SEL_MASK, RCC_CCIPR9_SPI2SEL_HSI_DIV_CK);
#endif

#ifdef CONFIG_STM32_SPI3
static struct stm32_spi_priv_s g_spi3 =
  SPI_PRIV_INITIALIZER(3, STM32_SPI3_BASE, STM32_RCC_APB1LENSR,
                       STM32_RCC_APB1LRSTSR, STM32_RCC_APB1LRSTCR,
                       RCC_APB1LENSR_SPI3ENS, RCC_APB1LRSTSR_SPI3RSTS,
                       RCC_CCIPR9_SPI3SEL_MASK, RCC_CCIPR9_SPI3SEL_HSI_DIV_CK);
#endif

#ifdef CONFIG_STM32_SPI4
static struct stm32_spi_priv_s g_spi4 =
  SPI_PRIV_INITIALIZER(4, STM32_SPI4_BASE, STM32_RCC_APB2ENSR,
                       STM32_RCC_APB2RSTSR, STM32_RCC_APB2RSTCR,
                       RCC_APB2ENSR_SPI4ENS, RCC_APB2RSTSR_SPI4RSTS,
                       RCC_CCIPR9_SPI4SEL_MASK, RCC_CCIPR9_SPI4SEL_HSI_DIV_CK);
#endif

#ifdef CONFIG_STM32_SPI5
static struct stm32_spi_priv_s g_spi5 =
  SPI_PRIV_INITIALIZER(5, STM32_SPI5_BASE, STM32_RCC_APB2ENSR,
                       STM32_RCC_APB2RSTSR, STM32_RCC_APB2RSTCR,
                       RCC_APB2ENSR_SPI5ENS, RCC_APB2RSTSR_SPI5RSTS,
                       RCC_CCIPR9_SPI5SEL_MASK, RCC_CCIPR9_SPI5SEL_HSI_DIV_CK);
#endif

#ifdef CONFIG_STM32_SPI6
static struct stm32_spi_priv_s g_spi6 =
  SPI_PRIV_INITIALIZER(6, STM32_SPI6_BASE, STM32_RCC_APB4LENSR,
                       STM32_RCC_APB4LRSTSR, STM32_RCC_APB4LRSTCR,
                       RCC_APB4LENSR_SPI6ENS, RCC_APB4LRSTSR_SPI6RSTS,
                       RCC_CCIPR9_SPI6SEL_MASK, RCC_CCIPR9_SPI6SEL_HSI_DIV_CK);
#endif

static inline uint32_t spi_getreg(struct stm32_spi_priv_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void spi_putreg(struct stm32_spi_priv_s *priv,
                              unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static bool spi_dwt_initialize(void)
{
  modifyreg32(NVIC_DEMCR, 0, NVIC_DEMCR_TRCENA);
  if ((getreg32(DWT_CTRL) & DWT_CTRL_NOCYCCNT_MASK) != 0)
    {
      return false;
    }

  modifyreg32(DWT_CTRL, 0, DWT_CTRL_CYCCNTENA_MASK);
  return (getreg32(DWT_CTRL) & DWT_CTRL_CYCCNTENA_MASK) != 0;
}

static bool spi_deadline_start(struct spi_deadline_s *deadline,
                               uint64_t timeout_us)
{
  uint64_t cycles;

  cycles = (timeout_us * STM32_CPUCLK_FREQUENCY + 999999u) / 1000000u;
  if (cycles == 0 || cycles >= 0x100000000ull)
    {
      return false;
    }

  deadline->start  = getreg32(DWT_CYCCNT);
  deadline->cycles = (uint32_t)cycles;
  return true;
}

static bool spi_deadline_expired(const struct spi_deadline_s *deadline)
{
  return (uint32_t)(getreg32(DWT_CYCCNT) - deadline->start) >=
         deadline->cycles;
}

static int spi_config_pins(struct stm32_spi_priv_s *priv)
{
  int ret = OK;

  switch (priv->bus)
    {
#ifdef CONFIG_STM32_SPI1
      case 1:
#ifdef GPIO_SPI1_SCK
        ret = stm32_configgpio(GPIO_SPI1_SCK);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI1_MISO
        ret = stm32_configgpio(GPIO_SPI1_MISO);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI1_MOSI
        ret = stm32_configgpio(GPIO_SPI1_MOSI);
        if (ret < 0)
          {
            return ret;
          }
#endif
        break;
#endif
#ifdef CONFIG_STM32_SPI2
      case 2:
#ifdef GPIO_SPI2_SCK
        ret = stm32_configgpio(GPIO_SPI2_SCK);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI2_MISO
        ret = stm32_configgpio(GPIO_SPI2_MISO);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI2_MOSI
        ret = stm32_configgpio(GPIO_SPI2_MOSI);
        if (ret < 0)
          {
            return ret;
          }
#endif
        break;
#endif
#ifdef CONFIG_STM32_SPI3
      case 3:
#ifdef GPIO_SPI3_SCK
        ret = stm32_configgpio(GPIO_SPI3_SCK);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI3_MISO
        ret = stm32_configgpio(GPIO_SPI3_MISO);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI3_MOSI
        ret = stm32_configgpio(GPIO_SPI3_MOSI);
        if (ret < 0)
          {
            return ret;
          }
#endif
        break;
#endif
#ifdef CONFIG_STM32_SPI4
      case 4:
#ifdef GPIO_SPI4_SCK
        ret = stm32_configgpio(GPIO_SPI4_SCK);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI4_MISO
        ret = stm32_configgpio(GPIO_SPI4_MISO);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI4_MOSI
        ret = stm32_configgpio(GPIO_SPI4_MOSI);
        if (ret < 0)
          {
            return ret;
          }
#endif
        break;
#endif
#ifdef CONFIG_STM32_SPI5
      case 5:
#ifdef GPIO_SPI5_SCK
        ret = stm32_configgpio(GPIO_SPI5_SCK);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI5_MISO
        ret = stm32_configgpio(GPIO_SPI5_MISO);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI5_MOSI
        ret = stm32_configgpio(GPIO_SPI5_MOSI);
        if (ret < 0)
          {
            return ret;
          }
#endif
        break;
#endif
#ifdef CONFIG_STM32_SPI6
      case 6:
#ifdef GPIO_SPI6_SCK
        ret = stm32_configgpio(GPIO_SPI6_SCK);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI6_MISO
        ret = stm32_configgpio(GPIO_SPI6_MISO);
        if (ret < 0)
          {
            return ret;
          }
#endif
#ifdef GPIO_SPI6_MOSI
        ret = stm32_configgpio(GPIO_SPI6_MOSI);
        if (ret < 0)
          {
            return ret;
          }
#endif
        break;
#endif
      default:
        return -EINVAL;
    }

  return ret;
}

static int spi_initialize(struct stm32_spi_priv_s *priv)
{
  irqstate_t flags;
  uint32_t cfg1;
  uint32_t cfg2;
  int ret;

  flags = enter_critical_section();
  if (priv->initialized)
    {
      leave_critical_section(flags);
      return OK;
    }

  if (!priv->lock_initialized)
    {
      ret = nxmutex_init(&priv->lock);
      if (ret < 0)
        {
          leave_critical_section(flags);
          return ret;
        }

      priv->lock_initialized = true;
    }

  if (!spi_dwt_initialize())
    {
      leave_critical_section(flags);
      return -ENODEV;
    }

  modifyreg32(STM32_RCC_CCIPR9, priv->ccipr_mask, priv->ccipr_source);
  putreg32(priv->rcc_enable_mask, priv->rcc_enable);

  ret = spi_config_pins(priv);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  if (!priv->reset_done)
    {
      putreg32(priv->rcc_reset_mask, priv->rcc_reset_set);
      putreg32(priv->rcc_reset_mask, priv->rcc_reset_clear);
      priv->reset_done = true;
    }

  spi_putreg(priv, STM32_SPI_CR1_OFFSET, 0);
  spi_putreg(priv, STM32_SPI_IER_OFFSET, 0);
  spi_putreg(priv, STM32_SPI_IFCR_OFFSET, SPI_IFCR_CLEARABLE);

  cfg1 = SPI_CFG1_FTHLV_1DATA |
         SPI_CFG1_DSIZE_8BIT |
         SPI_CFG1_MBR_DIV256;
  spi_putreg(priv, STM32_SPI_CFG1_OFFSET, cfg1);

  cfg2 = SPI_CFG2_AFCNTR | SPI_CFG2_MASTER | SPI_CFG2_SSM |
         SPI_CFG2_COMM_FULLDUPLEX;
  spi_putreg(priv, STM32_SPI_CFG2_OFFSET, cfg2);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI | SPI_CR1_SPE);
  up_udelay(1);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  priv->initialized = true;
  leave_critical_section(flags);
  return OK;
}

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

static void spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  switch (priv->bus)
    {
#ifdef CONFIG_STM32_SPI1
      case 1:
        stm32_spi1select(dev, devid, selected);
        break;
#endif
#ifdef CONFIG_STM32_SPI2
      case 2:
        stm32_spi2select(dev, devid, selected);
        break;
#endif
#ifdef CONFIG_STM32_SPI3
      case 3:
        stm32_spi3select(dev, devid, selected);
        break;
#endif
#ifdef CONFIG_STM32_SPI4
      case 4:
        stm32_spi4select(dev, devid, selected);
        break;
#endif
#ifdef CONFIG_STM32_SPI5
      case 5:
        stm32_spi5select(dev, devid, selected);
        break;
#endif
#ifdef CONFIG_STM32_SPI6
      case 6:
        stm32_spi6select(dev, devid, selected);
        break;
#endif
      default:
        spierr("ERROR: select called for invalid SPI bus %u\n",
               (unsigned int)priv->bus);
        break;
    }
}

static uint8_t spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  switch (priv->bus)
    {
#ifdef CONFIG_STM32_SPI1
      case 1: return stm32_spi1status(dev, devid);
#endif
#ifdef CONFIG_STM32_SPI2
      case 2: return stm32_spi2status(dev, devid);
#endif
#ifdef CONFIG_STM32_SPI3
      case 3: return stm32_spi3status(dev, devid);
#endif
#ifdef CONFIG_STM32_SPI4
      case 4: return stm32_spi4status(dev, devid);
#endif
#ifdef CONFIG_STM32_SPI5
      case 5: return stm32_spi5status(dev, devid);
#endif
#ifdef CONFIG_STM32_SPI6
      case 6: return stm32_spi6status(dev, devid);
#endif
      default:
        spierr("ERROR: status called for invalid SPI bus %u\n",
               (unsigned int)priv->bus);
        return 0;
    }
}

static int spi_apply_mode(struct stm32_spi_priv_s *priv)
{
  uint32_t cfg2 = SPI_CFG2_AFCNTR | SPI_CFG2_MASTER | SPI_CFG2_SSM |
                  SPI_CFG2_COMM_FULLDUPLEX;

  if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) != 0)
    {
      return -EBUSY;
    }

  switch (priv->mode)
    {
      case SPIDEV_MODE0:
        break;

      case SPIDEV_MODE1:
        cfg2 |= SPI_CFG2_CPHA;
        break;

      case SPIDEV_MODE2:
        cfg2 |= SPI_CFG2_CPOL;
        break;

      case SPIDEV_MODE3:
        cfg2 |= SPI_CFG2_CPOL | SPI_CFG2_CPHA;
        break;

      default:
        return -EINVAL;
    }

  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  spi_putreg(priv, STM32_SPI_CFG2_OFFSET, cfg2);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI | SPI_CR1_SPE);
  up_udelay(1);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  return OK;
}

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;
  enum spi_mode_e oldmode = priv->mode;

  priv->mode = mode;
  priv->last_error = spi_apply_mode(priv);
  if (priv->last_error < 0)
    {
      priv->mode = oldmode;
      spierr("ERROR: SPI%u failed to set mode: %d\n",
             priv->bus, priv->last_error);
    }
}

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;
  static const uint16_t dividers[] =
  {
    2, 4, 8, 16, 32, 64, 128, 256
  };
  static const uint32_t mbr[] =
  {
    SPI_CFG1_MBR_DIV2, SPI_CFG1_MBR_DIV4, SPI_CFG1_MBR_DIV8,
    SPI_CFG1_MBR_DIV16, SPI_CFG1_MBR_DIV32, SPI_CFG1_MBR_DIV64,
    SPI_CFG1_MBR_DIV128, SPI_CFG1_MBR_DIV256
  };
  uint32_t actual = 0;
  uint32_t cfg1;
  unsigned int i;

  if (frequency == 0)
    {
      priv->last_error = -EINVAL;
      spierr("ERROR: SPI%u rejected zero frequency\n", priv->bus);
      return 0;
    }

  for (i = 0; i < sizeof(dividers) / sizeof(dividers[0]); i++)
    {
      uint32_t candidate = STM32_HSI_FREQUENCY / dividers[i];

      if (candidate <= frequency)
        {
          actual = candidate;
          break;
        }
    }

  if (actual == 0)
    {
      priv->last_error = -ERANGE;
      spierr("ERROR: SPI%u requested frequency %lu below minimum\n",
             priv->bus, (unsigned long)frequency);
      return 0;
    }

  if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) != 0)
    {
      priv->last_error = -EBUSY;
      spierr("ERROR: SPI%u frequency change while enabled\n", priv->bus);
      return 0;
    }

  cfg1 = spi_getreg(priv, STM32_SPI_CFG1_OFFSET);
  cfg1 &= ~SPI_CFG1_MBR_MASK;
  cfg1 |= mbr[i];
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  spi_putreg(priv, STM32_SPI_CFG1_OFFSET, cfg1);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI | SPI_CR1_SPE);
  up_udelay(1);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);

  priv->frequency = actual;
  priv->last_error = 0;
  return actual;
}

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;
  uint32_t cfg1;

  if (nbits != 8 && nbits != 16)
    {
      priv->last_error = -EINVAL;
      spierr("ERROR: SPI%u unsupported frame width %d\n", priv->bus, nbits);
      return;
    }

  cfg1 = spi_getreg(priv, STM32_SPI_CFG1_OFFSET);
  if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) != 0)
    {
      priv->last_error = -EBUSY;
      spierr("ERROR: SPI%u frame-width change while enabled\n", priv->bus);
      return;
    }

  cfg1 &= ~SPI_CFG1_DSIZE_MASK;
  cfg1 |= nbits == 8 ? SPI_CFG1_DSIZE_8BIT : SPI_CFG1_DSIZE_16BIT;

  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  spi_putreg(priv, STM32_SPI_CFG1_OFFSET, cfg1);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI | SPI_CR1_SPE);
  up_udelay(1);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  priv->nbits = nbits;
  priv->last_error = 0;
}

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  priv->last_error = features == 0 ? 0 : -ENOSYS;
  return priv->last_error;
}
#endif

#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_setdelay(struct spi_dev_s *dev, uint32_t a, uint32_t b,
                        uint32_t c, uint32_t i)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  UNUSED(a);
  UNUSED(b);
  UNUSED(c);
  UNUSED(i);
  priv->last_error = -ENOSYS;
  return priv->last_error;
}
#endif

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  UNUSED(devid);
  UNUSED(cmd);
  priv->last_error = -ENOSYS;
  return priv->last_error;
}
#endif

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;

  priv->last_error = -ENOSYS;
  return priv->last_error;
}
#endif

static uint16_t spi_get_txframe(const uint8_t *buffer, size_t index,
                                uint8_t nbits)
{
  uint16_t frame;

  if (buffer == NULL)
    {
      return nbits == 8 ? 0xffu : 0xffffu;
    }

  if (nbits == 8)
    {
      return buffer[index];
    }

  memcpy(&frame, buffer + index * sizeof(frame), sizeof(frame));
  return frame;
}

static void spi_put_rxframe(uint8_t *buffer, size_t index, uint8_t nbits,
                            uint16_t frame)
{
  if (buffer == NULL)
    {
      return;
    }

  if (nbits == 8)
    {
      buffer[index] = (uint8_t)frame;
    }
  else
    {
      memcpy(buffer + index * sizeof(frame), &frame, sizeof(frame));
    }
}

static bool spi_rx_pending(uint32_t status)
{
  return (status & (SPI_SR_RXWNE | SPI_SR_RXPLVL_MASK)) != 0;
}

static uint16_t spi_read_rxframe(struct stm32_spi_priv_s *priv)
{
  if (priv->nbits == 8)
    {
      return getreg8(priv->base + STM32_SPI_RXDR_OFFSET);
    }

  return getreg16(priv->base + STM32_SPI_RXDR_OFFSET);
}

static void spi_write_txframe(struct stm32_spi_priv_s *priv, uint16_t frame)
{
  if (priv->nbits == 8)
    {
      putreg8((uint8_t)frame, priv->base + STM32_SPI_TXDR_OFFSET);
    }
  else
    {
      putreg16(frame, priv->base + STM32_SPI_TXDR_OFFSET);
    }
}

static uint64_t spi_transfer_timeout_us(struct stm32_spi_priv_s *priv,
                                       size_t nframes)
{
  uint64_t bits = (uint64_t)nframes * priv->nbits;
  uint64_t wire_us = (bits * 1000000u + priv->frequency - 1u) /
                     priv->frequency;

  return wire_us + SPI_TIMEOUT_MARGIN_US;
}

static void spi_drain_rx(struct stm32_spi_priv_s *priv)
{
  uint32_t status = spi_getreg(priv, STM32_SPI_SR_OFFSET);

  while (spi_rx_pending(status))
    {
      (void)spi_read_rxframe(priv);
      status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
    }
}

static void spi_abort_transfer(struct stm32_spi_priv_s *priv, bool started)
{
  struct spi_deadline_s deadline;
  uint32_t cr1;
  uint32_t status;

  cr1 = spi_getreg(priv, STM32_SPI_CR1_OFFSET);
  if ((cr1 & SPI_CR1_SPE) == 0)
    {
      spi_putreg(priv, STM32_SPI_IFCR_OFFSET, SPI_IFCR_CLEARABLE);
      return;
    }

  status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
  if (!started || (cr1 & SPI_CR1_CSTART) == 0 ||
      (status & SPI_SR_EOT) != 0)
    {
      spi_drain_rx(priv);
      spi_putreg(priv, STM32_SPI_IFCR_OFFSET, SPI_IFCR_CLEARABLE);
      spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
      return;
    }

  spi_putreg(priv, STM32_SPI_CR1_OFFSET,
             SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_CSUSP);
  if (!spi_deadline_start(&deadline, SPI_ABORT_TIMEOUT_US))
    {
      priv->faulted = true;
      return;
    }

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_SUSP) == 0 ||
         (spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_CSTART) != 0)
    {
      if (spi_deadline_expired(&deadline))
        {
          priv->faulted = true;
          spierr("ERROR: SPI%u suspend timed out after transfer failure\n",
                 priv->bus);
          return;
        }
    }

  spi_drain_rx(priv);
  spi_putreg(priv, STM32_SPI_IFCR_OFFSET, SPI_IFCR_CLEARABLE);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
}

static int spi_transfer_chunk(struct stm32_spi_priv_s *priv,
                              const uint8_t *txbuffer, uint8_t *rxbuffer,
                              size_t offset, size_t nframes, bool *started)
{
  struct spi_deadline_s deadline;
  size_t txframes = 0;
  size_t rxframes = 0;
  bool extra_rx = false;
  uint32_t status;
  uint32_t cr1;

  if (!spi_deadline_start(&deadline,
                          spi_transfer_timeout_us(priv, nframes)))
    {
      return -ERANGE;
    }

  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
  spi_putreg(priv, STM32_SPI_IFCR_OFFSET, SPI_IFCR_CLEARABLE);
  spi_putreg(priv, STM32_SPI_CR2_OFFSET,
             (uint32_t)nframes & SPI_CR2_TSIZE_MASK);

  *started = false;
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI | SPI_CR1_SPE);
  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXP) == 0)
    {
      status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
      if ((status & SPI_SR_ERRORS) != 0)
        {
          return -EIO;
        }

      if (spi_deadline_expired(&deadline))
        {
          return -ETIMEDOUT;
        }
    }

  spi_write_txframe(priv, spi_get_txframe(txbuffer, offset, priv->nbits));
  txframes++;
  cr1 = spi_getreg(priv, STM32_SPI_CR1_OFFSET);
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, cr1 | SPI_CR1_CSTART);
  *started = true;

  for (;;)
    {
      status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
      if ((status & SPI_SR_ERRORS) != 0)
        {
          return -EIO;
        }

      while (spi_rx_pending(status))
        {
          uint16_t frame = spi_read_rxframe(priv);

          if (rxframes < nframes)
            {
              spi_put_rxframe(rxbuffer, offset + rxframes, priv->nbits,
                              frame);
            }
          else
            {
              extra_rx = true;
            }

          rxframes++;
          status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
        }

      if (txframes < nframes && (status & SPI_SR_TXP) != 0)
        {
          spi_write_txframe(priv,
                            spi_get_txframe(txbuffer, offset + txframes,
                                            priv->nbits));
          txframes++;
        }

      if ((status & (SPI_SR_EOT | SPI_SR_TXC)) ==
          (SPI_SR_EOT | SPI_SR_TXC))
        {
          if (txframes != nframes)
            {
              return -EIO;
            }

          while (spi_rx_pending(status))
            {
              uint16_t frame = spi_read_rxframe(priv);

              if (rxframes < nframes)
                {
                  spi_put_rxframe(rxbuffer, offset + rxframes, priv->nbits,
                                  frame);
                }
              else
                {
                  extra_rx = true;
                }

              rxframes++;
              status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
            }

          if (spi_deadline_expired(&deadline))
            {
              return -ETIMEDOUT;
            }

          if (rxframes == nframes && !extra_rx)
            {
              up_udelay(1);
              spi_putreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SSI);
              spi_putreg(priv, STM32_SPI_IFCR_OFFSET, SPI_IFCR_CLEARABLE);
              return OK;
            }

          if (rxframes > nframes || extra_rx)
            {
              return -EIO;
            }
        }

      if (spi_deadline_expired(&deadline))
        {
          return -ETIMEDOUT;
        }
    }
}

static int spi_transfer(struct stm32_spi_priv_s *priv,
                        const void *txbuffer, void *rxbuffer, size_t nwords)
{
  const uint8_t *tx = (const uint8_t *)txbuffer;
  uint8_t *rx = (uint8_t *)rxbuffer;
  size_t offset = 0;
  size_t wordsize = priv->nbits / 8;
  bool started;
  int ret = OK;

  if (nwords == 0)
    {
      return OK;
    }

  if (nwords > SIZE_MAX / wordsize)
    {
      ret = -EOVERFLOW;
      goto failed;
    }

  if (priv->faulted)
    {
      ret = -EIO;
      goto failed;
    }

  while (offset < nwords)
    {
      size_t nframes = nwords - offset;

      if (nframes > SPI_CR2_TSIZE_MASK)
        {
          nframes = SPI_CR2_TSIZE_MASK;
        }

      started = false;
      ret = spi_transfer_chunk(priv, tx, rx, offset, nframes, &started);
      if (ret < 0)
        {
          spi_abort_transfer(priv, started);
          goto failed;
        }

      /* SPE is disabled between TSIZE chunks; external chip select is not. */
      offset += nframes;
    }

  priv->last_error = 0;
  return OK;

failed:
  priv->last_error = ret;
  spierr("ERROR: SPI%u transfer failed: %d\n", priv->bus, ret);
  return ret;
}

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t word)
{
  struct stm32_spi_priv_s *priv = (struct stm32_spi_priv_s *)dev;
  uint16_t tx = (uint16_t)word;
  uint16_t rx = 0;
  int ret;

  ret = spi_transfer(priv, &tx, &rx, 1);
  if (ret < 0)
    {
      return UINT32_MAX;
    }

  return priv->nbits == 8 ? (uint8_t)rx : rx;
}

#ifdef CONFIG_SPI_EXCHANGE
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  (void)spi_transfer((struct stm32_spi_priv_s *)dev, txbuffer, rxbuffer,
                     nwords);
}
#else
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  (void)spi_transfer((struct stm32_spi_priv_s *)dev, buffer, NULL, nwords);
}

static void spi_recvblock(struct spi_dev_s *dev, void *buffer, size_t nwords)
{
  (void)spi_transfer((struct stm32_spi_priv_s *)dev, NULL, buffer, nwords);
}
#endif

static struct stm32_spi_priv_s *spi_get_priv(int bus)
{
  switch (bus)
    {
#ifdef CONFIG_STM32_SPI1
      case 1: return &g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
      case 2: return &g_spi2;
#endif
#ifdef CONFIG_STM32_SPI3
      case 3: return &g_spi3;
#endif
#ifdef CONFIG_STM32_SPI4
      case 4: return &g_spi4;
#endif
#ifdef CONFIG_STM32_SPI5
      case 5: return &g_spi5;
#endif
#ifdef CONFIG_STM32_SPI6
      case 6: return &g_spi6;
#endif
      default:
        return NULL;
    }
}

struct spi_dev_s *stm32_spibus_initialize(int bus)
{
  struct stm32_spi_priv_s *priv = spi_get_priv(bus);
  int ret;

  if (priv == NULL)
    {
      spierr("ERROR: SPI bus %d is not enabled\n", bus);
      return NULL;
    }

  ret = spi_initialize(priv);
  if (ret < 0)
    {
      priv->last_error = ret;
      spierr("ERROR: SPI%u initialization failed: %d\n", priv->bus, ret);
      return NULL;
    }

  return &priv->dev;
}

#else

struct spi_dev_s *stm32_spibus_initialize(int bus)
{
  spierr("ERROR: no STM32N6 SPI buses are enabled (bus %d)\n", bus);
  return NULL;
}

#endif /* SPI bus enabled */
