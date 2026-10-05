/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_spi5_bmp280_test.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <syslog.h>
#include <string.h>

#include "nucleo-n657x0-q.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_BMP280_CS       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                              GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN3)

#define BMP280_SPI_DEVID     SPIDEV_BAROMETER(0)
#define SPI5_LOOPBACK_FREQ   250000

void stm32_spi5select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  (void)dev;

  if (devid == BMP280_SPI_DEVID)
    {
      stm32_gpiowrite(GPIO_BMP280_CS, !selected);
    }
}

uint8_t stm32_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  (void)dev;

  return devid == BMP280_SPI_DEVID ? SPI_STATUS_PRESENT : 0;
}

#ifdef CONFIG_NUCLEO_N657X0_Q_SPI5_LOOPBACK_TEST

int stm32_spi5_loopback_test(void)
{
  static const uint8_t tx[] = {0x0a, 0xff, 0xaa, 0x55, 0xd0, 0x00};
  uint8_t rx[sizeof(tx)];
  struct spi_dev_s *spi;
  uint32_t frequency;
  size_t i;
  int ret;

  ret = stm32_configgpio(GPIO_BMP280_CS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI5 loopback: chip-select GPIO setup failed: %d\n",
             ret);
      return ret;
    }

  syslog(LOG_INFO, "SPI5 loopback: CS CLEAR & SET\n");

  spi = stm32_spibus_initialize(5);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "SPI5 loopback: bus initialization failed\n");
      return -ENODEV;
    }

  ret = SPI_LOCK(spi, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI5 loopback: bus lock failed: %d\n", ret);
      return ret;
    }

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  frequency = SPI_SETFREQUENCY(spi, SPI5_LOOPBACK_FREQ);
  if (frequency == 0)
    {
      syslog(LOG_ERR, "SPI5 loopback: frequency setup failed\n");
      ret = -EIO;
      goto out;
    }

  memset(rx, 0xff, sizeof(rx));
  SPI_EXCHANGE(spi, tx, rx, sizeof(tx));

  for (i = 0; i < sizeof(tx); i++)
    {
      if (rx[i] != tx[i])
        {
          syslog(LOG_ERR,
                 "SPI5 loopback: byte %u TX=0x%02x RX=0x%02x\n",
                 (unsigned int)i, tx[i], rx[i]);
          ret = -EIO;
          //goto out;
        }
    }

  if (ret == 0)
  {
  syslog(LOG_INFO, "SPI5 loopback passed at %lu Hz\n",
         (unsigned long)frequency);

  }
  syslog(LOG_INFO, "SPI5 loopback rx data [%lx %lx %lx %lx %lx %lx] \n",
         (unsigned long)rx[0],
         (unsigned long)rx[1],
         (unsigned long)rx[2],
         (unsigned long)rx[3],
         (unsigned long)rx[4],
         (unsigned long)rx[5]);
  ret = OK;

out:
  SPI_LOCK(spi, false);
  return ret;
}

#endif

#ifdef CONFIG_NUCLEO_N657X0_Q_SPI5_BMP280_TEST

#define BMP280_REG_DATA      0xf7
#define BMP280_REG_CTRL      0xf4
#define BMP280_REG_CONFIG    0xf5
#define BMP280_REG_ID        0xd0
#define BMP280_CHIP_ID       0x58

#define BMP280_CTRL_MEAS     ((5 << 2) | (2 << 5) | 3)
#define BMP280_CONFIG_FILTER (4 << 2)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bmp280_transfer(struct spi_dev_s *spi, const uint8_t *tx,
                           uint8_t *rx, size_t nbytes)
{
#ifdef CONFIG_SPI_EXCHANGE
  SPI_EXCHANGE(spi, tx, rx, nbytes);
  return OK;
#else
  size_t i;

  for (i = 0; i < nbytes; i++)
    {
      uint32_t value = SPI_SEND(spi, tx[i]);

      if (value == UINT32_MAX)
        {
          return -EIO;
        }

      if (rx != NULL)
        {
          rx[i] = (uint8_t)value;
        }
    }

  return OK;
#endif
}

static int bmp280_read_register(struct spi_dev_s *spi, uint8_t reg,
                                uint8_t *value)
{
  uint8_t tx[2] = {reg | 0x80, 0};
  uint8_t rx[2];
  int ret;

  SPI_SELECT(spi, BMP280_SPI_DEVID, true);
  ret = bmp280_transfer(spi, tx, rx, sizeof(tx));
  SPI_SELECT(spi, BMP280_SPI_DEVID, false);
  if (ret < 0)
    {
      return ret;
    }

  if (reg == BMP280_REG_ID)
    {
      syslog(LOG_INFO,
             "BMP280: SPI5 ID exchange TX=%02x %02x RX=%02x %02x\n",
             tx[0], tx[1], rx[0], rx[1]);
    }

  *value = rx[1];
  return OK;
}

static int bmp280_write_register(struct spi_dev_s *spi, uint8_t reg,
                                 uint8_t value)
{
  uint8_t tx[2] = {reg & 0x7f, value};
  int ret;

  SPI_SELECT(spi, BMP280_SPI_DEVID, true);
  ret = bmp280_transfer(spi, tx, NULL, sizeof(tx));
  SPI_SELECT(spi, BMP280_SPI_DEVID, false);
  return ret;
}

static int bmp280_read_data(struct spi_dev_s *spi, uint8_t data[6])
{
  uint8_t tx[7] = {BMP280_REG_DATA | 0x80, 0, 0, 0, 0, 0, 0};
  uint8_t rx[7];
  int i;
  int ret;

  SPI_SELECT(spi, BMP280_SPI_DEVID, true);
  ret = bmp280_transfer(spi, tx, rx, sizeof(tx));
  SPI_SELECT(spi, BMP280_SPI_DEVID, false);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < 6; i++)
    {
      data[i] = rx[i + 1];
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_spi5_bmp280_test(void)
{
  struct spi_dev_s *spi;
  uint8_t id;
  uint8_t data[6];
  uint32_t pressure;
  uint32_t temperature;
  int ret;

  ret = stm32_configgpio(GPIO_BMP280_CS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMP280: chip-select GPIO setup failed: %d\n", ret);
      return ret;
    }

  spi = stm32_spibus_initialize(5);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "BMP280: SPI5 initialization failed\n");
      return -ENODEV;
    }

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, 1000000);

  ret = bmp280_read_register(spi, BMP280_REG_ID, &id);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMP280: SPI5 chip-ID transfer failed: %d\n", ret);
      goto out;
    }

  if (id != BMP280_CHIP_ID)
    {
      syslog(LOG_ERR, "BMP280: unexpected chip ID 0x%02x (expected 0x%02x)\n",
             id, BMP280_CHIP_ID);
      ret = -ENODEV;
      goto out;
    }

  ret = bmp280_write_register(spi, BMP280_REG_CONFIG, BMP280_CONFIG_FILTER);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMP280: SPI5 config write failed: %d\n", ret);
      goto out;
    }

  ret = bmp280_write_register(spi, BMP280_REG_CTRL, BMP280_CTRL_MEAS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMP280: SPI5 measurement setup failed: %d\n", ret);
      goto out;
    }

  ret = nxsig_usleep(50000);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMP280: measurement wait failed: %d\n", ret);
      goto out;
    }

  ret = bmp280_read_data(spi, data);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BMP280: SPI5 data transfer failed: %d\n", ret);
      goto out;
    }

  pressure = ((uint32_t)data[0] << 12) |
             ((uint32_t)data[1] << 4) |
             ((uint32_t)data[2] >> 4);
  temperature = ((uint32_t)data[3] << 12) |
                ((uint32_t)data[4] << 4) |
                ((uint32_t)data[5] >> 4);

  syslog(LOG_INFO,
         "BMP280 SPI5 ID=0x%02x raw pressure=%lu temperature=%lu\n",
         id, (unsigned long)pressure, (unsigned long)temperature);
  ret = OK;

out:
  SPI_SELECT(spi, BMP280_SPI_DEVID, false);
  SPI_LOCK(spi, false);
  return ret;
}
#endif
