/****************************************************************************
 * drivers/mtd/mx25um51245g.c
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
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI commands *************************************************************/

#define MX25UM_RDID             0x9f
#define MX25UM_RDSR             0x05
#define MX25UM_WREN             0x06
#define MX25UM_RDCR2            0x71
#define MX25UM_WRCR2            0x72
#define MX25UM_RSTEN            0x66
#define MX25UM_RST              0x99

/* Octal commands ***********************************************************/

#define MX25UM_OCTA_STR_READ    0xec13
#define MX25UM_OCTA_DTR_READ    0xee11
#define MX25UM_OCTA_PP          0x12ed
#define MX25UM_OCTA_SE          0x21de
#define MX25UM_OCTA_CE          0x609f
#define MX25UM_OCTA_RDSR        0x05fa
#define MX25UM_OCTA_WREN        0x06f9
#define MX25UM_OCTA_RDCR2       0x718e
#define MX25UM_OCTA_RSTEN       0x6699
#define MX25UM_OCTA_RST         0x9966

/* Registers ****************************************************************/

#define MX25UM_SR_WIP           (1 << 0)
#define MX25UM_SR_WEL           (1 << 1)

#define MX25UM_CR2_MODE         0x00000000
#define MX25UM_CR2_LATENCY      0x00000300
#define MX25UM_CR2_DOPI         0x02
#define MX25UM_CR2_DC_20        0x00

/* Geometry *****************************************************************/

#define MX25UM_PAGE_SHIFT       8
#define MX25UM_PAGE_SIZE        (1 << MX25UM_PAGE_SHIFT)
#define MX25UM_SECTOR_SHIFT     12
#define MX25UM_SECTOR_SIZE      (1 << MX25UM_SECTOR_SHIFT)
#define MX25UM_NSECTORS         16384
#define MX25UM_FLASH_SIZE       (MX25UM_SECTOR_SIZE * MX25UM_NSECTORS)
#define MX25UM_ERASED_STATE     0xff

/* Timing *******************************************************************/

#define MX25UM_RESET_TIME_MS    100
#define MX25UM_PAGE_TIME_MS     1000
#define MX25UM_SUBSECTOR_TIME_MS 400
#define MX25UM_BULK_TIME_MS     460000
#define MX25UM_REGISTER_TIME_MS 40
#define MX25UM_STR_REG_DUMMY    2
#define MX25UM_DTR_REG_DUMMY    4
#define MX25UM_READ_DUMMY       20

/* The generic QSPI API has no DQS control.  DQS must be configured by the
 * controller/board implementation for DOPI transfers.
 */

/* Identification ***********************************************************/

#define MX25UM_MANUFACTURER     0xc2
#define MX25UM_MEMORY_TYPE      0x80
#define MX25UM_MEMORY_DENSITY   0x3a

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum mx25um_protocol_e
{
  MX25UM_PROTOCOL_SPI,
  MX25UM_PROTOCOL_SOPI,
  MX25UM_PROTOCOL_DOPI
};

struct mx25um_dev_s
{
  struct mtd_dev_s mtd;
  FAR struct qspi_dev_s *qspi;
  enum mx25um_protocol_e protocol;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mx25um_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks);
static ssize_t mx25um_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buffer);
static ssize_t mx25um_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buffer);
static ssize_t mx25um_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t mx25um_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buffer);
#endif
static int mx25um_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg);

static int mx25um_lock(FAR struct mx25um_dev_s *priv);
static void mx25um_unlock(FAR struct mx25um_dev_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25um_command
 ****************************************************************************/

static int mx25um_command(FAR struct mx25um_dev_s *priv,
                          uint32_t instruction,
                          enum mx25um_protocol_e protocol)
{
  struct qspi_cmdinfo_s cmd;

  cmd.flags   = protocol == MX25UM_PROTOCOL_SPI ? 0 : QSPICMD_IOCTAL;
  if (protocol == MX25UM_PROTOCOL_DOPI)
    {
      cmd.flags |= QSPICMD_DTR;
    }

  cmd.addrlen = 0;
  cmd.cmd     = instruction;
  cmd.buflen  = 0;
  cmd.addr    = 0;
  cmd.buffer  = NULL;

  return QSPI_COMMAND(priv->qspi, &cmd);
}

/****************************************************************************
 * Name: mx25um_read_status
 ****************************************************************************/

static int mx25um_read_status(FAR struct mx25um_dev_s *priv,
                              FAR uint8_t *status)
{
  struct qspi_meminfo_s mem;
  uint8_t data[2];
  int ret;

  mem.flags   = 0;
  mem.addrlen = 0;
  mem.dummies = 0;
  mem.cmd     = MX25UM_RDSR;
  mem.buflen  = 1;
  mem.addr    = 0;
  mem.buffer  = data;

  if (priv->protocol != MX25UM_PROTOCOL_SPI)
    {
      mem.flags   = QSPIMEM_IOCTAL | QSPIMEM_OCTALIO;
      mem.addrlen = 4;
      mem.dummies = MX25UM_STR_REG_DUMMY;
      mem.cmd     = MX25UM_OCTA_RDSR;

      if (priv->protocol == MX25UM_PROTOCOL_DOPI)
        {
          mem.flags  |= QSPIMEM_DTR;
          mem.dummies = MX25UM_DTR_REG_DUMMY;
          mem.buflen  = 2;
        }
    }

  ret = QSPI_MEMORY(priv->qspi, &mem);
  if (ret == OK)
    {
      *status = data[0];
    }

  return ret;
}

/****************************************************************************
 * Name: mx25um_wait_ready
 ****************************************************************************/

static int mx25um_wait_ready(FAR struct mx25um_dev_s *priv,
                             unsigned int timeout)
{
  uint8_t status;
  int retry;
  int ret;

  for (retry = 0; retry < timeout; retry++)
    {
      ret = mx25um_read_status(priv, &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & MX25UM_SR_WIP) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: mx25um_write_enable
 ****************************************************************************/

static int mx25um_write_enable(FAR struct mx25um_dev_s *priv)
{
  uint8_t status;
  int ret;

  ret = mx25um_wait_ready(priv, MX25UM_PAGE_TIME_MS);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25um_command(priv,
                       priv->protocol == MX25UM_PROTOCOL_SPI ?
                       MX25UM_WREN : MX25UM_OCTA_WREN,
                       priv->protocol);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25um_read_status(priv, &status);
  if (ret < 0)
    {
      return ret;
    }

  return (status & MX25UM_SR_WEL) != 0 ? OK : -EACCES;
}

/****************************************************************************
 * Name: mx25um_reset
 ****************************************************************************/

static int mx25um_reset_protocol(FAR struct mx25um_dev_s *priv,
                                  enum mx25um_protocol_e protocol)
{
  uint32_t rsten;
  uint32_t reset;
  int ret;

  rsten = protocol == MX25UM_PROTOCOL_SPI ?
          MX25UM_RSTEN : MX25UM_OCTA_RSTEN;
  reset = protocol == MX25UM_PROTOCOL_SPI ?
          MX25UM_RST : MX25UM_OCTA_RST;

  ret = mx25um_command(priv, rsten, protocol);
  if (ret == OK)
    {
      ret = mx25um_command(priv, reset, protocol);
    }

  return ret;
}

static int mx25um_reset(FAR struct mx25um_dev_s *priv)
{
  /* Send reset in every possible protocol.  A frame in the wrong protocol
   * is ignored by the flash, while the matching frame returns it to SPI.
   * The following JEDEC-ID read is the authoritative verification.
   */

  mx25um_reset_protocol(priv, MX25UM_PROTOCOL_SPI);
  mx25um_reset_protocol(priv, MX25UM_PROTOCOL_SOPI);
  mx25um_reset_protocol(priv, MX25UM_PROTOCOL_DOPI);

  priv->protocol = MX25UM_PROTOCOL_SPI;
  up_mdelay(MX25UM_RESET_TIME_MS);
  return OK;
}

/****************************************************************************
 * Name: mx25um_read_id
 ****************************************************************************/

static int mx25um_read_id(FAR struct mx25um_dev_s *priv)
{
  uint8_t id[3];
  struct qspi_cmdinfo_s cmd;
  int ret;

  cmd.flags   = QSPICMD_READDATA;
  cmd.addrlen = 0;
  cmd.cmd     = MX25UM_RDID;
  cmd.buflen  = sizeof(id);
  cmd.addr    = 0;
  cmd.buffer  = id;

  ret = QSPI_COMMAND(priv->qspi, &cmd);
  if (ret < 0)
    {
      return ret;
    }

  if (id[0] != MX25UM_MANUFACTURER || id[1] != MX25UM_MEMORY_TYPE ||
      id[2] != MX25UM_MEMORY_DENSITY)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: mx25um_write_cr2
 ****************************************************************************/

static int mx25um_write_cr2(FAR struct mx25um_dev_s *priv,
                            uint32_t address, uint8_t value)
{
  struct qspi_cmdinfo_s cmd;
  int ret;

  cmd.flags   = QSPICMD_ADDRESS | QSPICMD_WRITEDATA;
  cmd.addrlen = 4;
  cmd.cmd     = MX25UM_WRCR2;
  cmd.buflen  = 1;
  cmd.addr    = address;
  cmd.buffer  = &value;

  ret = mx25um_write_enable(priv);
  return ret < 0 ? ret : QSPI_COMMAND(priv->qspi, &cmd);
}

/****************************************************************************
 * Name: mx25um_read_cr2
 ****************************************************************************/

static int mx25um_read_cr2(FAR struct mx25um_dev_s *priv,
                           uint32_t address, FAR uint8_t *value)
{
  struct qspi_meminfo_s mem;
  uint8_t data[2];
  int ret;

  mem.flags   = 0;
  mem.addrlen = 4;
  mem.dummies = 0;
  mem.cmd     = MX25UM_RDCR2;
  mem.buflen  = 1;
  mem.addr    = address;
  mem.buffer  = data;

  if (priv->protocol != MX25UM_PROTOCOL_SPI)
    {
      mem.flags   = QSPIMEM_IOCTAL | QSPIMEM_OCTALIO;
      mem.dummies = MX25UM_STR_REG_DUMMY;
      mem.cmd     = MX25UM_OCTA_RDCR2;

      if (priv->protocol == MX25UM_PROTOCOL_DOPI)
        {
          mem.flags  |= QSPIMEM_DTR;
          mem.dummies = MX25UM_DTR_REG_DUMMY;
          mem.buflen  = 2;
        }
    }

  ret = QSPI_MEMORY(priv->qspi, &mem);
  if (ret == OK)
    {
      *value = data[0];
    }

  return ret;
}

/****************************************************************************
 * Name: mx25um_enter_dopi
 ****************************************************************************/

static int mx25um_enter_dopi(FAR struct mx25um_dev_s *priv)
{
  uint8_t value;
  int ret;

  ret = mx25um_write_cr2(priv, MX25UM_CR2_LATENCY, MX25UM_CR2_DC_20);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25um_wait_ready(priv, MX25UM_REGISTER_TIME_MS);
  if (ret < 0)
    {
      return ret;
    }

  ret = mx25um_write_cr2(priv, MX25UM_CR2_MODE, MX25UM_CR2_DOPI);
  if (ret < 0)
    {
      return ret;
    }

  priv->protocol = MX25UM_PROTOCOL_DOPI;
  up_mdelay(MX25UM_REGISTER_TIME_MS);

  ret = mx25um_wait_ready(priv, MX25UM_REGISTER_TIME_MS);
  if (ret == OK)
    {
      ret = mx25um_read_cr2(priv, MX25UM_CR2_MODE, &value);
    }

  if (ret == OK && value != MX25UM_CR2_DOPI)
    {
      ret = -EIO;
    }

  if (ret == OK)
    {
      ret = mx25um_read_cr2(priv, MX25UM_CR2_LATENCY, &value);
    }

  if (ret == OK && value != MX25UM_CR2_DC_20)
    {
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: mx25um_lock
 ****************************************************************************/

static int mx25um_lock(FAR struct mx25um_dev_s *priv)
{
  int ret;

  ret = QSPI_LOCK(priv->qspi, true);
  if (ret >= 0)
    {
      QSPI_SETMODE(priv->qspi, CONFIG_MX25UM51245G_QSPIMODE);
      QSPI_SETBITS(priv->qspi, 8);
      QSPI_SETFREQUENCY(priv->qspi,
                        CONFIG_MX25UM51245G_QSPI_FREQUENCY);
    }

  return ret;
}

/****************************************************************************
 * Name: mx25um_unlock
 ****************************************************************************/

static void mx25um_unlock(FAR struct mx25um_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, false);
}

/****************************************************************************
 * Name: mx25um_read_data
 ****************************************************************************/

static int mx25um_read_data_transfer(FAR struct mx25um_dev_s *priv,
                                     uint32_t address,
                                     FAR uint8_t *buffer, size_t buflen)
{
  struct qspi_meminfo_s mem;

  mem.flags   = QSPIMEM_IOCTAL | QSPIMEM_OCTALIO | QSPIMEM_DTR;
  mem.addrlen = 4;
  mem.dummies = MX25UM_READ_DUMMY;
  mem.cmd     = MX25UM_OCTA_DTR_READ;
  mem.buflen  = buflen;
  mem.addr    = address;
  mem.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &mem);
}

static int mx25um_read_data(FAR struct mx25um_dev_s *priv,
                            uint32_t address, FAR uint8_t *buffer,
                            size_t buflen)
{
  uint8_t data[2];
  size_t evenlen;
  int ret;

  if (buflen == 0)
    {
      return OK;
    }

  /* DOPI array reads require an even start address.  Keep every transfer an
   * even number of bytes as well so controllers can clock complete words.
   */

  if ((address & 1) != 0)
    {
      ret = mx25um_read_data_transfer(priv, address - 1, data, sizeof(data));
      if (ret < 0)
        {
          return ret;
        }

      *buffer++ = data[1];
      address++;
      buflen--;
    }

  evenlen = buflen & ~(size_t)1;
  if (evenlen > 0)
    {
      ret = mx25um_read_data_transfer(priv, address, buffer, evenlen);
      if (ret < 0)
        {
          return ret;
        }

      address += evenlen;
      buffer  += evenlen;
      buflen  -= evenlen;
    }

  if (buflen != 0)
    {
      ret = mx25um_read_data_transfer(priv, address, data, sizeof(data));
      if (ret < 0)
        {
          return ret;
        }

      *buffer = data[0];
    }

  return OK;
}

/****************************************************************************
 * Name: mx25um_page_program
 ****************************************************************************/

static int mx25um_page_program(FAR struct mx25um_dev_s *priv,
                               uint32_t address,
                               FAR const uint8_t *buffer, size_t buflen)
{
  struct qspi_meminfo_s mem;
  uint8_t data[MX25UM_PAGE_SIZE];
  FAR const uint8_t *xbuffer = buffer;
  uint32_t xaddress = address;
  size_t leading;
  size_t xferlen = buflen;
  int ret;

  /* DOPI page program requires an even address and an even byte count.  An
   * erased-state padding byte does not alter the adjacent flash location.
   */

  if ((address & 1) != 0 || (buflen & 1) != 0)
    {
      leading = address & 1;
      xaddress -= leading;
      xferlen += leading;
      if ((xferlen & 1) != 0)
        {
          xferlen++;
        }

      DEBUGASSERT(xferlen <= sizeof(data));
      memset(data, MX25UM_ERASED_STATE, xferlen);
      memcpy(data + leading, buffer, buflen);
      xbuffer = data;
    }

  mem.flags   = QSPIMEM_WRITE | QSPIMEM_IOCTAL | QSPIMEM_OCTALIO |
                QSPIMEM_DTR;
  mem.addrlen = 4;
  mem.dummies = 0;
  mem.cmd     = MX25UM_OCTA_PP;
  mem.buflen  = xferlen;
  mem.addr    = xaddress;
  mem.buffer  = (FAR void *)xbuffer;

  ret = mx25um_write_enable(priv);
  if (ret == OK)
    {
      ret = QSPI_MEMORY(priv->qspi, &mem);
    }

  return ret < 0 ? ret : mx25um_wait_ready(priv, MX25UM_PAGE_TIME_MS);
}

/****************************************************************************
 * Name: mx25um_sector_erase
 ****************************************************************************/

static int mx25um_sector_erase(FAR struct mx25um_dev_s *priv,
                               uint32_t address)
{
  struct qspi_cmdinfo_s cmd;
  int ret;

  cmd.flags   = QSPICMD_ADDRESS | QSPICMD_IOCTAL | QSPICMD_OCTALIO |
                QSPICMD_DTR;
  cmd.addrlen = 4;
  cmd.cmd     = MX25UM_OCTA_SE;
  cmd.buflen  = 0;
  cmd.addr    = address;
  cmd.buffer  = NULL;

  ret = mx25um_write_enable(priv);
  if (ret == OK)
    {
      ret = QSPI_COMMAND(priv->qspi, &cmd);
    }

  return ret < 0 ? ret :
         mx25um_wait_ready(priv, MX25UM_SUBSECTOR_TIME_MS);
}

/****************************************************************************
 * Name: mx25um_chip_erase
 ****************************************************************************/

static int mx25um_chip_erase(FAR struct mx25um_dev_s *priv)
{
  int ret;

  ret = mx25um_write_enable(priv);
  if (ret == OK)
    {
      ret = mx25um_command(priv, MX25UM_OCTA_CE,
                           MX25UM_PROTOCOL_DOPI);
    }

  return ret < 0 ? ret : mx25um_wait_ready(priv, MX25UM_BULK_TIME_MS);
}

/****************************************************************************
 * Name: mx25um_erase
 ****************************************************************************/

static int mx25um_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks)
{
  FAR struct mx25um_dev_s *priv = (FAR struct mx25um_dev_s *)dev;
  size_t erased;
  int ret;

  if (startblock < 0 || startblock >= MX25UM_NSECTORS ||
      nblocks > MX25UM_NSECTORS - startblock)
    {
      return -EINVAL;
    }

  ret = mx25um_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (erased = 0; ret == OK && erased < nblocks; erased++)
    {
      ret = mx25um_sector_erase(priv,
                                (startblock + erased) <<
                                MX25UM_SECTOR_SHIFT);
    }

  mx25um_unlock(priv);
  return ret < 0 ? ret : (int)erased;
}

/****************************************************************************
 * Name: mx25um_bread
 ****************************************************************************/

static ssize_t mx25um_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t ret;

  ret = mx25um_read(dev, startblock << MX25UM_PAGE_SHIFT,
                    nblocks << MX25UM_PAGE_SHIFT, buffer);
  return ret < 0 ? ret : (ssize_t)nblocks;
}

/****************************************************************************
 * Name: mx25um_bwrite
 ****************************************************************************/

static ssize_t mx25um_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct mx25um_dev_s *priv = (FAR struct mx25um_dev_s *)dev;
  size_t written;
  int ret;

  if (startblock < 0 || startblock >=
      (MX25UM_FLASH_SIZE >> MX25UM_PAGE_SHIFT) ||
      nblocks > (MX25UM_FLASH_SIZE >> MX25UM_PAGE_SHIFT) - startblock)
    {
      return -EINVAL;
    }

  ret = mx25um_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (written = 0; ret == OK && written < nblocks; written++)
    {
      ret = mx25um_page_program(priv,
                                (startblock + written) <<
                                MX25UM_PAGE_SHIFT,
                                buffer + (written << MX25UM_PAGE_SHIFT),
                                MX25UM_PAGE_SIZE);
    }

  mx25um_unlock(priv);
  return ret < 0 ? ret : (ssize_t)written;
}

/****************************************************************************
 * Name: mx25um_read
 ****************************************************************************/

static ssize_t mx25um_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct mx25um_dev_s *priv = (FAR struct mx25um_dev_s *)dev;
  int ret;

  if (offset < 0 || offset >= MX25UM_FLASH_SIZE ||
      nbytes > MX25UM_FLASH_SIZE - offset)
    {
      return -EINVAL;
    }

  ret = mx25um_lock(priv);
  if (ret == OK)
    {
      ret = mx25um_read_data(priv, offset, buffer, nbytes);
      mx25um_unlock(priv);
    }

  return ret < 0 ? ret : (ssize_t)nbytes;
}

#ifdef CONFIG_MTD_BYTE_WRITE
/****************************************************************************
 * Name: mx25um_write
 ****************************************************************************/

static ssize_t mx25um_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct mx25um_dev_s *priv = (FAR struct mx25um_dev_s *)dev;
  size_t pagesize;
  size_t written;
  int ret;

  if (offset < 0 || offset >= MX25UM_FLASH_SIZE ||
      nbytes > MX25UM_FLASH_SIZE - offset)
    {
      return -EINVAL;
    }

  ret = mx25um_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (written = 0; ret == OK && written < nbytes; written += pagesize)
    {
      pagesize = MX25UM_PAGE_SIZE - ((offset + written) &
                                     (MX25UM_PAGE_SIZE - 1));
      if (pagesize > nbytes - written)
        {
          pagesize = nbytes - written;
        }

      ret = mx25um_page_program(priv, offset + written,
                                buffer + written, pagesize);
    }

  mx25um_unlock(priv);
  return ret < 0 ? ret : (ssize_t)written;
}
#endif

/****************************************************************************
 * Name: mx25um_ioctl
 ****************************************************************************/

static int mx25um_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct mx25um_dev_s *priv = (FAR struct mx25um_dev_s *)dev;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)(uintptr_t)arg;

          if (geo == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              geo->blocksize    = MX25UM_PAGE_SIZE;
              geo->erasesize    = MX25UM_SECTOR_SIZE;
              geo->neraseblocks = MX25UM_NSECTORS;
              strlcpy(geo->model, "MX25UM51245G", sizeof(geo->model));
              ret = OK;
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)(uintptr_t)arg;

          if (info == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              info->numsectors  = MX25UM_FLASH_SIZE / MX25UM_PAGE_SIZE;
              info->sectorsize  = MX25UM_PAGE_SIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = mx25um_lock(priv);
        if (ret == OK)
          {
            ret = mx25um_chip_erase(priv);
            mx25um_unlock(priv);
          }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *state = (FAR uint8_t *)(uintptr_t)arg;

          if (state == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              *state = MX25UM_ERASED_STATE;
              ret = OK;
            }
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25um51245g_initialize
 *
 * Description:
 *   Bind an MX25UM51245G flash memory to an XSPI interface.
 *
 * Input Parameters:
 *   qspi - QSPI interface
 *
 * Returned Value:
 *   An MTD device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *
mx25um51245g_initialize(FAR struct qspi_dev_s *qspi)
{
  FAR struct mx25um_dev_s *priv;
  int ret;

  if (qspi == NULL)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->mtd.erase  = mx25um_erase;
  priv->mtd.bread  = mx25um_bread;
  priv->mtd.bwrite = mx25um_bwrite;
  priv->mtd.read   = mx25um_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = mx25um_write;
#endif
  priv->mtd.ioctl  = mx25um_ioctl;
  priv->mtd.name   = "mx25um51245g";
  priv->qspi       = qspi;
  priv->protocol   = MX25UM_PROTOCOL_SPI;

  ret = mx25um_lock(priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  ret = mx25um_reset(priv);
  if (ret == OK)
    {
      ret = mx25um_read_id(priv);
    }

  if (ret == OK)
    {
      ret = mx25um_enter_dopi(priv);
    }

  mx25um_unlock(priv);

  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return &priv->mtd;
}
