/****************************************************************************
 * arch/arm/src/stm32n6/stm32_xspi.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * N6 XSPI2 QSPI interface scaffold.  The polling transfer engine is added
 * in the next implementation step; unsupported transactions fail clearly.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/qspi.h>

#include "stm32_xspi.h"

#ifdef CONFIG_STM32_XSPI

struct stm32_xspi_dev_s
{
  struct qspi_dev_s qspi;
  mutex_t lock;
};

static int xspi_lock(FAR struct qspi_dev_s *dev, bool lock);
static uint32_t xspi_setfrequency(FAR struct qspi_dev_s *dev,
                                  uint32_t frequency);
static void xspi_setmode(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode);
static void xspi_setbits(FAR struct qspi_dev_s *dev, int nbits);
static int xspi_command(FAR struct qspi_dev_s *dev,
                        FAR struct qspi_cmdinfo_s *cmdinfo);
static int xspi_memory(FAR struct qspi_dev_s *dev,
                       FAR struct qspi_meminfo_s *meminfo);
static FAR void *xspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen);
static void xspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer);

static const struct qspi_ops_s g_xspi_ops =
{
  .lock         = xspi_lock,
  .setfrequency = xspi_setfrequency,
  .setmode      = xspi_setmode,
  .setbits      = xspi_setbits,
  .command      = xspi_command,
  .memory       = xspi_memory,
  .alloc        = xspi_alloc,
  .free         = xspi_free,
};

static int xspi_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  FAR struct stm32_xspi_dev_s *priv =
    (FAR struct stm32_xspi_dev_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

static uint32_t xspi_setfrequency(FAR struct qspi_dev_s *dev,
                                  uint32_t frequency)
{
  UNUSED(dev);
  UNUSED(frequency);
  return 0;
}

static void xspi_setmode(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  UNUSED(dev);
  UNUSED(mode);
}

static void xspi_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
  UNUSED(dev);
  UNUSED(nbits);
}

static int xspi_command(FAR struct qspi_dev_s *dev,
                        FAR struct qspi_cmdinfo_s *cmdinfo)
{
  UNUSED(dev);
  UNUSED(cmdinfo);
  return -ENOSYS;
}

static int xspi_memory(FAR struct qspi_dev_s *dev,
                       FAR struct qspi_meminfo_s *meminfo)
{
  UNUSED(dev);
  UNUSED(meminfo);
  return -ENOSYS;
}

static FAR void *xspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  UNUSED(dev);
  UNUSED(buflen);
  return NULL;
}

static void xspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  UNUSED(dev);
  UNUSED(buffer);
}

FAR struct qspi_dev_s *stm32_xspi_initialize(int intf)
{
  FAR struct stm32_xspi_dev_s *priv;

  /* XSPI1 and XSPI3 are intentionally not advertised for N6 yet. */

  if (intf != 2)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->qspi.ops = &g_xspi_ops;
  nxmutex_init(&priv->lock);
  return &priv->qspi;
}

#endif /* CONFIG_STM32_XSPI */
