/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc_port.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <nuttx/mutex.h>
#include <nuttx/net/gmii.h>

#include "imxrt_netc_priv.h"
#include "hardware/imxrt_netc.h"
#include "hardware/rt118x/imxrt118x_netc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Runtime lock order: network lock, then MDIO lock. */

static mutex_t g_mdiolock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uintptr_t imxrt_netc_mdiobase(enum imxrt_netc_mdio_e bus)
{
  if (bus == IMXRT_NETC_EMDIO)
    {
      return IMXRT_NETC_EMDIO_REGS;
    }

  if (bus >= IMXRT_NETC_PORT0_MDIO && bus <= IMXRT_NETC_PORT3_MDIO)
    {
      return IMXRT_NETC_PORT_MDIO(bus - IMXRT_NETC_PORT0_MDIO);
    }

  return 0;
}

static int imxrt_netc_mdio(enum imxrt_netc_mdio_e bus, uint8_t phyaddr,
                           uint8_t reg, uint16_t *value, bool read)
{
  uintptr_t base = imxrt_netc_mdiobase(bus);
  uint32_t status;
  int ret;

  if (base == 0 || phyaddr > 31 || reg > 31 || value == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_mdiolock);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0);
  if (ret == OK)
    {
      status = getreg32(base + NETC_MDIO_CFG);
      status &= ~(NETC_MDIO_C45 | NETC_MDIO_READ_ERROR);
      putreg32(status | NETC_MDIO_ADDR_ERROR | NETC_MDIO_COMPLETE,
               base + NETC_MDIO_CFG);
      putreg32(NETC_MDIO_PHY(phyaddr) | reg |
               (read ? NETC_MDIO_READ : 0), base + NETC_MDIO_CTL);
      if (!read)
        {
          putreg32(*value, base + NETC_MDIO_DATA);
        }

      ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0);
      if (ret == OK)
        {
          status = getreg32(base + NETC_MDIO_CFG);
          if ((status & NETC_MDIO_ADDR_ERROR) != 0)
            {
              ret = -EACCES;
            }
          else if (read && (status & NETC_MDIO_READ_ERROR) != 0)
            {
              ret = -ENODEV;
            }
          else if (read)
            {
              *value = getreg32(base + NETC_MDIO_DATA);
            }
        }
    }

  nxmutex_unlock(&g_mdiolock);
  if (ret < 0 && ret != -ENODEV)
    {
      nerr("MDIO bus %u PHY %u register %u: %d\n", bus, phyaddr, reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_mdio_read(enum imxrt_netc_mdio_e bus, uint8_t phyaddr,
                         uint8_t reg, uint16_t *value)
{
  return imxrt_netc_mdio(bus, phyaddr, reg, value, true);
}

int imxrt_netc_mdio_write(enum imxrt_netc_mdio_e bus, uint8_t phyaddr,
                          uint8_t reg, uint16_t value)
{
  return imxrt_netc_mdio(bus, phyaddr, reg, &value, false);
}

int imxrt_netc_port_validate(const struct imxrt_netc_ports_hw_s *hw,
                            const struct imxrt_netc_port_s *config)
{
  uintptr_t base = imxrt_netc_mdiobase(hw->mdio);
  unsigned int last = config->phy.address_high;

  if (last == UINT16_MAX)
    {
      last = config->phy.address_lo;
    }

  if (base == 0 || config->phy.clause != 22 ||
      config->phy.address_lo > last || last > 31 ||
      memchr(config->phy.name, '\0', sizeof(config->phy.name)) == NULL ||
      config->mode < IMXRT_NETC_MII || config->mode > IMXRT_NETC_RGMII ||
      config->linkmodes == 0 ||
      (config->linkmodes & ~(IMXRT_NETC_10_FULL | IMXRT_NETC_100_FULL |
                            IMXRT_NETC_1000_FULL)) != 0 ||
      ((config->linkmodes & IMXRT_NETC_1000_FULL) != 0 &&
       config->mode != IMXRT_NETC_RGMII &&
       config->mode != IMXRT_NETC_GMII) ||
      (config->phy.status != UINT16_MAX &&
       (config->phy.status > 31 || config->phy.speed_mask == 0 ||
        config->phy.duplex == 0 ||
        ((config->linkmodes & IMXRT_NETC_10_FULL) != 0 &&
         (config->phy.mbps10 & ~config->phy.speed_mask) != 0) ||
        ((config->linkmodes & IMXRT_NETC_100_FULL) != 0 &&
         (config->phy.mbps100 & ~config->phy.speed_mask) != 0) ||
        ((config->linkmodes & IMXRT_NETC_1000_FULL) != 0 &&
         (config->phy.mbps1000 & ~config->phy.speed_mask) != 0) ||
        ((config->linkmodes &
          (IMXRT_NETC_10_FULL | IMXRT_NETC_100_FULL)) ==
         (IMXRT_NETC_10_FULL | IMXRT_NETC_100_FULL) &&
         config->phy.mbps10 == config->phy.mbps100) ||
        ((config->linkmodes &
          (IMXRT_NETC_10_FULL | IMXRT_NETC_1000_FULL)) ==
         (IMXRT_NETC_10_FULL | IMXRT_NETC_1000_FULL) &&
         config->phy.mbps10 == config->phy.mbps1000) ||
        ((config->linkmodes &
          (IMXRT_NETC_100_FULL | IMXRT_NETC_1000_FULL)) ==
         (IMXRT_NETC_100_FULL | IMXRT_NETC_1000_FULL) &&
         config->phy.mbps100 == config->phy.mbps1000))))
    {
      return -EINVAL;
    }

  return OK;
}

int imxrt_netc_port_initialize(const struct imxrt_netc_ports_hw_s *hw,
                              const struct imxrt_netc_port_s *config,
                              struct imxrt_netc_link_s *link,
                              uint32_t clock_hz)
{
  uintptr_t base = imxrt_netc_mdiobase(hw->mdio);
  unsigned int last = config->phy.address_high;
  unsigned int address;
  uint32_t divisor;
  uint16_t id1;
  uint16_t id2;
  uint16_t advertise;
  uint16_t control;
  int ret;

  ret = imxrt_netc_port_validate(hw, config);
  if (ret < 0)
    {
      return ret;
    }

  if (last == UINT16_MAX)
    {
      last = config->phy.address_lo;
    }

  /* MDC <= 2.5 MHz: NETC clock / (2 * divisor + 1). */

  divisor = (clock_hz + 4999999) / 5000000;
  if (divisor < 5)
    {
      divisor = 5;
    }

  if (divisor > 511)
    {
      return -ERANGE;
    }

  ret = nxmutex_lock(&g_mdiolock);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0);
  if (ret == OK)
    {
      putreg32(NETC_MDIO_NEG | NETC_MDIO_DIV(divisor) |
               NETC_MDIO_HOLD(2) | NETC_MDIO_ADDR_ERROR | NETC_MDIO_COMPLETE,
               base + NETC_MDIO_CFG);
    }

  nxmutex_unlock(&g_mdiolock);
  if (ret < 0)
    {
      return ret;
    }

  for (address = config->phy.address_lo; address <= last; address++)
    {
      ret = imxrt_netc_mdio_read(hw->mdio, address, MII_PHYID1, &id1);
      if (ret == -ENODEV)
        {
          nerr("PHY %s MDIO bus %u address %u: no response to PHYID1\n",
               config->phy.name, hw->mdio, address);
          continue;
        }

      if (ret < 0)
        {
          return ret;
        }

      ret = imxrt_netc_mdio_read(hw->mdio, address, MII_PHYID2, &id2);
      if (ret < 0)
        {
          nerr("PHY %s MDIO bus %u address %u: PHYID2 read failed: %d\n",
               config->phy.name, hw->mdio, address, ret);
          return ret;
        }

      if (id1 == config->phy.id1 &&
          (id2 & ~MII_PHYID2_REV_MASK) ==
          (config->phy.id2 & ~MII_PHYID2_REV_MASK) &&
          id1 != UINT16_MAX && (id1 != 0 || id2 != 0))
        {
          break;
        }

      nerr("PHY %s MDIO bus %u address %u: ID %04x:%04x, "
           "expected %04x:%04x (revision ignored)\n",
           config->phy.name, hw->mdio, address,
           id1, id2, config->phy.id1, config->phy.id2);
    }

  if (address > last)
    {
      nerr("PHY %s not found on MDIO bus %u\n",
           config->phy.name, hw->mdio);
      return -ENODEV;
    }

  link->phyaddr = address;
  link->speed = 0;
  if (config->phyinit != NULL)
    {
      ret = config->phyinit(config, hw->mdio, address);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = imxrt_netc_mdio_read(hw->mdio, address, MII_MCR, &control);
  if (ret < 0)
    {
      return ret;
    }

  if ((control & MII_MCR_RESET) != 0)
    {
      return -EBUSY;
    }

  advertise = MII_ADVERTISE_CSMA;
  if ((config->linkmodes & IMXRT_NETC_10_FULL) != 0)
    {
      advertise |= MII_ADVERTISE_10BASETXFULL;
    }

  if ((config->linkmodes & IMXRT_NETC_100_FULL) != 0)
    {
      advertise |= MII_ADVERTISE_100BASETXFULL;
    }

  ret = imxrt_netc_mdio_write(hw->mdio, address, MII_ADVERTISE,
                              advertise);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_netc_mdio_read(hw->mdio, address, MII_MSR, &control);
  if (ret < 0)
    {
      return ret;
    }

  if ((control & MII_MSR_ANEGABLE) == 0)
    {
      return -EOPNOTSUPP;
    }

  if ((control & MII_MSR_ESTATEN) != 0)
    {
      advertise = (config->linkmodes & IMXRT_NETC_1000_FULL) != 0 ?
                  GMII_1000BTCR_1000BASETFULL : 0;
      ret = imxrt_netc_mdio_write(hw->mdio, address, GMII_1000BTCR,
                                  advertise);
      if (ret < 0)
        {
          return ret;
        }
    }
  else if ((config->linkmodes & IMXRT_NETC_1000_FULL) != 0)
    {
      return -EOPNOTSUPP;
    }

  return imxrt_netc_mdio_write(hw->mdio, address, MII_MCR,
                               MII_MCR_ANENABLE | MII_MCR_ANRESTART);
}

int imxrt_netc_port_link(const struct imxrt_netc_ports_hw_s *hw,
                        const struct imxrt_netc_port_s *config,
                        struct imxrt_netc_link_s *link, uint16_t *speed)
{
  uint16_t status;
  uint16_t local;
  uint16_t partner;
  uint16_t mode;
  int ret;

  *speed = 0;

  /* BMSR link is latched low; always read twice. */

  ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, MII_MSR, &status);
  if (ret == OK)
    {
      ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, MII_MSR, &status);
    }

  if (ret < 0 || (status & (MII_MSR_LINKSTATUS | MII_MSR_ANEGCOMPLETE)) !=
                (MII_MSR_LINKSTATUS | MII_MSR_ANEGCOMPLETE))
    {
      return ret;
    }

  ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, MII_MCR, &local);
  if (ret < 0)
    {
      return ret;
    }

  if ((local & MII_MCR_ANENABLE) == 0)
    {
      return -EPROTO;
    }

  if (config->phy.status != UINT16_MAX)
    {
      ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, config->phy.status,
                                &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & config->phy.duplex) == 0)
        {
          return -EPROTO;
        }

      mode = status & config->phy.speed_mask;
      if (mode == config->phy.mbps1000 &&
          (config->linkmodes & IMXRT_NETC_1000_FULL) != 0)
        {
          *speed = 1000;
        }
      else if (mode == config->phy.mbps100 &&
               (config->linkmodes & IMXRT_NETC_100_FULL) != 0)
        {
          *speed = 100;
        }
      else if (mode == config->phy.mbps10 &&
               (config->linkmodes & IMXRT_NETC_10_FULL) != 0)
        {
          *speed = 10;
        }
      else
        {
          return -EPROTO;
        }

      return OK;
    }

  if ((config->linkmodes & IMXRT_NETC_1000_FULL) != 0)
    {
      ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, GMII_1000BTCR,
                                &local);
      if (ret == OK)
        {
          ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, GMII_1000BTSR,
                                    &partner);
        }

      if (ret < 0)
        {
          return ret;
        }

      if ((partner & GMII_1000BTSR_MSFAULT) != 0)
        {
          return -EPROTO;
        }

      if ((local & GMII_1000BTCR_1000BASETFULL) != 0 &&
          (partner & GMII_1000BTSR_LP1000BASETFULL) != 0)
        {
          *speed = 1000;
          return OK;
        }
    }

  ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, MII_ADVERTISE, &local);
  if (ret == OK)
    {
      ret = imxrt_netc_mdio_read(hw->mdio, link->phyaddr, MII_LPA, &partner);
    }

  if (ret < 0)
    {
      return ret;
    }

  mode = local & partner;
  if ((mode & MII_ADVERTISE_100BASETXFULL) != 0 &&
      (config->linkmodes & IMXRT_NETC_100_FULL) != 0)
    {
      *speed = 100;
    }
  else if ((mode & MII_ADVERTISE_10BASETXFULL) != 0 &&
           (config->linkmodes & IMXRT_NETC_10_FULL) != 0)
    {
      *speed = 10;
    }
  else
    {
      return -EPROTO;
    }

  return OK;
}

int imxrt_netc_port_mode(uintptr_t port, uintptr_t mac,
                        enum imxrt_netc_mii_e mode, uint16_t speed)
{
  uint32_t interface = mode;
  int ret;

  modifyreg32(mac + NETC_MAC_CMD, NETC_MAC_RX, 0);
  ret = imxrt_netc_wait(mac + NETC_MAC_IEVENT, NETC_MAC_RXEMPTY,
                        NETC_MAC_RXEMPTY);
  if (ret == OK)
    {
      ret = imxrt_netc_wait(port + NETC_PORT_PSR, NETC_PORT_RXBUSY, 0);
    }

  if (ret == OK)
    {
      modifyreg32(port + NETC_PORT_POR, 0, NETC_PORT_DISABLE);
      ret = imxrt_netc_wait(mac + NETC_MAC_IEVENT, NETC_MAC_TXEMPTY,
                            NETC_MAC_TXEMPTY);
    }

  if (ret < 0)
    {
      modifyreg32(port + NETC_PORT_POR, 0, NETC_PORT_DISABLE);
      return ret;
    }

  /* Allow the final minimum frame to leave the MAC before disabling TX. */

  up_udelay(52);
  modifyreg32(mac + NETC_MAC_CMD, NETC_MAC_TX, 0);
  if (speed == 0)
    {
      return OK;
    }

  if (speed == 10)
    {
      interface |= NETC_MAC_IF_10M | NETC_MAC_IF_SSP(1);
    }
  else if (speed == 1000)
    {
      interface |= NETC_MAC_IF_SSP(2);
    }

  modifyreg32(mac + NETC_MAC_IFMODE, NETC_MAC_IF_MASK, interface);
  modifyreg32(port + NETC_PORT_PCR, NETC_PORT_PSPEED_MASK,
              NETC_PORT_PSPEED(speed - 1));
  putreg32(NETC_PACKET_SIZE + 4, mac + NETC_MAC_MAXFRM);
  putreg32(64, mac + NETC_MAC_MINFRM);
  modifyreg32(mac + NETC_MAC_CMD, 0,
              NETC_MAC_TXPAD | NETC_MAC_TX | NETC_MAC_RX);
  modifyreg32(port + NETC_PORT_POR, NETC_PORT_DISABLE, 0);
  return OK;
}

void imxrt_netc_pseudo(uintptr_t port)
{
  unsigned int tc;

  modifyreg32(port + NETC_PORT_PCR, NETC_PORT_PSPEED_MASK,
              NETC_PORT_PSPEED(999));
  putreg32(20, port + NETC_PORT_PRXSDUOR);
  putreg32(20, port + NETC_PORT_PTXSDUOR);
  for (tc = 0; tc < 8; tc++)
    {
      putreg32(NETC_PORT_MAXSDU_DISABLE, port + NETC_PORT_PTCTMSDUR(tc));
    }
}
