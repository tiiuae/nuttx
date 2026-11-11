/****************************************************************************
 * drivers/net/phy/ti/dp83867.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/ti_phy.h>

#include "phy.h"
#include "dp83867.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dp83867_config_init(FAR struct phy_device_s *phydev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct phy_driver_s g_dp83867_driver =
{
  .name         = "TI DP83867E",
  .phy_id       = DP83867_PHY_ID,
  .phy_id_mask  = DP83867_PHY_ID_MASK,
  .config_init  = dp83867_config_init,
  .config_aneg  = genphy_config_aneg,
  .read_status  = genphy_read_status,
  .config_power = genphy_config_power,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dp83867_modify_ext(FAR struct phy_device_s *phydev,
                             uint16_t regaddr, uint16_t mask, uint16_t set)
{
  uint16_t value;
  int ret;

  /* The PHY requires indirect C22 access even on buses with native C45 ops.
   * Keep address selection and read-modify-write under one bus lock.
   */

  ret = mdio_lock(phydev->bus);
  if (ret < 0)
    {
      return ret;
    }

  ret = mdio_write_nolock(phydev->bus, phydev->mdio_addr,
                          MII_MMDCONTROL, MII_MMD_VEND2);
  if (ret < 0)
    {
      goto out;
    }

  ret = mdio_write_nolock(phydev->bus, phydev->mdio_addr,
                          MII_MMDADDRDATA, regaddr);
  if (ret < 0)
    {
      goto out;
    }

  ret = mdio_write_nolock(phydev->bus, phydev->mdio_addr,
                          MII_MMDCONTROL,
                          MII_MMD_VEND2 | MII_MMDCONTROL_FUNC_NOINCR);
  if (ret < 0)
    {
      goto out;
    }

  ret = mdio_read_nolock(phydev->bus, phydev->mdio_addr,
                         MII_MMDADDRDATA, &value);
  if (ret < 0)
    {
      goto out;
    }

  value = (value & ~mask) | (set & mask);
  ret = mdio_write_nolock(phydev->bus, phydev->mdio_addr,
                          MII_MMDADDRDATA, value);

out:
  mdio_unlock(phydev->bus);
  return ret;
}

static int dp83867_config_init(FAR struct phy_device_s *phydev)
{
  uint16_t mode = DP83867_RGMIICTL_RGMII_EN;
  uint16_t delay;
  int ret;

  switch (phydev->interface)
    {
      case PHY_INTERFACE_RGMII_ID:
        mode |= DP83867_RGMIICTL_TX_DELAY_EN |
                DP83867_RGMIICTL_RX_DELAY_EN;
        break;

      case PHY_INTERFACE_RGMII_TXID:
        mode |= DP83867_RGMIICTL_TX_DELAY_EN;
        break;

      case PHY_INTERFACE_RGMII_RXID:
        mode |= DP83867_RGMIICTL_RX_DELAY_EN;
        break;

      case PHY_INTERFACE_RGMII:
        break;

      default:
        return -EOPNOTSUPP;
    }

  /* SGMII takes priority over RGMII, regardless of RGMII_EN. */

  ret = phy_clear_bits(phydev, DP83867_PHYCR, DP83867_PHYCR_SGMII_EN);
  if (ret < 0)
    {
      return ret;
    }

  /* RX_CTRL strap modes 1/2 otherwise select an internal test mode. */

  ret = dp83867_modify_ext(phydev, DP83867_CFG4,
                           DP83867_CFG4_INT_TST_MODE_1, 0);
  if (ret < 0)
    {
      return ret;
    }

  delay = (CONFIG_NET_PHY_TI_DP83867_TX_DELAY <<
           DP83867_RGMIIDCTL_TX_SHIFT) | CONFIG_NET_PHY_TI_DP83867_RX_DELAY;

  ret = dp83867_modify_ext(phydev, DP83867_RGMIIDCTL,
                           DP83867_RGMIIDCTL_TX_MASK |
                           DP83867_RGMIIDCTL_RX_MASK, delay);
  if (ret < 0)
    {
      return ret;
    }

  ret = dp83867_modify_ext(phydev, DP83867_RGMIICTL,
                           DP83867_RGMIICTL_RGMII_EN |
                           DP83867_RGMIICTL_TX_DELAY_EN |
                           DP83867_RGMIICTL_RX_DELAY_EN, mode);
  return ret < 0 ? ret : genphy_config_init(phydev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dp83867_register(void)
{
  return phy_driver_register(&g_dp83867_driver);
}
