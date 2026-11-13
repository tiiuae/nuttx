/****************************************************************************
 * drivers/net/phy/ti/dp83867.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/net/phy/dp83867.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dp83867_modify_ext(FAR const struct phy_desc_s *desc,
                              FAR const struct mdio_ops_s *ops, int mdio,
                              uint16_t regaddr, uint16_t mask, uint16_t set)
{
  uint16_t value;
  int ret;

  /* The PHY requires indirect C22 access even on buses with native C45 ops.
   * Keep address selection and read-modify-write under one bus lock.
   */

  ret = ops->write(mdio, desc->address_lo,
                   MII_MMDCONTROL, 31);
  if (ret < 0)
    {
      return ret;
    }

  ret = ops->write(mdio, desc->address_lo,
                   MII_MMDADDRDATA, regaddr);
  if (ret < 0)
    {
      return ret;
    }

  ret = ops->write(mdio, desc->address_lo,
                   MII_MMDCONTROL,
                   31 | MII_MMDCONTROL_FUNC_NOINCR);
  if (ret < 0)
    {
      return ret;
    }

  ret = ops->read(mdio, desc->address_lo,
                  MII_MMDADDRDATA, &value);
  if (ret < 0)
    {
      return ret;
    }

  value = (value & ~mask) | (set & mask);
  return ops->write(mdio, desc->address_lo,
                    MII_MMDADDRDATA, value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dp83867_phy_reset(FAR const struct phy_desc_s *desc,
                      FAR const struct mdio_ops_s *ops, int mdio)
{
  uint16_t value;
  unsigned int elapsed;
  int ret;

  ret = ops->write(mdio, desc->address_lo, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      return ret;
    }

  for (elapsed = 0; elapsed < DP83867_RESET_TIMEOUT_MS; elapsed++)
    {
      ret = ops->read(mdio, desc->address_lo, MII_MCR, &value);
      if (ret < 0)
        {
          return ret;
        }

      if ((value & MII_MCR_RESET) == 0)
        {
          return OK;
        }

      usleep(1000);
    }

  return -ETIMEDOUT;
}

int dp83867_phy_init(FAR const struct phy_desc_s *desc,
                      FAR const struct mdio_ops_s *ops, int mdio)
{
  uint16_t mode = GMII_DP83867_RGMIICTL_RGMII_EN;
  uint16_t value;
  uint16_t delay;
  int ret;

  if (desc->iftype == PHY_TYPE_RMII)
    {
      mode |= GMII_DP83867_RGMIICTL_TX_DELAY_EN |
              GMII_DP83867_RGMIICTL_RX_DELAY_EN;
    }

  /* SGMII takes priority over RGMII, regardless of RGMII_EN. */

  ret = ops->read(mdio, desc->address_lo,
                  GMII_DP83867_PHYCR, &value);
  if (ret < 0)
    {
      return ret;
    }

  delay &= ~GMII_DP83867_PHYCR_SGMII_EN;

  ret = ops->write(mdio, desc->address_lo,
                   GMII_DP83867_PHYCR, value);
  if (ret < 0)
    {
      return ret;
    }

  /* RX_CTRL strap modes 1/2 otherwise select an internal test mode. */

  ret = dp83867_modify_ext(desc, ops, mdio, GMII_DP83867_CFG4,
                           GMII_DP83867_CFG4_INT_TST_MODE_1, 0);
  if (ret < 0)
    {
      return ret;
    }

  delay = (CONFIG_NET_PHY_TI_DP83867_TX_DELAY <<
           GMII_DP83867_RGMIIDCTL_TX_SHIFT) | CONFIG_NET_PHY_TI_DP83867_RX_DELAY;

  ret = dp83867_modify_ext(desc, ops, mdio, GMII_DP83867_RGMIIDCTL,
                           GMII_DP83867_RGMIIDCTL_TX_MASK |
                           GMII_DP83867_RGMIIDCTL_RX_MASK, delay);
  if (ret < 0)
    {
      return ret;
    }

  return dp83867_modify_ext(desc, ops, mdio, GMII_DP83867_RGMIICTL,
                           GMII_DP83867_RGMIICTL_RGMII_EN |
                           GMII_DP83867_RGMIICTL_TX_DELAY_EN |
                           GMII_DP83867_RGMIICTL_RX_DELAY_EN, mode);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct phy_ops_s g_dp83867_phy_ops =
{
  .phyinit = dp83867_phy_init,
  .phyreset = dp83867_phy_reset,
};
