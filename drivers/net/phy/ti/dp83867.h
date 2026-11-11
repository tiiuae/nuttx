/****************************************************************************
 * drivers/net/phy/ti/dp83867.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __DRIVERS_NET_PHY_TI_DP83867_H
#define __DRIVERS_NET_PHY_TI_DP83867_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DP83867_PHY_ID                  0x2000a231
#define DP83867_PHY_ID_MASK             0xfffffff0

/* Clause 22 registers */

#define DP83867_PHYCR                   0x10
#define DP83867_PHYCR_SGMII_EN          (1 << 11)

/* Extended registers, accessed through MMD 31 using Clause 22 */

#define DP83867_CFG4                    0x0031
#define DP83867_CFG4_INT_TST_MODE_1     (1 << 7)
#define DP83867_RGMIICTL                0x0032
#define DP83867_RGMIICTL_RGMII_EN       (1 << 7)
#define DP83867_RGMIICTL_TX_DELAY_EN    (1 << 1)
#define DP83867_RGMIICTL_RX_DELAY_EN    (1 << 0)
#define DP83867_RGMIIDCTL               0x0086
#define DP83867_RGMIIDCTL_TX_SHIFT      4
#define DP83867_RGMIIDCTL_TX_MASK       (15 << DP83867_RGMIIDCTL_TX_SHIFT)
#define DP83867_RGMIIDCTL_RX_MASK       15

#endif /* __DRIVERS_NET_PHY_TI_DP83867_H */
