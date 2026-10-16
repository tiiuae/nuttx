/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum imxrt_netc_mii_e
{
  IMXRT_NETC_MII = 1,
  IMXRT_NETC_RMII = 3,
  IMXRT_NETC_RGMII = 4
};

enum imxrt_netc_phy_e
{
  IMXRT_NETC_PHY_RTL8201,
  IMXRT_NETC_PHY_RTL8211
};

struct imxrt_netc_port_s
{
  /* EMDIO_CFG/PEMDIOCR address (full EMDIO block base also accepted). */

  uintptr_t mdio;
  uint8_t phyaddr;
  enum imxrt_netc_mii_e mode;
  enum imxrt_netc_phy_e phy;
  bool rxdelay;
  bool txdelay;
};

struct imxrt_netc_config_s
{
  uint8_t portmask;
  uint8_t mac[6];
  struct imxrt_netc_port_s ports[4];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Board setup prepares clocks, access and PHYs before initializing ENETC. */

int imxrt_netc_initialize(const struct imxrt_netc_config_s *config);
int imxrt_netc_boardinitialize(void);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H */
