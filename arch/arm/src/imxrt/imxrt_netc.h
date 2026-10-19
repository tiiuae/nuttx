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
#include <nuttx/net/mii.h>
#include <stdbool.h>
#include <stdint.h>
#include <arch/imxrt/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_NETC_10_FULL     (1 << 0)
#define IMXRT_NETC_100_FULL    (1 << 1)
#define IMXRT_NETC_1000_FULL   (1 << 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum imxrt_netc_mii_e
{
  IMXRT_NETC_MII = 1,
  IMXRT_NETC_GMII,
  IMXRT_NETC_RMII,
  IMXRT_NETC_RGMII
};

/* MDIO endpoints are chip resources, not caller-supplied MMIO addresses.
 * Per-port endpoints require a fixed PHY address. EMDIO permits probing an
 * address range. A locked IERB must contain the selected addresses.
 */

enum imxrt_netc_mdio_e
{
  IMXRT_NETC_EMDIO = 0,
  IMXRT_NETC_PORT0_MDIO,
  IMXRT_NETC_PORT1_MDIO,
  IMXRT_NETC_PORT2_MDIO,
  IMXRT_NETC_PORT3_MDIO
};

struct imxrt_netc_port_s
{
  bool enabled;
  struct phy_desc_s phy;
  enum imxrt_netc_mii_e mode;
  uint8_t linkmodes;             /* Advertised IMXRT_NETC_*_FULL modes */

  /* Optional board PHY setup after identification, before advertisement.
   * May call imxrt_netc_mdio_read/write(). Restore the PHY's default page
   * before returning. Board code owns PHY delays and reset sequencing.
   */

  int (*phyinit)(const struct imxrt_netc_port_s *port, uint8_t phyaddr);
};

struct imxrt_netc_enetc_s
{
  bool enabled;
  uint8_t msgintr;        /* Exclusively reserved MSGINTR channel 1..6 */
  uint8_t mac[6];
  struct imxrt_netc_port_s ports[IMXRT_NETC_SW0_NPORTS];
};

struct imxrt_netc_config_s
{
  struct imxrt_netc_enetc_s enetc[IMXRT_NETC_NENETC];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Configuration is copied. Array indices identify hardware ENETC instances
 * and their external ports; disabled entries are ignored. Supply every
 * requested interface in one call, before shared IERB resources are locked.
 * The board must first establish pinmux, clocks, reset, TRDC permissions
 * and DMA-accessible normal heaps.
 * This driver never resets/unlocks shared NETC infrastructure. Unlocked IERB
 * resources are configured only before any PCI function has been enabled.
 * Only ENETC1 is supported; ENETC0's direct PHY is not implemented.
 */

int imxrt_netc_initialize(const struct imxrt_netc_config_s *config);

/* Task context only, after initialization has enabled the selected MDIO
 * endpoint (including within phyinit). Diagnostic users must serialize
 * multi-transaction PHY paging against link polling (net_lock at runtime).
 */

int imxrt_netc_mdio_read(enum imxrt_netc_mdio_e bus, uint8_t phyaddr,
                         uint8_t reg, uint16_t *value);
int imxrt_netc_mdio_write(enum imxrt_netc_mdio_e bus, uint8_t phyaddr,
                          uint8_t reg, uint16_t value);

#ifdef __cplusplus
}
#endif
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H */
