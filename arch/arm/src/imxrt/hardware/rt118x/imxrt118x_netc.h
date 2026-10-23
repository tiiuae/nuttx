/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_netc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "imxrt118x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IMXRT1180RM Rev. 10, chapters 3, 53 and 54. */

#define IMXRT_NETC_PF(n)            (IMXRT_NETC_ECAM_BASE + (n) * 0x1000)
#define IMXRT_NETC_EMDIO_REGS       (IMXRT_EMDIO_BASE + 0x1c00)
#define IMXRT_NETC_PORT_MDIO(n)     (IMXRT_SW0_BASE + 0x5c00 + (n) * 0x4000)
#define IMXRT_NETC_SW_PORT(n)       (IMXRT_SW0_BASE + ((n) + 1) * 0x4000)
#define IMXRT_NETC_SW_MAC(n)        (IMXRT_NETC_SW_PORT(n) + 0x1000)

/* BLK_CTRL_WAKEUP, IMXRT1180RM Rev. 10, section 16.6.1.
 * These protocol encodings differ from NETC PMIFR.
 */

#define IMXRT_NETC_PORT_MISC_CFG    (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + 0x24)
#define NETC_PORT_MISC_RMII_CLK_OUT(n) (1u << (n))
#define IMXRT_NETC_LINK_CFG(n)      (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + \
                                   0x100 + (n) * 4)
#define NETC_LINK_CFG_MII_PROT_MASK 0x0fu
#define NETC_LINK_CFG_MII           0u
#define NETC_LINK_CFG_RMII          1u
#define NETC_LINK_CFG_RGMII         2u
#define NETC_LINK_CFG_REVMII        (1u << 31)

#define NETC_PRB_NETCRR             0x0100
#define NETC_PRB_NETCSR             0x0104
#define NETC_PRB_SR                 (1 << 0)
#define NETC_PRB_LOCK               (1 << 1)
#define NETC_PRB_ERROR              (1 << 0)
#define NETC_PRB_STATE              (1 << 1)
#define NETC_IERB_CLKFR             0x0178
#define NETC_IERB_CLKCR             0x017c
#define NETC_IERB_CLOCK(mhz, ns)     ((mhz) | ((ns) << 16))

/* Nonzero whole-MHz FREQ encoding limits, not device operating limits. */

#define NETC_IERB_CLKCR_MIN_HZ      1000000u
#define NETC_IERB_CLKCR_MAX_HZ      2047000000u

#define NETC_IERB_LINKBCR(n)        (0x1010 + (n) * 0x40)
#define NETC_IERB_PHYADDR_MASK      (31 << 8)
#define NETC_IERB_PHYADDR(n)        ((uint32_t)(n) << 8)

#define NETC_MSGINTR_MSIIR          0x00
#define NETC_MSGINTR_MSIR           0x04
#define NETC_MSIX_ADDRESS           0x00
#define NETC_MSIX_ADDRESS_HI        0x04
#define NETC_MSIX_DATA              0x08
#define NETC_MSIX_CONTROL           0x0c
#define NETC_MSIX_VECTOR_MASK       1

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H */
