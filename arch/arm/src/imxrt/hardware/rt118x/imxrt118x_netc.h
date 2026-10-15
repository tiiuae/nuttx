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

/* RT1180RM rev. 10, chapter 53; checked against MIMXRT1189_cm7.h.
 * ECAM command/control registers are 16-bit, NOT 32-bit registers.
 */

#define IMXRT_NETC_PF1_BASE        0x60001000ul
#define IMXRT_NETC_PF2_BASE        0x60002000ul
#define IMXRT_NETC_PF3_BASE        0x60003000ul
#define IMXRT_NETC_PF4_BASE        0x60004000ul
#define IMXRT_NETC_IERB_BASE       0x60800000ul
#define IMXRT_NETC_PRB_BASE        0x60900000ul
#define IMXRT_NETC_ENETC1_MSIX     0x60c00000ul

#define NETC_PCI_CMD               0x0004
#define NETC_PCI_DEVCTL            0x0048
#define NETC_PCI_MSIXCTL           0x0082
#define NETC_PCI_CMD_ENABLE        0x0006
#define NETC_PCI_FLR               0x8000
#define NETC_PCI_MSIX_ENABLE       0x8000
#define NETC_PCI_MSIX_MASK         0x4000

#define NETC_PRB_NETCRR            0x0100
#define NETC_PRB_NETCSR            0x0104
#define NETC_PRB_SR                (1u << 0)
#define NETC_PRB_LOCK              (1u << 1)
#define NETC_PRB_ERROR             (1u << 0)
#define NETC_PRB_STATE             (1u << 1)
#define NETC_IERB_CLKFR            0x0178
#define NETC_IERB_CLKCR            0x017c
#define NETC_IERB_RCMSIAMQR        0x020c
#define NETC_IERB_MSI_AMQ_MASK     (7u << 27)
#define NETC_IERB_MSI_AMQ_INIT     (1u << 27)
#define NETC_IERB_LINKBCR(n)       (0x1010 + (n) * 0x40)
#define NETC_IERB_PHYADDR_MASK     (31u << 8)

#define NETC_SI_SIMR               0x0000
#define NETC_SI_SIRBGCR            0x0038
#define NETC_SI_SIBCAR             0x0040
#define NETC_SI_SIIPVBDRMR0        0x0150
#define NETC_SI_SITXIDR0           0x0a18
#define NETC_SI_SIRXIDR0           0x0a28
#define NETC_SI_SIMSITRVR(n)       (0x0b00 + (n) * 4)
#define NETC_SI_SIMSIRRVR(n)       (0x0b80 + (n) * 4)
#define NETC_SI_EN                (1u << 31)
#define NETC_SI_TX_FRAME(n)        (1u << ((n) + 16))
#define NETC_SI_RX_RING(n)         (1u << (n))

#define NETC_TX_BASE(n)            (0x8000 + (n) * 0x200)
#define NETC_RX_BASE(n)            (0x8100 + (n) * 0x200)
#define NETC_BDR_MR               0x00
#define NETC_BDR_SR               0x04
#define NETC_RX_BSR               0x08
#define NETC_RX_CIR               0x0c
#define NETC_BDR_BAR0             0x10
#define NETC_BDR_BAR1             0x14
#define NETC_BDR_PIR              0x18
#define NETC_TX_CIR               0x1c
#define NETC_BDR_LENR             0x20
#define NETC_BDR_IER              0xa0
#define NETC_BDR_ICR0             0xa8
#define NETC_BDR_ICR1             0xac
#define NETC_BDR_EN               (1u << 31)
#define NETC_BDR_SBE              (1u << 16)
#define NETC_TX_FRAME_IE          (1u << 1)
#define NETC_RX_THRESHOLD_IE      (1u << 0)
#define NETC_RX_LOSSY             (1u << 4)
#define NETC_RX_COALESCE          (1u << 31)
#define NETC_RX_THRESHOLD         1u

#define NETC_ENETC_PMR            0x0010
#define NETC_ENETC_PARCSCR        0x009c
#define NETC_ENETC_PSIPMMR        0x0200
#define NETC_ENETC_PSIPVMR        0x0204
#define NETC_ENETC_PRIO2TCMR0     0x0380
#define NETC_ENETC_SMHRBDRMR(n)   (0x0880 + ((n) - 1) * 4)
#define NETC_ENETC_PSIPMAR0       0x2000
#define NETC_ENETC_PSIPMAR1       0x2004
#define NETC_ENETC_PSICFGR0       0x2010
#define NETC_ENETC_PSICFGR2       0x2018
#define NETC_ENETC_SI0EN          (1u << 16)
#define NETC_ENETC_UCPROMISC      (1u << 0)
#define NETC_ENETC_MCPROMISC      (1u << 16)
#define NETC_ENETC_VLANPROMISC    (1u << 0)
#define NETC_ENETC_VLANUNTAGGED   (1u << 16)
#define NETC_ENETC_CHECKSUM_OFF   3u
#define NETC_PORT_POR            0x0100
#define NETC_PORT_TXDIS           (1u << 0)
#define NETC_PORT_RXDIS           (1u << 1)
#define NETC_PSEUDO_PPMCR         0x0010
#define NETC_PSEUDO_TXPAD         (1u << 16)

/* The MDIO helper base is the address of EMDIO_CFG or PEMDIOCR.
 * The full EMDIO block base is also accepted for board convenience.
 */

#define IMXRT_NETC_EMDIO          (IMXRT_EMDIO_BASE + 0x1c00)
#define IMXRT_NETC_PEMDIO(n)      (0x60a05c00ul + (n) * 0x4000)
#define NETC_MDIO_CFG             0x00
#define NETC_MDIO_CTL             0x04
#define NETC_MDIO_DATA            0x08
#define NETC_MDIO_BUSY            (1u << 31)
#define NETC_MDIO_READ_ERROR      (1u << 1)
#define NETC_MDIO_ADDR_ERROR      (1u << 28)
#define NETC_MDIO_C45             (1u << 6)
#define NETC_MDIO_DIV(n)          ((n) << 7)
#define NETC_MDIO_HOLD(n)         ((n) << 2)
#define NETC_MDIO_READ            (1u << 15)
#define NETC_MDIO_PHY(n)          ((uint32_t)(n) << 5)
#define NETC_MSGINTR_MSIIR        0x00
#define NETC_MSGINTR_MSIR         0x04

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H */
