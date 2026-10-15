/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_netc_switch.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_SWITCH_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_SWITCH_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RT1180 register offsets/fields checked against MIMXRT1189_cm7.h;
 * NTMP 2.0 layouts checked against NXP fsl_netc.h and fsl_netc_hw.c.
 */

#define NETC_SW_BASE              0x60a00000ul
#define NETC_SW_PORT(n)           (NETC_SW_BASE + ((n) + 1) * 0x4000)
#define NETC_SW_ETH(n)            (NETC_SW_PORT(n) + 0x1000)
#define NETC_SW_CBDRMR            0x0800
#define NETC_SW_CBDRSR            0x0804
#define NETC_SW_CBDRBAR0          0x0810
#define NETC_SW_CBDRBAR1          0x0814
#define NETC_SW_CBDRPIR           0x0818
#define NETC_SW_CBDRCIR           0x081c
#define NETC_SW_CBDRLENR          0x0820
#define NETC_SW_VFHTDECR0         0x2010
#define NETC_SW_VFHTDECR1         0x2014
#define NETC_SW_VFHTDECR2         0x2018
#define NETC_SW_FDBHTMCR          0x2024
#define NETC_SW_EN                (1u << 31)
#define NETC_SW_CPU               (1u << 4)
#define NETC_SW_FIXED_FID         (1u << 12)
#define NETC_SW_LEARN_FLOOD       ((2u << 24) | (2u << 27))

#define NETC_SW_PCR               0x0010
#define NETC_SW_PTAR              0x0050
#define NETC_SW_PQOSMR            0x0054
#define NETC_SW_PIPFCR            0x0084
#define NETC_SW_POR               0x0100
#define NETC_SW_PSR               0x0104
#define NETC_SW_PRXSDUOR          0x0108
#define NETC_SW_PTXSDUOR          0x010c
#define NETC_SW_PTGSCR            0x0110
#define NETC_SW_PDGSR             0x0138
#define NETC_SW_PTCTMSDUR(n)      (0x0208 + (n) * 0x20)
#define NETC_SW_PISIDCR           0x0460
#define NETC_SW_PIPV2QMR0         0x0470
#define NETC_SW_STANDARD_TPIDS    0x33u
#define NETC_SW_NULL_STREAM       (0xffffu << 16)
#define NETC_SW_MAXSDU_DISABLE    (1u << 24)
#define NETC_SW_PCTFCR            0x0450
#define NETC_SW_BPCR              0x0500
#define NETC_SW_BPDVR             0x0510
#define NETC_SW_BPSTGSR           0x0520
#define NETC_SW_DISABLE           3u
#define NETC_SW_BUSY              3u
#define NETC_SW_PSPEED_MASK       (0x3fffu << 16)
#define NETC_SW_PSPEED(n)         ((uint32_t)(n) << 16)
#define NETC_SW_VLAN_UNAWARE       (1u << 24)
#define NETC_SW_ACCEPT_TAGS        (15u << 20)
#define NETC_SW_STG_FORWARD        2u
#define NETC_SW_STORE_FORWARD      3u

#define NETC_SW_MAC_CMD           0x0008
#define NETC_SW_MAC_MAXFRM        0x0014
#define NETC_SW_MAC_MINFRM        0x0018
#define NETC_SW_MAC_IEVENT        0x0040
#define NETC_SW_MAC_IFMODE        0x0300
#define NETC_SW_MAC1_OFFSET       0x0400
#define NETC_SW_MAC_TX            (1u << 0)
#define NETC_SW_MAC_RX            (1u << 1)
#define NETC_SW_MAC_PAD           (1u << 15)
#define NETC_SW_MAC_TXEMPTY       (1u << 5)
#define NETC_SW_MAC_RXEMPTY       (1u << 6)
#define NETC_SW_IF_MASK           (7u | (1u << 4) | (1u << 6) | (3u << 13))
#define NETC_SW_IF_10M            (1u << 4)
#define NETC_SW_IF_HALF           (1u << 6)
#define NETC_SW_IF_SSP(n)         ((uint32_t)(n) << 13)

#define NETC_NTMP_FDB             15u
#define NETC_NTMP_DELETE          1u
#define NETC_NTMP_ADDQUERYUPDATE  14u
#define NETC_NTMP_EXACT           (1u << 12)
#define NETC_NTMP_VERSION2        (2u << 24)
#define NETC_NTMP_NPF             (1u << 15)
#define NETC_NTMP_READY           (1u << 31)
#define NETC_NTMP_ERROR_MASK      (0xfffu << 16)

#define NETC_NTMP_FDB_CFGEU           (1u << 0)
#define NETC_NTMP_QUERY_ACTIONS_SHIFT 24
#define NETC_NTMP_QUERY_ENTRY_ID      (1u << NETC_NTMP_QUERY_ACTIONS_SHIFT)

/* Request: 4-byte common header, 32-byte match, 12-byte configuration. */

#define NETC_NTMP_FDB_REQUEST     48u
#define NETC_NTMP_FDB_RESPONSE    36u
#define NETC_NTMP_FDB_CFG_WORD    9u

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_SWITCH_H */
