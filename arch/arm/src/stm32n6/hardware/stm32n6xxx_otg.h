/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_otg.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_OTG_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_OTG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RM0486 chapter 73. Registers and FIFO ports use 32-bit accesses.
 * STM32_OTG_BASE is supplied by the selected controller.
 */

#define STM32_OTG_NENDPOINTS             9
#define STM32_OTG_FIFO_BYTES             4096
#define STM32_OTG_FIFO_WORDS             1024

/* Global registers: GRXSTSR peeks at status; GRXSTSP pops it. */

#define STM32_OTG_GOTGCTL_OFFSET         0x0000
#define STM32_OTG_GOTGINT_OFFSET         0x0004
#define STM32_OTG_GAHBCFG_OFFSET         0x0008
#define STM32_OTG_GUSBCFG_OFFSET         0x000c
#define STM32_OTG_GRSTCTL_OFFSET         0x0010
#define STM32_OTG_GINTSTS_OFFSET         0x0014
#define STM32_OTG_GINTMSK_OFFSET         0x0018
#define STM32_OTG_GRXSTSR_OFFSET         0x001c
#define STM32_OTG_GRXSTSP_OFFSET         0x0020
#define STM32_OTG_GRXFSIZ_OFFSET         0x0024
#define STM32_OTG_DIEPTXF0_OFFSET        0x0028
#define STM32_OTG_GCCFG_OFFSET           0x0038
#define STM32_OTG_CID_OFFSET             0x003c
#define STM32_OTG_GLPMCFG_OFFSET         0x0054
#define STM32_OTG_DIEPTXF_OFFSET(n)      (0x0104 + 4 * ((n) - 1))

/* Device registers; endpoint indices are 0..8, TX FIFO size indices 1..8. */

#define STM32_OTG_DCFG_OFFSET            0x0800
#define STM32_OTG_DCTL_OFFSET            0x0804
#define STM32_OTG_DSTS_OFFSET            0x0808
#define STM32_OTG_DIEPMSK_OFFSET         0x0810
#define STM32_OTG_DOEPMSK_OFFSET         0x0814
#define STM32_OTG_DAINT_OFFSET           0x0818
#define STM32_OTG_DAINTMSK_OFFSET        0x081c
#define STM32_OTG_DTHRCTL_OFFSET         0x0830
#define STM32_OTG_DIEPEMPMSK_OFFSET      0x0834
#define STM32_OTG_DIEPCTL_OFFSET(n)      (0x0900 + 0x20 * (n))
#define STM32_OTG_DIEPINT_OFFSET(n)      (0x0908 + 0x20 * (n))
#define STM32_OTG_DIEPTSIZ_OFFSET(n)     (0x0910 + 0x20 * (n))
#define STM32_OTG_DIEPDMA_OFFSET(n)      (0x0914 + 0x20 * (n))
#define STM32_OTG_DTXFSTS_OFFSET(n)      (0x0918 + 0x20 * (n))
#define STM32_OTG_DOEPCTL_OFFSET(n)      (0x0b00 + 0x20 * (n))
#define STM32_OTG_DOEPINT_OFFSET(n)      (0x0b08 + 0x20 * (n))
#define STM32_OTG_DOEPTSIZ_OFFSET(n)     (0x0b10 + 0x20 * (n))
#define STM32_OTG_DOEPDMA_OFFSET(n)      (0x0b14 + 0x20 * (n))
#define STM32_OTG_PCGCCTL_OFFSET         0x0e00
#define STM32_OTG_PCGCCTL1_OFFSET        0x0e04
#define STM32_OTG_DFIFO_OFFSET(n)        (0x1000 + 0x1000 * (n))

/* Register addresses */

#define STM32_OTG_GOTGCTL     (STM32_OTG_BASE + STM32_OTG_GOTGCTL_OFFSET)
#define STM32_OTG_GOTGINT     (STM32_OTG_BASE + STM32_OTG_GOTGINT_OFFSET)
#define STM32_OTG_GAHBCFG     (STM32_OTG_BASE + STM32_OTG_GAHBCFG_OFFSET)
#define STM32_OTG_GUSBCFG     (STM32_OTG_BASE + STM32_OTG_GUSBCFG_OFFSET)
#define STM32_OTG_GRSTCTL     (STM32_OTG_BASE + STM32_OTG_GRSTCTL_OFFSET)
#define STM32_OTG_GINTSTS     (STM32_OTG_BASE + STM32_OTG_GINTSTS_OFFSET)
#define STM32_OTG_GINTMSK     (STM32_OTG_BASE + STM32_OTG_GINTMSK_OFFSET)
#define STM32_OTG_GRXSTSR     (STM32_OTG_BASE + STM32_OTG_GRXSTSR_OFFSET)
#define STM32_OTG_GRXSTSP     (STM32_OTG_BASE + STM32_OTG_GRXSTSP_OFFSET)
#define STM32_OTG_GRXFSIZ     (STM32_OTG_BASE + STM32_OTG_GRXFSIZ_OFFSET)
#define STM32_OTG_DIEPTXF0    (STM32_OTG_BASE + STM32_OTG_DIEPTXF0_OFFSET)
#define STM32_OTG_GCCFG       (STM32_OTG_BASE + STM32_OTG_GCCFG_OFFSET)
#define STM32_OTG_CID         (STM32_OTG_BASE + STM32_OTG_CID_OFFSET)
#define STM32_OTG_GLPMCFG     (STM32_OTG_BASE + STM32_OTG_GLPMCFG_OFFSET)
#define STM32_OTG_DIEPTXF(n)  (STM32_OTG_BASE + STM32_OTG_DIEPTXF_OFFSET(n))
#define STM32_OTG_DCTL        (STM32_OTG_BASE + STM32_OTG_DCTL_OFFSET)
#define STM32_OTG_DCFG        (STM32_OTG_BASE + STM32_OTG_DCFG_OFFSET)
#define STM32_OTG_DSTS        (STM32_OTG_BASE + STM32_OTG_DSTS_OFFSET)
#define STM32_OTG_DIEPMSK     (STM32_OTG_BASE + STM32_OTG_DIEPMSK_OFFSET)
#define STM32_OTG_DOEPMSK     (STM32_OTG_BASE + STM32_OTG_DOEPMSK_OFFSET)
#define STM32_OTG_DAINT       (STM32_OTG_BASE + STM32_OTG_DAINT_OFFSET)
#define STM32_OTG_DAINTMSK    (STM32_OTG_BASE + STM32_OTG_DAINTMSK_OFFSET)
#define STM32_OTG_DTHRCTL     (STM32_OTG_BASE + STM32_OTG_DTHRCTL_OFFSET)
#define STM32_OTG_DIEPEMPMSK  (STM32_OTG_BASE + STM32_OTG_DIEPEMPMSK_OFFSET)
#define STM32_OTG_DIEPCTL(n)  (STM32_OTG_BASE + STM32_OTG_DIEPCTL_OFFSET(n))
#define STM32_OTG_DIEPINT(n)  (STM32_OTG_BASE + STM32_OTG_DIEPINT_OFFSET(n))
#define STM32_OTG_DIEPTSIZ(n) (STM32_OTG_BASE + STM32_OTG_DIEPTSIZ_OFFSET(n))
#define STM32_OTG_DIEPDMA(n)  (STM32_OTG_BASE + STM32_OTG_DIEPDMA_OFFSET(n))
#define STM32_OTG_DTXFSTS(n)  (STM32_OTG_BASE + STM32_OTG_DTXFSTS_OFFSET(n))
#define STM32_OTG_DOEPCTL(n)  (STM32_OTG_BASE + STM32_OTG_DOEPCTL_OFFSET(n))
#define STM32_OTG_DOEPINT(n)  (STM32_OTG_BASE + STM32_OTG_DOEPINT_OFFSET(n))
#define STM32_OTG_DOEPTSIZ(n) (STM32_OTG_BASE + STM32_OTG_DOEPTSIZ_OFFSET(n))
#define STM32_OTG_DOEPDMA(n)  (STM32_OTG_BASE + STM32_OTG_DOEPDMA_OFFSET(n))
#define STM32_OTG_PCGCCTL     (STM32_OTG_BASE + STM32_OTG_PCGCCTL_OFFSET)
#define STM32_OTG_PCGCCTL1    (STM32_OTG_BASE + STM32_OTG_PCGCCTL1_OFFSET)
#define STM32_OTG_DFIFO(n)    (STM32_OTG_BASE + STM32_OTG_DFIFO_OFFSET(n))

/* GOTGCTL: read-only session/mode status */

#define OTG_GOTGCTL_CIDSTS              (1u << 16)
#define OTG_GOTGCTL_ASVLD               (1u << 18)
#define OTG_GOTGCTL_BSVLD               (1u << 19)
#define OTG_GOTGCTL_CURMOD              (1u << 21)

/* GOTGINT: write one to clear */

#define OTG_GOTGINT_SEDET               (1u << 2)
#define OTG_GOTGINT_ADTOCHG             (1u << 18)
#define OTG_GOTGINT_W1C_MASK            (OTG_GOTGINT_SEDET | \
                                        OTG_GOTGINT_ADTOCHG)

/* GAHBCFG */

#define OTG_GAHBCFG_GINTMSK             (1u << 0)
#define OTG_GAHBCFG_HBSTLEN_SHIFT       1
#define OTG_GAHBCFG_HBSTLEN_MASK        (15u << 1)
#define OTG_GAHBCFG_DMAEN               (1u << 5)
#define OTG_GAHBCFG_TXFELVL             (1u << 7)
#define OTG_GAHBCFG_PTXFELVL            (1u << 8)

/* GUSBCFG: H7 PHYSEL/ULPI bits are reserved on N6. */

#define OTG_GUSBCFG_TOCAL_SHIFT         0
#define OTG_GUSBCFG_TOCAL_MASK          7u
#define OTG_GUSBCFG_TRDT_SHIFT          10
#define OTG_GUSBCFG_TRDT_MASK           (15u << 10)
#define OTG_GUSBCFG_PHYLPC              (1u << 15)
#define OTG_GUSBCFG_FHMOD               (1u << 29)
#define OTG_GUSBCFG_FDMOD               (1u << 30)

/* GRSTCTL: reset/flush bits self-clear; AHBIDL/DMAREQ are read-only. */

#define OTG_GRSTCTL_CSRST               (1u << 0)
#define OTG_GRSTCTL_PSRST               (1u << 1)
#define OTG_GRSTCTL_RXFFLSH             (1u << 4)
#define OTG_GRSTCTL_TXFFLSH             (1u << 5)
#define OTG_GRSTCTL_TXFNUM_SHIFT        6
#define OTG_GRSTCTL_TXFNUM_MASK         (31u << 6)
#define OTG_GRSTCTL_TXFNUM(n)           ((n) << 6)
#define OTG_GRSTCTL_TXFNUM_ALL          (16u << 6)
#define OTG_GRSTCTL_DMAREQ              (1u << 30)
#define OTG_GRSTCTL_AHBIDL              (1u << 31)

/* GINTSTS/GINTMSK device-mode bits. Summary/FIFO status is read-only. */

#define OTG_GINT_CMOD                   (1u << 0)
#define OTG_GINT_MMIS                   (1u << 1)
#define OTG_GINT_OTGINT                 (1u << 2)
#define OTG_GINT_SOF                    (1u << 3)
#define OTG_GINT_RXFLVL                 (1u << 4)
#define OTG_GINT_NPTXFE                 (1u << 5)
#define OTG_GINT_GINAKEFF               (1u << 6)
#define OTG_GINT_GONAKEFF               (1u << 7)
#define OTG_GINT_ESUSP                  (1u << 10)
#define OTG_GINT_USBSUSP                (1u << 11)
#define OTG_GINT_USBRST                 (1u << 12)
#define OTG_GINT_ENUMDNE                (1u << 13)
#define OTG_GINT_ISOODRP                (1u << 14)
#define OTG_GINT_EOPF                   (1u << 15)
#define OTG_GINT_IEPINT                 (1u << 18)
#define OTG_GINT_OEPINT                 (1u << 19)
#define OTG_GINT_IISOIXFR               (1u << 20)
#define OTG_GINT_INCOMPISOOUT           (1u << 21)
#define OTG_GINT_DATAFSUSP              (1u << 22)
#define OTG_GINT_RSTDET                 (1u << 23)
#define OTG_GINT_LPMINT                 (1u << 27)
#define OTG_GINT_CIDSCHG                (1u << 28)
#define OTG_GINT_SRQINT                 (1u << 30)
#define OTG_GINT_WKUPINT                (1u << 31)
#define OTG_GINTSTS_W1C_MASK            (OTG_GINT_MMIS | OTG_GINT_SOF | \
                                        OTG_GINT_ESUSP | OTG_GINT_USBSUSP | \
                                        OTG_GINT_USBRST | OTG_GINT_ENUMDNE | \
                                        OTG_GINT_ISOODRP | OTG_GINT_EOPF | \
                                        OTG_GINT_IISOIXFR | \
                                        OTG_GINT_INCOMPISOOUT | \
                                        OTG_GINT_DATAFSUSP | OTG_GINT_RSTDET | \
                                        OTG_GINT_LPMINT | OTG_GINT_CIDSCHG | \
                                        OTG_GINT_SRQINT | OTG_GINT_WKUPINT)

/* GRXSTSR/GRXSTSP device receive status */

#define OTG_GRXST_EPNUM_SHIFT           0
#define OTG_GRXST_EPNUM_MASK            15u
#define OTG_GRXST_BCNT_SHIFT            4
#define OTG_GRXST_BCNT_MASK             (0x7ffu << 4)
#define OTG_GRXST_DPID_SHIFT            15
#define OTG_GRXST_DPID_MASK             (3u << 15)
#define OTG_GRXST_PKTSTS_SHIFT          17
#define OTG_GRXST_PKTSTS_MASK           (15u << 17)
#define OTG_GRXST_PKTSTS_GONAK          (1u << 17)
#define OTG_GRXST_PKTSTS_OUTRECVD       (2u << 17)
#define OTG_GRXST_PKTSTS_OUTDONE        (3u << 17)
#define OTG_GRXST_PKTSTS_SETUPDONE      (4u << 17)
#define OTG_GRXST_PKTSTS_SETUPRECVD     (6u << 17)
#define OTG_GRXST_STSPHST               (1u << 27)

/* FIFO start/depth fields are measured in 32-bit words, not bytes. */

#define OTG_GRXFSIZ_DEPTH_MASK          0xffffu
#define OTG_DIEPTXF_START_SHIFT         0
#define OTG_DIEPTXF_START_MASK          0xffffu
#define OTG_DIEPTXF_DEPTH_SHIFT         16
#define OTG_DIEPTXF_DEPTH_MASK          (0xffffu << 16)

/* GCCFG: B-session validity is always software-controlled on N6. */

#define OTG_GCCFG_VBUSVLD               (1u << 4) /* Read-only */
#define OTG_GCCFG_VBVALOVAL             (1u << 23)
#define OTG_GCCFG_IDPULLUPDIS           (1u << 28)

/* DCFG/DCTL/DSTS */

#define OTG_DCFG_DSPD_MASK              3u
#define OTG_DCFG_DSPD_HS                0u
#define OTG_DCFG_DSPD_FS                1u
#define OTG_DCFG_NZLSOHSK               (1u << 2)
#define OTG_DCFG_DAD_SHIFT              4
#define OTG_DCFG_DAD_MASK               (0x7fu << 4)
#define OTG_DCFG_PFIVL_SHIFT            11
#define OTG_DCFG_PFIVL_MASK             (3u << 11)
#define OTG_DCFG_PFIVL_80PCT            (0u << 11)
#define OTG_DCTL_RWUSIG                 (1u << 0)
#define OTG_DCTL_SDIS                   (1u << 1)
#define OTG_DCTL_GINSTS                 (1u << 2) /* Read-only */
#define OTG_DCTL_GONSTS                 (1u << 3) /* Read-only */
#define OTG_DCTL_TCTL_SHIFT             4
#define OTG_DCTL_TCTL_MASK              (7u << 4)
#define OTG_DCTL_SGINAK                 (1u << 7)
#define OTG_DCTL_CGINAK                 (1u << 8)
#define OTG_DCTL_SGONAK                 (1u << 9)
#define OTG_DCTL_CGONAK                 (1u << 10)
#define OTG_DCTL_POPRGDNE               (1u << 11)
#define OTG_DSTS_SUSPSTS                (1u << 0)
#define OTG_DSTS_ENUMSPD_SHIFT          1
#define OTG_DSTS_ENUMSPD_MASK           (3u << 1)
#define OTG_DSTS_ENUMSPD_HS             (0u << 1)
#define OTG_DSTS_ENUMSPD_FS             (1u << 1)
#define OTG_DSTS_EERR                   (1u << 3)
#define OTG_DSTS_FNSOF_SHIFT            8
#define OTG_DSTS_FNSOF_MASK             (0x3fffu << 8)

/* Endpoint control. OUT EP0 has an encoded maximum-packet field. */

#define OTG_DOEPCTL0_MPSIZ_MASK         3u
#define OTG_DOEPCTL0_MPSIZ_64           0u
#define OTG_DOEPCTL0_MPSIZ_32           1u
#define OTG_DOEPCTL0_MPSIZ_16           2u
#define OTG_DOEPCTL0_MPSIZ_8            3u
#define OTG_EPCTL_MPSIZ_MASK            0x7ffu
#define OTG_EPCTL_USBAEP                (1u << 15)
#define OTG_EPCTL_DPID                  (1u << 16) /* Read-only */
#define OTG_EPCTL_NAKSTS                (1u << 17) /* Read-only */
#define OTG_EPCTL_EPTYP_SHIFT           18
#define OTG_EPCTL_EPTYP_MASK            (3u << 18)
#define OTG_EPCTL_EPTYP_CTRL            (0u << 18)
#define OTG_EPCTL_EPTYP_BULK            (2u << 18)
#define OTG_EPCTL_EPTYP_INTR            (3u << 18)
#define OTG_EPCTL_STALL                 (1u << 21)
#define OTG_DIEPCTL_TXFNUM_SHIFT        22
#define OTG_DIEPCTL_TXFNUM_MASK         (15u << 22)
#define OTG_EPCTL_CNAK                  (1u << 26)
#define OTG_EPCTL_SNAK                  (1u << 27)
#define OTG_EPCTL_SD0PID                (1u << 28)
#define OTG_EPCTL_SD1PID                (1u << 29)
#define OTG_EPCTL_EPDIS                 (1u << 30)
#define OTG_EPCTL_EPENA                 (1u << 31)

/* Endpoint interrupts: TXFE is read-only; other named bits are W1C.
 * Common endpoint interrupt mask bits occupy the same positions except
 * TXFE, which is enabled per endpoint through DIEPEMPMSK.
 */

#define OTG_DIEPINT_XFRC                (1u << 0)
#define OTG_DIEPINT_EPDISD              (1u << 1)
#define OTG_DIEPINT_AHBERR              (1u << 2)
#define OTG_DIEPINT_TOC                 (1u << 3)
#define OTG_DIEPINT_ITTXFE              (1u << 4)
#define OTG_DIEPINT_INEPNM              (1u << 5)
#define OTG_DIEPINT_INEPNE              (1u << 6)
#define OTG_DIEPINT_TXFE                (1u << 7)
#define OTG_DIEPINT_TXFIFOUDRN          (1u << 8)
#define OTG_DIEPINT_PKTDRPSTS           (1u << 11)
#define OTG_DIEPINT_NAK                 (1u << 13)
#define OTG_DIEPINT_W1C_MASK            0x0000297fu
#define OTG_DOEPINT_XFRC                (1u << 0)
#define OTG_DOEPINT_EPDISD              (1u << 1)
#define OTG_DOEPINT_AHBERR              (1u << 2)
#define OTG_DOEPINT_STUP                (1u << 3)
#define OTG_DOEPINT_OTEPDIS             (1u << 4)
#define OTG_DOEPINT_STSPHSRX            (1u << 5)
#define OTG_DOEPINT_B2BSTUP             (1u << 6)
#define OTG_DOEPINT_OUTPKTERR           (1u << 8)
#define OTG_DOEPINT_BERR                (1u << 12)
#define OTG_DOEPINT_NAK                 (1u << 13)
#define OTG_DOEPINT_NYET                (1u << 14)
#define OTG_DOEPINT_STPKTRX             (1u << 15)
#define OTG_DOEPINT_W1C_MASK            0x0000f17fu
#define OTG_DIEPMSK_VALID_MASK          0x0000217fu
#define OTG_DOEPMSK_VALID_MASK          0x0000717fu

/* DAINT is read-only; DAINTMSK/DIEPEMPMSK select implemented endpoints. */

#define OTG_DAINT_IN(n)                 (1u << (n))
#define OTG_DAINT_OUT(n)                (1u << ((n) + 16))
#define OTG_DAINT_ALL                   0x01ff01ffu
#define OTG_DIEPEMPMSK_ALL              0x000001ffu

/* Transfer sizes. EP0 limits differ from EP1..EP8. */

#define OTG_DIEPTSIZ0_XFRSIZ_MASK       0x7fu
#define OTG_DIEPTSIZ0_PKTCNT_SHIFT      19
#define OTG_DIEPTSIZ0_PKTCNT_MASK       (3u << 19)
#define OTG_DOEPTSIZ0_XFRSIZ_MASK       0x7fu
#define OTG_DOEPTSIZ0_PKTCNT            (1u << 19)
#define OTG_DOEPTSIZ0_STUPCNT_SHIFT     29
#define OTG_DOEPTSIZ0_STUPCNT_MASK      (3u << 29)
#define OTG_EPTSIZ_XFRSIZ_MASK          0x7ffffu
#define OTG_EPTSIZ_PKTCNT_SHIFT         19
#define OTG_EPTSIZ_PKTCNT_MASK          (0x3ffu << 19)
#define OTG_DTXFSTS_INEPTFSAV_MASK      0xffffu

/* PCGCCTL */

#define OTG_PCGCCTL_STPPCLK             (1u << 0)
#define OTG_PCGCCTL_GATEHCLK            (1u << 1)
#define OTG_PCGCCTL_PHYSUSP             (1u << 4)
#define OTG_PCGCCTL_ENL1GTG             (1u << 5)
#define OTG_PCGCCTL_PHYSLEEP            (1u << 6)
#define OTG_PCGCCTL_SUSP                (1u << 7)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_OTG_H */
