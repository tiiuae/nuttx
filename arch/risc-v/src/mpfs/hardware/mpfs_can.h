/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_can.h
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CAN_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MSS CAN TX/RX buffer configuration */

#define CAN_RX_BUFFER 32u
#define CAN_TX_BUFFER 32u

/* MSS CAN Configuration and Speed definitions */

#define CAN_PRESET                  (mpfs_can_config_reg.L)0
#define CAN_SAMPLE_BOTH_EDGES       0x00000001u
#define CAN_THREE_SAMPLES           0x00000002u
#define CAN_SET_SJW(_sjw)           (_sjw<<2u)
#define CAN_AUTO_RESTART            0x00000010u
#define CAN_SET_TSEG2(_tseg2)       (_tseg2<<5u)
#define CAN_SET_TSEG1(_tseg1)       (_tseg1<<8u)
#define CAN_SET_BITRATE(_bitrate)   (_bitrate<<16u)
#define CAN_ARB_ROUNDROBIN          0x00000000u
#define CAN_ARB_FIXED_PRIO          0x00001000u
#define CAN_BIG_ENDIAN              0x00000000u
#define CAN_LITTLE_ENDIAN           0x00002000u

/*-------------------------------------------------------------------------*//*
  The following constants are used in the PolarFire SoC MSS CAN driver for
  bitrate definitions:

  | Constants          |  Description                                        |
  |--------------------|-----------------------------------------------------|
  | CAN_SPEED_8M_5K    | Indicates CAN controller shall be configured with   |
  |                    | 5Kbps baud rate if the input clock is 8MHz.         |
  | CAN_SPEED_16M_5K   | Indicates CAN controller shall be configured with   |
  |                    | 5Kbps baud rate if the input clock is 16MHz.        |
  | CAN_SPEED_32M_5K   | Indicates CAN controller shall be configured with   |
  |                    | 5Kbps baud rate if the input clock is 32MHz.        |
  | CAN_SPEED_8M_10K   | Indicates CAN controller shall be configured with   |
  |                    | 10Kbps baud rate if the input clock is 8MHz.        |
  | CAN_SPEED_16M_10K  | Indicates CAN controller shall be configured with   |
  |                    | 10Kbps baud rate if the input clock is 16MHz.       |
  | CAN_SPEED_32M_10K  | Indicates CAN controller shall be configured with   |
  |                    | 10Kbps baud rate if the input clock is 32MHz.       |
  | CAN_SPEED_8M_20K   | Indicates CAN controller shall be configured with   |
  |                    | 20Kbps baud rate if the input clock is 8MHz.        |
  | CAN_SPEED_16M_20K  | Indicates CAN controller shall be configured with   |
  |                    | 20Kbps baud rate if the input clock is 16MHz.       |
  | CAN_SPEED_32M_20K  | Indicates CAN controller shall be configured with   |
  |                    | 20Kbps baud rate if the input clock is 32MHz.       |
  | CAN_SPEED_8M_50K   | Indicates CAN controller shall be configured with   |
  |                    | 50Kbps baud rate if the input clock is 8MHz.        |
  | CAN_SPEED_16M_50K  | Indicates CAN controller shall be configured with   |
  |                    | 50Kbps baud rate if the input clock is 16MHz.       |
  | CAN_SPEED_32M_50K  | Indicates CAN controller shall be configured with   |
  |                    | 50Kbps baud rate if the input clock is 32MHz.       |
  | CAN_SPEED_8M_100K  | Indicates CAN controller shall be configured with   |
  |                    | 100Kbps baud rate if the input clock is 8MHz.       |
  | CAN_SPEED_16M_100K | Indicates CAN controller shall be configured with   |
  |                    | 100Kbps baud rate if the input clock is 16MHz.      |
  | CAN_SPEED_32M_100K | Indicates CAN controller shall be configured with   |
  |                    | 100Kbps baud rate if the input clock is 32MHz.      |
  | CAN_SPEED_8M_125K  | Indicates CAN controller shall be configured with   |
  |                    | 125Kbps baud rate if the input clock is 8MHz.       |
  | CAN_SPEED_16M_125K | Indicates CAN controller shall be configured with   |
  |                    | 125Kbps baud rate if the input clock is 16MHz.      |
  | CAN_SPEED_32M_125K | Indicates CAN controller shall be configured with   |
  |                    | 125Kbps baud rate if the input clock is 32MHz.      |
  | AN_SPEED_8M_250K   | Indicates CAN controller shall be configured with   |
  |                    | 250Kbps baud rate if the input clock is 8MHz.       |
  | CAN_SPEED_16M_250K | Indicates CAN controller shall be configured with   |
  |                    | 250Kbps baud rate if the input clock is 16MHz.      |
  | CAN_SPEED_32M_250K | Indicates CAN controller shall be configured with   |
  |                    | 250Kbps baud rate if the input clock is 32MHz.      |
  | CAN_SPEED_8M_500K  | Indicates CAN controller shall be configured with   |
  |                    | 500Kbps baud rate if the input clock is 8MHz.       |
  | CAN_SPEED_16M_500K | Indicates CAN controller shall be configured with   |
  |                    | 500Kbps baud rate if the input clock is 16MHz.      |
  | CAN_SPEED_32M_500K | Indicates CAN controller shall be configured with   |
  |                    | 500Kbps baud rate if the input clock is 32MHz.      |
  | CAN_SPEED_8M_1M    | Indicates CAN controller shall be configured with   |
  |                    | 1MBPS baud rate if the input clock is 8MHz.         |
  | CAN_SPEED_16M_1M   | Indicates CAN controller shall be configured with   |
  |                    | 1MBPS baud rate if the input clock is 16MHz.        |
  | CAN_SPEED_32M_1M   | Indicates CAN controller shall be configured with   |
  |                    | 1MBPS baud rate if the input clock is 32MHz.        |
 */

/* 5000m       81%  Sample bit three times  */

#define CAN_SPEED_8M_5K      CAN_SET_BITRATE(99)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_16M_5K     CAN_SET_BITRATE(199)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_32M_5K     CAN_SET_BITRATE(399)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES

/* 5000m       81%  Sample bit three times */

#define CAN_SPEED_8M_10K     CAN_SET_BITRATE(49)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_16M_10K    CAN_SET_BITRATE(99)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_32M_10K    CAN_SET_BITRATE(199)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES

/* 2500m       81%  Sample bit three times */

#define CAN_SPEED_8M_20K     CAN_SET_BITRATE(24)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_16M_20K    CAN_SET_BITRATE(49)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES
#define CAN_SPEED_32M_20K    CAN_SET_BITRATE(99)|CAN_SET_TSEG1(11)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES

/* 1000m       87% */

#define CAN_SPEED_8M_50K     CAN_SET_BITRATE(9)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_50K    CAN_SET_BITRATE(19)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_50K    CAN_SET_BITRATE(39)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)

/* 600m        87% */

#define CAN_SPEED_8M_100K    CAN_SET_BITRATE(4)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_100K   CAN_SET_BITRATE(9)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_100K   CAN_SET_BITRATE(19)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)

/*  500m        87% */

#define CAN_SPEED_8M_125K    CAN_SET_BITRATE(3)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_125K   CAN_SET_BITRATE(7)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_125K   CAN_SET_BITRATE(15)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)

/* 250m        87% */

#define CAN_SPEED_8M_250K    CAN_SET_BITRATE(1)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_250K   CAN_SET_BITRATE(3)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_250K   CAN_SET_BITRATE(7)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)

/* 100m        75% @ 8M, 87% @ 16M */

#define CAN_SPEED_8M_500K    CAN_SET_BITRATE(1)|CAN_SET_TSEG1(4)|CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_500K   CAN_SET_BITRATE(1)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_500K   CAN_SET_BITRATE(3)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)

/* 25m         75% */
#define CAN_SPEED_8M_1M      CAN_SET_BITRATE(0)|CAN_SET_TSEG1(4)|CAN_SET_TSEG2(1)
#define CAN_SPEED_16M_1M     CAN_SET_BITRATE(1)|CAN_SET_TSEG1(4)|CAN_SET_TSEG2(1)
#define CAN_SPEED_32M_1M     CAN_SET_BITRATE(1)|CAN_SET_TSEG1(12)|CAN_SET_TSEG2(1)

/*-------------------------------------------------------------------------*//**
  The following constants are used for error codes:

  |  Constants            |  Description                                |
  |-----------------------|---------------------------------------------|
  | CAN_OK                | Indicates there is no error                 |
  | CAN_ERR               | Indicates error condition                   |
  | CAN_TSEG1_TOO_SMALL   | Value provided to configure TSEG1 is too    |
  |                       | small                                       |
  | CAN_TSEG2_TOO_SMALL   | Value provided to configure TSEG2 is too    |
  |                       | small                                       |
  | CAN_SJW_TOO_BIG       | Value provided to configure synchronous jump|
  |                       | width (SJW) is too big.                     |
  | CAN_BASIC_CAN_BUFFER  | Indicates that buffer is configured for     |
  |                       | Basic CAN operation                         |
  | CAN_NO_RTR_BUFFER     | Indicates that there is no buffer for       |
  |                       | remote transmit request (RTR) frame         |
  | CAN_INVALID_BUFFER    | Indicates invalid buffer number             |
 */

#define CAN_OK                 0u
#define CAN_ERR                1u
#define CAN_TSEG1_TOO_SMALL    2u
#define CAN_TSEG2_TOO_SMALL    3u
#define CAN_SJW_TOO_BIG        4u
#define CAN_BASIC_CAN_BUFFER  5u
#define CAN_NO_RTR_BUFFER     6u
#define CAN_INVALID_BUFFER    7u

/* MSS CAN Msg Flag bits */

#define CAN_NO_MSG         0x00u
#define CAN_VALID_MSG      0x01u

/*-------------------------------------------------------------------------*//**
  The following constants are used in the MSS CAN driver for Interrupt Bit
  Definitions

  |  Constants               |  Description                                   |
  |--------------------------|------------------------------------------------|
  | CAN_INT_GLOBAL           | Indicates to enable global interrupts          |
  | CAN_INT_ARB_LOSS         | Indicates arbitration loss interrupt           |
  | CAN_INT_OVR_LOAD         | Indicates overload message detected interrupt  |
  | CAN_INT_BIT_ERR          | Indicates bit error interrupt                  |
  | CAN_INT_STUFF_ERR        | Indicates bit stuffing error interrupt         |
  | CAN_INT_ACK_ERR          | Indicates acknowledge error interrupt          |
  | CAN_INT_FORM_ERR         | Indicates format error interrupt               |
  | CAN_INT_CRC_ERR          | Indicates CRC error interrupt                  |
  | CAN_INT_BUS_OFF          | Indicates bus off interrupt                    |
  | CAN_INT_RX_MSG_LOST      | Indicates received message lost interrupt      |
  | CAN_INT_TX_MSG           | Indicates message transmit interrupt           |
  | CAN_INT_RX_MSG           | Indicates receive message available interrupt  |
  | CAN_INT_RTR_MSG          | Indicates RTR auto-reply message sent interrupt|
  | CAN_INT_STUCK_AT_0       | Indicates stuck at dominant error interrupt    |
  | CAN_INT_SST_FAILURE      | Indicates single shot transmission failure     |
  |                          | interrupt                                      |
 */

#define CAN_INT_GLOBAL        1<<0    /* Global interrupt  */
#define CAN_INT_ARB_LOSS      1<<2    /* Arbitration loss interrupt  */
#define CAN_INT_OVR_LOAD      1<<3    /*Overload interrupt  */
#define CAN_INT_BIT_ERR       1<<4    /* Bit error interrupt  */
#define CAN_INT_STUFF_ERR     1<<5    /* Bit stuffing error interrupt  */
#define CAN_INT_ACK_ERR       1<<6    /* Acknowledgement error interrupt  */
#define CAN_INT_FORM_ERR      1<<7    /* Format error interrupt  */
#define CAN_INT_CRC_ERR       1<<8    /* CRC error interrupt  */
#define CAN_INT_BUS_OFF       1<<9    /* Bus-off interrupt  */
#define CAN_INT_RX_MSG_LOST   1<<10   /* Rx message lost interrupt  */
#define CAN_INT_TX_MSG        1<<11   /* Tx message interupt  */
#define CAN_INT_RX_MSG        1<<12   /* Rx message interrupt  */
#define CAN_INT_RTR_MSG       1<<13   /* RTR message interrupt  */
#define CAN_INT_STUCK_AT_0    1<<14   /* Stuck-at-0 error interrupt  */
#define CAN_INT_SST_FAILURE   1<<15   /* Single-shot transmission error interrupt*/

/*-------------------------------------------------------------------------*//**
  The following constants are used for transmit message buffer control bit
  definitions:

  |  Constants               |  Description                                   |
  |--------------------------|------------------------------------------------|
  | CAN_TX_WPNH_EBL          | Indicates “WPNH” bit mask                      |
  | CAN_TX_WPNL_EBL          | Indicates "WPNL" bit mask                      |
  | CAN_TX_INT_EBL           | Indicates transmit Interrupt enable bit mask   |
  | CAN_TX_ABORT             | Indicates Transmit abort mask                  |
  | CAN_TX_REQ               | Indicates transmit request flag bit position   |

 */

#define CAN_TX_WPNH_EBL      1<<23
#define CAN_TX_WPNL_EBL      1<<3
#define CAN_TX_INT_EBL       1<<2
#define CAN_TX_ABORT         1<<1
#define CAN_TX_REQ           0x01u

/*-------------------------------------------------------------------------*//**
  The following constants are used for receive message buffer control bit
  definitions:

  |  Constants               |  Description                                   |
  |--------------------------|------------------------------------------------|
  | CAN_RX_WPNH_EBL          | Indicates WPNH bit mask.                       |
  | CAN_RX_WPNL_EBL          | Indicates WPNL bit mask                        |
  | CAN_RX_LINK_EBL          | Indicates link flag bit mask                   |
  | CAN_RX_INT_EBL           | Indicates receive interrupt enable bit mask    |
  | CAN_RX_RTR_REPLY_EBL     | Indicates RTR reply bit mask                   |
  | CAN_RX_BUFFER_EBL        | Indicates Transaction buffer enable bit mask   |
  | CAN_RX_RTR_ABORT         | Indicates RTR abort request mask               |
  | CAN_RX_RTRP              | Indicates RTReply pending status mask          |
 */

#define CAN_RX_WPNH_EBL      1<<23
#define CAN_RX_WPNL_EBL      1<<7
#define CAN_RX_LINK_EBL      1<<6
#define CAN_RX_INT_EBL       1<<5
#define CAN_RX_RTR_REPLY_EBL 1<<4
#define CAN_RX_BUFFER_EBL    1<<3
#define CAN_RX_RTR_ABORT     1<<2
#define CAN_RX_RTRP          1<<1

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CAN_H */