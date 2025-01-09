/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_can.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/time.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>
#include <nuttx/can/can.h>

#include <arch/board/board.h>

#include "mpfs_can.h"
#include "riscv_internal.h"
#include "mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef OK
#  define OK 0
#endif

/* This module only compiles if the MSS CAN instance
 * is configured to the FPGA
 */

#ifndef CONFIG_MPFS_HAVE_MSS_CAN
#  error This should not be compiled if MSS CAN block is not enabled
#endif

/* Macros */

#define    CAN_ID_SHIFT                   18u
#define    CAN_ERROR_STATUS_SHIFT         16u
#define    CAN_ERROR_STATUS_MASK          0x03u
#define    CAN_ERROR_STATE_ACTIVE         0u
#define    CAN_ERROR_STATE_PASSIVE        1u
#define    CAN_ERROR_STATE_BUS_OFF        2u
#define    CAN_RX_GTE96_SHIFT             19u
#define    CAN_FLAG_MASK                  0x01u
#define    CAN_ERROR_COUNT_SHIFT          8u
#define    CAN_ERROR_COUNT_MASK           0xFFu
#define    CAN_TXGTE96_SHIFT              18u
#define    CAN_INT_MASK                   0xFFFFFFFCu
#define    ENABLE                         1u
#define    DISABLE                        0u
#define    SYSREG_CAN_SOFTRESET_MASK      (uint32_t)(3 << 14u)

/* High level driver operational configuration */

#define CANWORK                 HPWORK

/* For allocating the tx and rx CAN frame buffer */

#define POOL_SIZE               1
#define TIMESTAMP_SIZE          sizeof(struct timeval)  /* To support
                                                         * timestamping frame */

/****************************************************************************
 * Utility definitions
 ****************************************************************************/

#ifndef min
#define min(a, b)  ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b)  ((a) > (b) ? (a) : (b))
#endif

#define clamp(val, lo, hi)  min(max(val, lo), hi)

#define print_uint32_t(prefix, val)  do { \
  caninfo("%s: 0b", prefix); \
  for (int i = 31; i >= 0; i--) { \
      caninfo("%d", ((val) >> i) & 1); \
  } \
  caninfo("\n"); \
} while (0)

#if __GNUC__ >= 3
#  define expect(expr,value) __builtin_expect((expr),(value))
#else
#  define expect(expr,value) (expr)
#endif

#define expect_false(expr)   expect((expr) != 0, 0)
#define expect_true(expr)    expect((expr) != 0, 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/*-------------------------------------------------------------------------*//**
  The mpfs_can_mode_t enumeration specifies the possible operating modes of CAN
  controller. The meaning of the constants is as described below

  |  Modes                     |  Description                             |
  |----------------------------|------------------------------------------|
  | CANOP_MODE_NORMAL          | Indicates CAN controller is in normal    |
  |                            | operational mode.                        |
  | CANOP_MODE_LISTEN_ONLY     | Indicates CAN controller is in listen    |
  |                            | only mode.                               |
  | CANOP_MODE_EXT_LOOPBACK    | Indicates CAN controller is in external  |
  |                            | loop back mode.                          |
  | CANOP_MODE_INT_LOOPBACK    | Indicates CAN controller is in internal  |
  |                            | loop back mode.                          |
  | CANOP_SRAM_TEST_MODE       | Indicates CAN controller is in test mode.|
 */

typedef enum mpfs_can_mode
{
  CANOP_MODE_NORMAL       = 0x01u,
  CANOP_MODE_LISTEN_ONLY  = 0x03u,
  CANOP_MODE_EXT_LOOPBACK = 0x05u,
  CANOP_MODE_INT_LOOPBACK = 0x07u,
  CANOP_SRAM_TEST_MODE    = 0x08u
}   mpfs_can_mode_t;

/* CAN message object */

typedef struct _mpfs_can_msgobject
{
  /* CAN Message ID. */

  struct
  {
    volatile uint32_t N_ID:3;
    volatile uint32_t ID:29;
  };

  /* CAN Message Data organized as two 32 bit words or 8 data bytes */

  union
  {
    struct
    {
      volatile uint32_t DATAHIGH;
      volatile uint32_t DATALOW;
    };
    volatile int8_t DATA[8];
  };

  /* CAN Message flags and smaller values organized as one single 32 bit word
    * or a number of bit fields.
    */

  union
  {
    volatile uint32_t L;   /* 32 bit flag */
    struct
    {
      /* Flags structure. */

      volatile uint32_t NA0:16;
      volatile uint32_t DLC:4;
      volatile uint32_t IDE:1;
      volatile uint32_t RTR:1;
      volatile uint32_t NA1:10;
    };
  };
} mpfs_can_msgobject_t;

typedef mpfs_can_msgobject_t * pmpfs_can_msgobject_t;

/* CAN filter object */

typedef struct _CAN_filterobject
{
  /* Use sw range filter */
  bool use_range_filter;

  /* Use sw mask filter */

  bool use_mask_filter;

  /* Range filter upper */

  uint32_t range_upper;

  /* Range filter lower */

  uint32_t range_lower;

  /* Mask filter mask */

  uint32_t mask;

  /* Mask filter code */

  uint32_t code;

  /* Acceptance mask settings for hw filter */

  union
  {
    volatile    uint32_t L;
    struct
    {
      volatile uint32_t N_A:1;
      volatile uint32_t RTR:1;
      volatile uint32_t IDE:1;
      volatile uint32_t ID:29;
    };
  } AMR;

  /* Acceptance code settings for hw filter */

  union
  {
    volatile uint32_t L;
    struct
    {
      volatile uint32_t N_A:1;
      volatile uint32_t RTR:1;
      volatile uint32_t IDE:1;
      volatile uint32_t ID:29;
    };
  } ACR;
} mpfs_can_filterobject_t;

/* CAN TX message object */

typedef struct _CAN_txmsgobject
{
  /* CAN Message flags and smaller values organized as one single 32 bit word
    * or a number of bit fields.
    */

  union
  {
    volatile uint32_t L;

    /* Tx Flags structure. */

    struct
    {
      volatile uint32_t TXREQ:1;
      volatile uint32_t TXABORT:1;
      volatile uint32_t TXINTEBL:1;
      volatile uint32_t WPNL:1;
      volatile uint32_t NA0:12;
      volatile uint32_t DLC:4;
      volatile uint32_t IDE:1;
      volatile uint32_t RTR:1;
      volatile uint32_t NA1:1;
      volatile uint32_t WPNH:1;
      volatile uint32_t NA2:8;
    };
  } TXB;

  /* CAN Message ID. */

  struct
  {
    volatile uint32_t N_ID:3;
    volatile uint32_t ID:29;
  };

  /* CAN Message Data organized as two 32 bit words or 8 data bytes */

  union
  {
    struct
    {
      volatile uint32_t DATAHIGH;
      volatile uint32_t DATALOW;
    };
    volatile int8_t DATA[8];
  };

} mpfs_can_txmsgobject_t;

/* CAN RX message object */

typedef struct _mpfs_can_rxmsgobject
{
  /* CAN Message flags and smaller values organized as one single 32 bit word
    * or a number of bit fields.
    */

  union
  {
    volatile uint32_t L;                 /* 32 bit flag */

    /* Tx Flags structure. */

    struct
    {
      volatile uint32_t MSGAV:1;
      volatile uint32_t RTRREPLYPEND:1;
      volatile uint32_t RTRABORT:1;
      volatile uint32_t BUFFEREBL:1;
      volatile uint32_t RTRREPLY:1;
      volatile uint32_t RXINTEBL:1;
      volatile uint32_t LINKFLAG:1;
      volatile uint32_t WPNL:1;
      volatile uint32_t NA0:8;
      volatile uint32_t DLC:4;
      volatile uint32_t IDE:1;
      volatile uint32_t RTR:1;
      volatile uint32_t NA1:1;
      volatile uint32_t WPNH:1;
      volatile uint32_t NA2:8;
    };
  } RXB;

  /* CAN Message ID.  */

  struct
  {
    volatile uint32_t N_ID:3;
    volatile uint32_t ID:29;
  };

  /* CAN Message Data organized as two 32 bit words or 8 data bytes */

  union
  {
    struct
    {
      volatile uint32_t DATAHIGH;
      volatile uint32_t DATALOW;
    };
    volatile int8_t DATA[8];
  };

  /* CAN Message Filter: Acceptance mask register */

  union
  {
    volatile uint32_t L;
    struct
    {
      volatile uint32_t N_A:1;
      volatile uint32_t RTR:1;
      volatile uint32_t IDE:1;
      volatile uint32_t ID:29;
    };
  } AMR;

  /* CAN Message Filter: Acceptance code register */

  union
  {
    volatile uint32_t L;
    struct
    {
      volatile uint32_t N_A:1;
      volatile uint32_t RTR:1;
      volatile uint32_t IDE:1;
      volatile uint32_t ID:29;
    };
  } ACR;

  volatile uint32_t AMR_D;
  volatile uint32_t ACR_D;

} mpfs_can_rxmsgobject_t;
typedef mpfs_can_rxmsgobject_t * pmpfs_can_rxmsgobject_t;

/* Error status register */

typedef union error_status
{
  volatile  uint32_t L;
  struct
  {
    volatile uint32_t TX_ERR_CNT:8;
    volatile uint32_t RX_ERR_CNT:8;
    volatile uint32_t ERROR_STAT:2;
    volatile uint32_t TXGTE96:1;
    volatile uint32_t RXGTE96:1;
    volatile uint32_t N_A:12;
  };
} mpfs_can_error_status_t;

/* CAN device statistics */

typedef struct device_stats
{
  volatile uint32_t error_passive;     /* Changes to error passive count */
  volatile uint32_t bus_off;           /* Changes to bus off count */
  volatile uint32_t arbitration_lost;  /* Arbitration lost errors count */
  volatile uint32_t rx_overload;       /* Rx overload errors count */
  volatile uint32_t restarts;          /* CAN controller re-starts count */
  volatile uint32_t txb_sent;          /* Tx messages sent count */
} mpfs_can_device_stats_t;

/* Buffer status register */

typedef struct buffer_status
{
  volatile const uint32_t RXMSGAV;
  volatile const uint32_t TXREQ;
} mpfs_can_buffer_status_t;

/* Interrupt enable register */

typedef union int_enable
{
  volatile  uint32_t L;
  struct
  {
    volatile uint32_t INT_EBL:1;
    volatile uint32_t N_A0:1;
    volatile uint32_t ARB_LOSS:1;
    volatile uint32_t OVR_LOAD:1;
    volatile uint32_t BIT_ERR:1;
    volatile uint32_t STUFF_ERR:1;
    volatile uint32_t ACK_ERR:1;
    volatile uint32_t FORM_ERR:1;
    volatile uint32_t CRC_ERR:1;
    volatile uint32_t BUS_OFF:1;
    volatile uint32_t RX_MSG_LOSS:1;
    volatile uint32_t TX_MSG:1;
    volatile uint32_t RX_MSG:1;
    volatile uint32_t RTR_MSG:1;
    volatile uint32_t STUCK_AT_0:1;
    volatile uint32_t SST_FAILURE:1;
    volatile uint32_t N_A1:16;
  };
} mpfs_can_int_enable_t;

typedef mpfs_can_int_enable_t * pmpfs_can_int_enable_t;

/* Interrupt status register */

typedef union int_status
{
  volatile uint32_t L;
  struct
  {
    volatile uint32_t N_A0:2;
    volatile uint32_t ARB_LOSS:1;
    volatile uint32_t OVR_LOAD:1;
    volatile uint32_t BIT_ERR:1;
    volatile uint32_t STUFF_ERR:1;
    volatile uint32_t ACK_ERR:1;
    volatile uint32_t FORM_ERR:1;
    volatile uint32_t CRC_ERR:1;
    volatile uint32_t BUS_OFF:1;
    volatile uint32_t RX_MSG_LOSS:1;
    volatile uint32_t TX_MSG:1;
    volatile uint32_t RX_MSG:1;
    volatile uint32_t RTR_MSG:1;
    volatile uint32_t STUCK_AT_0:1;
    volatile uint32_t SST_FAILURE:1;
    volatile uint32_t N_A1:16;
  };
} mpfs_can_int_status_t;
typedef mpfs_can_int_status_t * pmpfs_can_int_status_t;

/* Command register */

typedef union command_reg
{
  volatile uint32_t L;
  struct
  {
    volatile uint32_t RUN_STOP:1;
    volatile uint32_t LISTEN_ONLY:1;
    volatile uint32_t LOOP_BACK:1;
    volatile uint32_t SRAM_TEST:1;
    volatile uint32_t SW_RESET:1;
    volatile uint32_t N_A:27;
  };
} mpfs_can_command_reg_t;

/* Configuration register */

typedef union can_config_reg
{
  volatile uint32_t L;
  struct
  {
    volatile uint32_t EDGE_MODE:1;
    volatile uint32_t SAMPLING_MODE:1;
    volatile uint32_t CFG_SJW:2;
    volatile uint32_t AUTO_RESTART:1;
    volatile uint32_t CFG_TSEG2:3;
    volatile uint32_t CFG_TSEG1:4;
    volatile uint32_t CFG_ARBITER:1;
    volatile uint32_t ENDIAN:1;
    volatile uint32_t ECR_MODE:1;
    volatile uint32_t N_A0:1;
    volatile uint32_t CFG_BITRATE:15;
    volatile uint32_t N_A1:1;
  };
} mpfs_can_config_reg_t;
typedef mpfs_can_config_reg_t * pmpfs_can_config_reg_t ;

/* Register mapping of CAN controller */

typedef struct CAN_device
{
  mpfs_can_int_status_t    IntStatus;      /* Interrupt status register */
  mpfs_can_int_enable_t    IntEbl;         /* Interrupt enable register */
  mpfs_can_buffer_status_t BufferStatus;   /* Buffer status indicators */
  mpfs_can_error_status_t  ErrorStatus;    /* Error status */
  mpfs_can_command_reg_t   Command;        /* CAN operating mode */
  mpfs_can_config_reg_t    Config;         /* Configuration register */
  uint32_t                 NA;
  mpfs_can_txmsgobject_t   TxMsg[CAN_TX_BUFFER];   /* Tx message buffers */
  mpfs_can_rxmsgobject_t   RxMsg[CAN_RX_BUFFER];   /* Rx message buffers */
} CAN_DEVICE;

typedef CAN_DEVICE * PCAN_DEVICE;

#define MPFS_CAN_0_LO_BASE           (CAN_DEVICE*)0x2010C000
#define MPFS_CAN_1_LO_BASE           (CAN_DEVICE*)0x2010D000
#define MPFS_CAN_0_HI_BASE           (CAN_DEVICE*)0x2810C000
#define MPFS_CAN_1_HI_BASE           (CAN_DEVICE*)0x2810D000

#define SYSREG_CAN_A_SOFTRESET_MASK           ( (uint32_t)0x01u << 14u )
#define SYSREG_CAN_B_SOFTRESET_MASK           ( (uint32_t)0x01u << 15u )

/*-------------------------------------------------------------------------*//**
  The structure mpfs_can_instance_t is used by the driver to manage the
  configuration and operation of each MSS CAN peripheral. The instance content
  should only be accessed by using the respective API functions.

  Each API function has a pointer to this instance as first argument.
 */

typedef struct can_instance
{
  CAN_DEVICE * hw_reg;      /* Pointer to CAN registers. */

  uint32_t bitrate_value;    /* The numerical bitrate value in bit/s */

  bool bifup;               /* Indicates if the CAN is up or down. */

  /* Interrupt handling */

  uint8_t irqn;             /* IRQ number */
  uint32_t isr;             /* Interrupt status register */

  /* Error status and Stats */

  uint8_t error_status;                 /* Error status */
  uint32_t tx_err_count;                /* Tx error count */
  uint32_t rx_err_count;                /* Rx error count */
  mpfs_can_device_stats_t stats;        /* Device statistics */

  /* Mailbox count */

  uint8_t  basic_can_rxb_count; /* number of rx buffers */
  uint8_t  basic_can_txb_count; /* number of tx buffers */

  /* Frame descriptors */

  struct can_frame *txdesc; /* Pointer to the transmit frame descriptor. */
  struct can_frame *rxdesc; /* Pointer to the receive frame descriptor. */

  /* MSS CAN composite message objects */

  pmpfs_can_msgobject_t tx_msg; /* Pointer to the transmit message object */
  pmpfs_can_msgobject_t rx_msg; /* Pointer to the receive message object */

  /* Work queue entries */

  struct work_s rxwork;   /* for deferring rx interrupt work to the wq */
  struct work_s txdwork;  /* For deferring tx done interrupt work to the wq */
  struct work_s pollwork; /* For deferring poll work to the wq */

  mpfs_can_filterobject_t filter;  /* hardware and software filters */

  struct net_driver_s dev;  /* Interface understood by the Nuttx network */
} mpfs_can_instance_t;

/* Driver memory pool */

static mpfs_can_instance_t g_can;

static uint8_t g_tx_pool[(sizeof(struct can_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct can_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE];

static mpfs_can_msgobject_t g_tx_msg;
static mpfs_can_msgobject_t g_rx_msg;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* (from interrupt) High-level RX related functions */

static bool mpfs_can_retrieve_rx_frame(mpfs_can_instance_t *priv,
                                       struct can_frame *cf);
static void mpfs_receive_work(void *arg);

/* (from interrupt) High-level TX related functions */

static void mpfs_txdone_work(void *arg);

/* High-level periodical TX related functions */

static int mpfs_transmit(mpfs_can_instance_t *priv);
static int mpfs_txpoll(struct net_driver_s *dev);
static void mpfs_txavail_work(void *arg);
static int mpfs_txavail(struct net_driver_s *dev);

/* (from interrupt) High-level error handling related functions */

static void mpfs_err_interrupt(mpfs_can_instance_t *priv, uint32_t isr);

/* Interrupt service routine */

static int mpfs_interrupt(int irq, void *context, void *arg);

/* RX SW/HW filter related functions */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static uint8_t mpfs_can_add_sw_filter(mpfs_can_instance_t *priv,
                                   uint8_t filter_type,
                                   uint32_t filter_id1,
                                   uint32_t filter_id2);
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

static uint8_t mpfs_can_reset_filter(mpfs_can_instance_t *priv);

/* CAN controller configuration setter and status getter helper functions */

static void mpfs_can_reset(mpfs_can_instance_t* priv, uint32_t cfg);

static void mpfs_can_set_mode(mpfs_can_instance_t* priv, mpfs_can_mode_t mode);

static void mpfs_can_set_int_ebl(mpfs_can_instance_t* priv, uint32_t irq_flag);
static void mpfs_can_clear_int_ebl(mpfs_can_instance_t* priv, uint32_t irq_flag);
static uint32_t mpfs_can_get_int_ebl(mpfs_can_instance_t* priv);

static void mpfs_can_clear_int_status(mpfs_can_instance_t* priv, uint32_t irq_flag);
static uint32_t mpfs_can_get_int_status(mpfs_can_instance_t* priv);

static uint8_t mpfs_can_get_error_status(mpfs_can_instance_t* priv);

static void mpfs_can_print_status (mpfs_can_instance_t* priv);

/* CAN controller life cycle functions */

static void mpfs_can_start(mpfs_can_instance_t* priv);
static void mpfs_can_stop(mpfs_can_instance_t* priv);

/* CAN message helper functions */

static uint32_t mpfs_can_set_id(pmpfs_can_msgobject_t pmsg);
static uint32_t mpfs_can_get_msg_filter_mask(uint32_t id, uint8_t ide, uint8_t rtr);
static uint8_t mpfs_can_set_bitrate(mpfs_can_instance_t* priv,
                                 uint32_t bitrate,
                                 bool reset);
static uint32_t mpfs_can_get_sample_point(mpfs_can_instance_t* priv);

/* CAN message RX buffer setter/getter functions */

static uint8_t mpfs_can_config_buffer(mpfs_can_instance_t* priv);

static uint8_t mpfs_can_get_message(mpfs_can_instance_t* priv);

static uint32_t mpfs_can_get_rx_buffer_status(mpfs_can_instance_t* priv);

static uint32_t mpfs_can_get_rx_error_count(mpfs_can_instance_t* priv);

static uint32_t mpfs_can_get_rx_gte96(mpfs_can_instance_t* priv);

/* CAN message TX bufer setter/getter functions */

static uint8_t mpfs_can_send_message_ready(mpfs_can_instance_t* priv);
static uint8_t mpfs_can_send_message(mpfs_can_instance_t* priv);

static uint32_t mpfs_can_get_tx_buffer_status(mpfs_can_instance_t* priv);

static uint32_t mpfs_can_get_tx_error_count(mpfs_can_instance_t* priv);

static uint32_t mpfs_can_get_tx_gte96(mpfs_can_instance_t* priv);

/* Driver interface to Nuttx network callbacks */

static int mpfs_ifup(struct net_driver_s *dev);
static int mpfs_ifdown(struct net_driver_s *dev);
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_can_retrieve_rx_frame
 *
 * Description:
 *  Retrieve CAN 2.0B frame from RX Buffer
 *
 * Input Parameters:
 *  priv    - Pointer to the private CAN driver state structure
 *  cf      - Pointer to CAN frame structure
 *
 * Returned Value:
 *  true if frame was read successfully, false otherwise
 *
 * Assumptions:
 *  Frame format word is already parsed in advance and provided as 'ffw' arg
 *
 ****************************************************************************/

static bool mpfs_can_retrieve_rx_frame(mpfs_can_instance_t *priv,
                                       struct can_frame *cf)
{
  unsigned int dlc;
  pmpfs_can_msgobject_t pmsg = priv->rx_msg;

  /* CAN ID & EFF & RTR Flags */

  cf->can_id = mpfs_can_get_msg_filter_mask(pmsg->ID, pmsg->IDE, pmsg->RTR);

  /* Filter check */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL

  if (priv->filter.use_mask_filter)
    {
      if ((cf->can_id & priv->filter.mask) != priv->filter.code)
        {
          return false;
        }
    }
  else if (priv->filter.use_range_filter)
    {
      if (cf->can_id < priv->filter.range_lower ||
          cf->can_id > priv->filter.range_upper)
        {
          return false;
        }
    }
#endif  /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

  /* DLC */

  dlc = pmsg->DLC;
  if (dlc <= 8)
    {
     cf->can_dlc = dlc;
    }
  else
    {
      canerr("DLC = %d is out of range\n", dlc);
      return false;
    }

  /* Data (big endian) */
  *(uint32_t *)(cf->data) = pmsg->DATALOW;
  *(uint32_t *)(cf->data + 4) = pmsg->DATAHIGH;

  return true;
}

/****************************************************************************
 * Name: mpfs_receive_work
 *
 * Description:
 *  An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void mpfs_receive_work(void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;
  struct can_frame *cf = (struct can_frame *)priv->rxdesc;

  while (mpfs_can_get_message(priv))
    {
      /* Retrieve the CAN 2.0B frame */

      if (!mpfs_can_retrieve_rx_frame(priv, cf))
      {
        /* Didn't receive full frame or message got filtered out */

        continue;
      }

      /* Lock the network; we have to protect the dev.d_len, dev.d_buf
      * and dev.d_iob from the devif_poll path
      */

      net_lock();

      /* Copy the buffer pointer to priv->dev.d_buf  Set amount of data
        * in priv->dev.d_len
        */

      priv->dev.d_len = sizeof(struct can_frame);
      priv->dev.d_buf = (uint8_t *)cf;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);
      can_input(&priv->dev);

      net_unlock();

      /* Point the packet buffer back to the next Tx buffer that will be
        * used during the next write.  If the write queue is full, then
        * this will point at an active buffer, which must not be written
        * to.  This is OK because devif_poll won't be called unless the
        * queue is not full.
        */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }

  /* Check for RX FIFO Overflow */

  if (CAN_INT_OVR_LOAD & priv->isr)
    {
      /* Re-enable RX overload error interrupt */

      mpfs_can_set_int_ebl(priv, CAN_INT_OVR_LOAD);
    }

  /* Re-enable INT_RX_MSG interrupt */

  mpfs_can_set_int_ebl(priv, CAN_INT_RX_MSG);
}

/****************************************************************************
 * Name: mpfs_txdone_work
 *
 * Description:
 *  An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Global interrupts are disabled by the watchdog logic.
 *  We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/

static void mpfs_txdone_work(void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, mpfs_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: mpfs_transmit
 *
 * Description:
 *  Start hardware transmission.  Called either from the txdone interrupt
 *  handling or from watchdog based polling.
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno on failure
 *
 * Assumptions:
 *  May or may not be called from an interrupt handler.  In either case,
 *  global interrupts are disabled, either explicitly or indirectly through
 *  interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_transmit(mpfs_can_instance_t *priv)
{
  uint8_t ret;

  /* Retrieve the CAN 2.0B frame from network device buffer */

  struct can_frame *cf = (struct can_frame *)priv->dev.d_buf;

  /* Create the CAN msg object to be sent */

  pmpfs_can_msgobject_t pmsg = priv->tx_msg;
  pmsg->IDE = cf->can_id & CAN_EFF_FLAG;
  pmsg->DLC = cf->can_dlc;
  pmsg->ID = mpfs_can_set_id(pmsg);
  pmsg->DATAHIGH = *(uint32_t *)(cf->data + 4);
  pmsg->DATALOW = *(uint32_t *)(cf->data);

  /* Insert CAN frame into available TX bf */

  if (CAN_OK != (ret = mpfs_can_send_message(priv)))
    {
      canwarn("Failed to send CAN frame due to %s\n",
              ret == CAN_INVALID_BUFFER ? "invalid buffer" :
              ret == CAN_NO_MSG ? "no TX buffer available" : "unknown error");
      return CAN_OK;
    }

  /* Increment statistics */

  priv->stats.txb_sent++;
  NETDEV_TXPACKETS(&priv->dev);

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero (CAN_OK) on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_txpoll(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      mpfs_transmit(priv);

      /* Check if there is room in the device to hold another packet. If
        * not, return a non-zero value to terminate the poll.
        */

      if (CAN_OK != mpfs_can_send_message_ready(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void mpfs_txavail_work(void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the controller to hold another outgoing
       * packet.
       */

      if (mpfs_can_send_message_ready(priv))
        {
          /* Yes, there is, poll the network for new TXT transmit */

          net_lock();
          devif_poll(&priv->dev, mpfs_txpoll);
          net_unlock();
        }
    }
}

/****************************************************************************
 * Name: mpfs_txavail
 *
 * Description:
 *  Driver callback invoked when new TX data is available.  This is a
 *  stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *  latency.
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  0 - CAN_OK
 *
 * Assumptions:
 *  Called in normal user mode
 *
 ****************************************************************************/

static int mpfs_txavail(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      mpfs_txavail_work(priv);
    }

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_err_interrupt
 *
 * Description:
 *    Error frame ISR
 *
 * Input Parameters:
 *    priv  - Pointer to the private CAN driver state structure
 *    isr   - Interrupt status register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_err_interrupt(mpfs_can_instance_t *priv, uint32_t isr)
{
  /* Set error status */

  uint8_t state = mpfs_can_get_error_status(priv);
  priv->tx_err_count = mpfs_can_get_tx_error_count(priv);
  priv->rx_err_count = mpfs_can_get_rx_error_count(priv);

  caninfo("error status %s, txerr %d,rxerr %d\n", state ?
          state == 1 ? "Error Passive" : "Bus Off" : "Error Active",
          priv->tx_err_count, priv->rx_err_count);

  /* Check for state change */

  if (priv->error_status == state)
    {
      canwarn("No state change! Missed interrupt?\n");
    }

  priv->error_status = state;

  switch (state)
    {
    case CAN_ERROR_STATE_BUS_OFF:
      priv->stats.bus_off++;
      canwarn("Change to BUS_OFF error state\n");
      break;
    case CAN_ERROR_STATE_PASSIVE:
      priv->stats.error_passive++;
      canwarn("Change to ERROR_PASSIVE error state\n");

      /* Mask bus off and arbitration loss interrupts */

      mpfs_can_clear_int_ebl(priv, CAN_INT_ARB_LOSS | CAN_INT_BUS_OFF);
      break;
    case CAN_ERROR_STATE_ACTIVE:
      caninfo("Change to ERROR_ACTIVE error state\n");

      /* Unmask bus off and arbitration loss interrupts */

      mpfs_can_set_int_ebl(priv, CAN_INT_ARB_LOSS | CAN_INT_BUS_OFF);
      return;
    default:
      canwarn("Unhandled error state %d\n", state);
      break;
    }

  if (CAN_INT_ARB_LOSS & isr)
    {
      canerr("Arbitration lost error interrupt\n");
      priv->stats.arbitration_lost++;
    }

  if (CAN_INT_OVR_LOAD & isr)
    {
      canerr("RX overload error interrupt\n");
      priv->stats.rx_overload++;

      /* Mask the interrupt until it is handled in worker */

      mpfs_can_clear_int_ebl(priv, CAN_INT_OVR_LOAD);

      /* Notify to socket interface */

      NETDEV_RXERRORS(&priv->dev);
    }

  /* Notify to socket interface. */

  NETDEV_ERRORS(&priv->dev);
}

/****************************************************************************
 * Name: mpfs_interrupt
 *
 * Description:
 *   Three interrupt sources will vector to this function:
 *   1. CAN frame transmit interrupt
 *   2. CAN frame receive interrupt
 *   3. Error interrupt
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int mpfs_interrupt(int irq, void *context, void *arg)
{
  mpfs_can_instance_t *priv = (mpfs_can_instance_t *)arg;

  /* Get the interrupt status */

  priv->isr = mpfs_can_get_int_status(priv);

  /* RX available interrupt */

  if (priv->isr & CAN_INT_RX_MSG)
    {
      /* Mask INT_RX_MSG until received message is handled be the worker */

      mpfs_can_clear_int_ebl(priv, CAN_INT_RX_MSG);

      work_queue(CANWORK, &priv->rxwork, mpfs_receive_work, priv, 0);
    }

  /* TX done interrupt */

  if (priv->isr & CAN_INT_TX_MSG)
    {
      /* Schedule work to poll for next available tx frame from the network */

      work_queue(CANWORK, &priv->txdwork, mpfs_txdone_work, priv, 0);
    }

  /* Error interrupts */

  if ((priv->isr & (CAN_INT_ARB_LOSS | CAN_INT_OVR_LOAD
            | CAN_INT_BIT_ERR | CAN_INT_STUFF_ERR
            | CAN_INT_ACK_ERR | CAN_INT_FORM_ERR
            | CAN_INT_CRC_ERR | CAN_INT_BUS_OFF)) != 0)
    {
      canerr("Some error interrupts...");
#ifdef CONFIG_DEBUG_CAN_INFO
      mpfs_can_print_status(priv);
#endif

      mpfs_err_interrupt(priv, priv->isr);
    }

  /* All interrupts are now handled, clear them */

  mpfs_can_clear_int_status(priv, priv->isr);
  return OK;
}

/****************************************************************************
 * Name: mpfs_can_add_sw_filter
 *
 * Description:
 *  Add new sw filter to CAN Controller. Currently only support ID filter
 *
 * Input Parameters:
 *  priv          - Pointer to the private CAN driver state structure
 *  filter_type   - The type of the filter: mask filter or range filter
 *  filter_id1    - filter id 1 (can be filter value for mask filter or range
 *                  low for range filter)
 *  filter_id2    - filter id 2 (can be filter mask for mask filter or range
 *                  high for range filter)
 *
 * Returned Value:
 *  This function returns CAN_OK on successful bitrate set, otherwise it will
 *  returns CAN_ERR
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static uint8_t mpfs_can_add_sw_filter(mpfs_can_instance_t *priv,
                                      uint8_t filter_type,
                                      uint32_t filter_id1,
                                      uint32_t filter_id2)
{
  if (filter_type == CAN_FILTER_MASK)
    {
      if (!priv->filter.use_mask_filter)
        {
          canwarn("Mask filter is already in use. Overwrite now\n");
        }
      priv->filter.use_mask_filter = true;
      priv->filter.code = filter_id1;
      priv->filter.mask = filter_id2;
    }
  else if (filter_type == CAN_FILTER_RANGE)
    {
      if (priv->filter.use_range_filter)
        {
          canwarn("Range filter is already in use. Overwrite now\n");
        }
      priv->filter.use_range_filter = true;
      priv->filter.range_lower = filter_id1;
      priv->filter.range_upper = filter_id2;
    }
  else
    {
      canerr("Invalid filter type\n");
      return CAN_ERR;
    }

  return CAN_OK;
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/****************************************************************************
 * Name: mpfs_can_reset_filter
 *
 * Description:
 *  Reset both sw and hw filters to default settings
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful bitrate set, otherwise it will
 *  returns CAN_ERR
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_reset_filter(mpfs_can_instance_t *priv)
{
  uint8_t ret;

  priv->filter.use_mask_filter = false;
  priv->filter.use_range_filter = false;
  priv->filter.AMR.L = 0xFFFFFFFFu; /* Mask bit == 1 => bits are not checked */

  /* Reset hw filter and configure RX buffer */

  if (CAN_OK != (ret = mpfs_can_config_buffer(priv)))
    {
      canerr("Failed to configure RX buffer:%d\n", ret);
      return CAN_ERR;
    }

  return CAN_OK;
}

/****************************************************************************
 * Name: The mpfs_can_reset
 *
 * Description:
 *  The mpfs_can_reset() function  sets the configuration register and
 *  starts the CAN controller for normal mode operation. This function is used
 *  when one needs to change the configuration settings while the CAN
 *  controller was already initialized using mpfs_can_init() function  and is
 *  running. mpfs_can_reset() function should not be used when the CAN
 *  controller wasn't initialized yet
 *  It performs following tasks:
 *    - Clears all pending interrupts
 *    - Stops CAN controller
 *    - Disable interrupts
 *    - Optionally sets new configuration
 *    - Starts CAN controller
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  cfg   - The cfg parameter is a 4 bytes variable used to set the
 *          configuration settings
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_reset(mpfs_can_instance_t* priv,
                            uint32_t cfg)
{
  /* Clear all pending interrupts */

  priv->hw_reg->IntStatus.L = DISABLE;

  /* Disable CAN Device */

  priv->hw_reg->Command.RUN_STOP = DISABLE;

  /* Disable receive interrupts. */

  priv->hw_reg->IntEbl.RX_MSG = DISABLE;

  /* Disable interrupts from CAN device. */

  priv->hw_reg->IntEbl.INT_EBL = DISABLE;

  /* Sets configuration bits */

  priv->hw_reg->Config.L = cfg;
  mpfs_can_start(priv);

  priv->stats.restarts++;
}

/****************************************************************************
 * Name: The mpfs_can_set_mode
 *
 * Description:
 *  The mpfs_can_set_mode() function sets the CAN controller operating mode
 *  based on the mode parameter. After this operation CAN controller is not in
 *  operational, to do that invoke mpfs_can_start() function
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  mode  - The mode parameter tells about desired operating mode of CAN
 *          controller. Possible operating modes are as mentioned below:
 *    |  Mode                    | Description                             |
 *    |--------------------------|-----------------------------------------|
 *    | CANOP_MODE_NORMAL        | Sets normal operating mode              |
 *    | CANOP_MODE_LISTEN_ONLY   | In listen-only mode, the CAN controller |
 *    |                          | does not send any messages. Normally    |
 *    |                          | used for automatic bitrate detection    |
 *    | CANOP_MODE_INT_LOOPBACK  | Selects internal loopback mode. This is |
 *    |                          | used for self-test                      |
 *    | CANOP_MODE_EXT_LOOPBACK  | Selects external loopback. The CAN      |
 *    |                          | controller will receive a copy of each  |
 *    |                          | message sent.                           |
 *    | CANOP_SRAM_TEST_MODE     | Sets SRAM test mode                     |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_set_mode(mpfs_can_instance_t* priv,
                       mpfs_can_mode_t mode)
{
  priv->hw_reg->Command.RUN_STOP = DISABLE;

  priv->hw_reg->Command.L = (uint32_t)mode;
}

/****************************************************************************
 * Name: The mpfs_can_set_int_ebl
 *
 * Description:
 *  The mpfs_can_set_int_ebl() function enable specific interrupt based on
 *  irq_flag parameter
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  irq_flag  - The irq_flag parameter is a 4 byte variable indicates
 *    Interrupt type. Possible values are:
 *  |  Constant              |  Description                                   |
 *  |------------------------|------------------------------------------------|
 *  | CAN_INT_GLOBAL         | Indicates to enable global interrupts          |
 *  | CAN_INT_ARB_LOSS       | Indicates arbitration loss interrupt           |
 *  | CAN_INT_OVR_LOAD       | Indicates overload message detected interrupt  |
 *  | CAN_INT_BIT_ERR        | Indicates bit error interrupt                  |
 *  | CAN_INT_STUFF_ERR      | Indicates bit stuffing error interrupt         |
 *  | CAN_INT_ACK_ERR        | Indicates acknowledge error interrupt          |
 *  | CAN_INT_FORM_ERR       | Indicates format error interrupt               |
 *  | CAN_INT_CRC_ERR        | Indicates CRC error interrupt                  |
 *  | CAN_INT_BUS_OFF        | Indicates bus off interrupt                    |
 *  | CAN_INT_RX_MSG_LOST    | Indicates received message lost interrupt      |
 *  | CAN_INT_TX_MSG         | Indicates message transmit interrupt           |
 *  | CAN_INT_RX_MSG         | Indicates receive message available interrupt  |
 *  | CAN_INT_RTR_MSG        | Indicates RTR auto-reply message sent interrupt|
 *  | CAN_INT_STUCK_AT_0     | Indicates stuck at dominant error interrupt    |
 *  | CAN_INT_SST_FAILURE    | Indicates single shot transmission failure     |
 *  |                        | interrupt                                      |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_set_int_ebl(mpfs_can_instance_t* priv,
                         uint32_t irq_flag)
{
  priv->hw_reg->IntEbl.L |= irq_flag;
}

/****************************************************************************
 * Name: The mpfs_can_clear_int_ebl
 *
 * Description:
 *  The mpfs_can_clear_int_ebl() function disable specific interrupt based on
 *  irq_flag parameter
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  irq_flag  - The irq_flag parameter is a 4 byte variable indicates
 *    Interrupt type. Possible values are:
 *  |  Constant              |  Description                                   |
 *  |------------------------|------------------------------------------------|
 *  | CAN_INT_GLOBAL         | Indicates to enable global interrupts          |
 *  | CAN_INT_ARB_LOSS       | Indicates arbitration loss interrupt           |
 *  | CAN_INT_OVR_LOAD       | Indicates overload message detected interrupt  |
 *  | CAN_INT_BIT_ERR        | Indicates bit error interrupt                  |
 *  | CAN_INT_STUFF_ERR      | Indicates bit stuffing error interrupt         |
 *  | CAN_INT_ACK_ERR        | Indicates acknowledge error interrupt          |
 *  | CAN_INT_FORM_ERR       | Indicates format error interrupt               |
 *  | CAN_INT_CRC_ERR        | Indicates CRC error interrupt                  |
 *  | CAN_INT_BUS_OFF        | Indicates bus off interrupt                    |
 *  | CAN_INT_RX_MSG_LOST    | Indicates received message lost interrupt      |
 *  | CAN_INT_TX_MSG         | Indicates message transmit interrupt           |
 *  | CAN_INT_RX_MSG         | Indicates receive message available interrupt  |
 *  | CAN_INT_RTR_MSG        | Indicates RTR auto-reply message sent interrupt|
 *  | CAN_INT_STUCK_AT_0     | Indicates stuck at dominant error interrupt    |
 *  | CAN_INT_SST_FAILURE    | Indicates single shot transmission failure     |
 *  |                        | interrupt                                      |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_clear_int_ebl(mpfs_can_instance_t* priv,
                            uint32_t irq_flag)
{
  priv->hw_reg->IntEbl.L &= ~irq_flag;
}

/****************************************************************************
 * Name: The mpfs_can_get_int_ebl
 *
 * Description:
 *  The mpfs_can_get_int_ebl() function returns the status of interrupt enable
 *  flags
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns interrupt enable flag status
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_int_ebl(mpfs_can_instance_t* priv)
{
  return (priv->hw_reg->IntEbl.L & CAN_INT_MASK);
}

/****************************************************************************
 * Name: mpfs_can_clear_int_status
 *
 * Description:
 *  The mpfs_can_clear_int_status() function  clears the selected interrupt
 *  flags
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  irq_flag  - The irq_flag parameter is a 4 byte variable indicates Interrupt
 *    type. Possible values are:
 *  |  Constants             |  Description                                   |
 *  |------------------------|------------------------------------------------|
 *  | CAN_INT_GLOBAL         | Indicates to enable global interrupts          |
 *  | CAN_INT_ARB_LOSS       | Indicates arbitration loss interrupt           |
 *  | CAN_INT_OVR_LOAD       | Indicates overload message detected interrupt  |
 *  | CAN_INT_BIT_ERR        | Indicates bit error interrupt                  |
 *  | CAN_INT_STUFF_ERR      | Indicates bit stuffing error interrupt         |
 *  | CAN_INT_ACK_ERR        | Indicates acknowledge error interrupt          |
 *  | CAN_INT_FORM_ERR       | Indicates format error interrupt               |
 *  | CAN_INT_CRC_ERR        | Indicates CRC error interrupt                  |
 *  | CAN_INT_BUS_OFF        | Indicates bus off interrupt                    |
 *  | CAN_INT_RX_MSG_LOST    | Indicates received message lost interrupt      |
 *  | CAN_INT_TX_MSG         | Indicates message transmit interrupt           |
 *  | CAN_INT_RX_MSG         | Indicates receive message available interrupt  |
 *  | CAN_INT_RTR_MSG        | Indicates RTR auto-reply message sent interrupt|
 *  | CAN_INT_STUCK_AT_0     | Indicates stuck at dominant error interrupt    |
 *  | CAN_INT_SST_FAILURE    | Indicates single shot transmission failure     |
 *  |                        | interrupt                                      |
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_clear_int_status(mpfs_can_instance_t* priv,
                               uint32_t irq_flag)
{
  priv->hw_reg->IntStatus.L = irq_flag;
}

/****************************************************************************
 * Name: mpfs_can_get_int_status
 *
 * Description:
 *  The mpfs_can_get_int_status() function returns the status of interrupts
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns status of existed interrupts
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_int_status(mpfs_can_instance_t* priv)
{
  return (priv->hw_reg->IntStatus.L);
}

/****************************************************************************
 * Name: mpfs_can_get_error_status
 *
 * Description:
 *  The mpfs_can_get_error_status() function returns the present error state of
 *  the CAN controller. Error state might be error active or error passive or
 *  bus-off
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  | Codes  |  Descriptions                 |
 *  |--------|-------------------------------|
 *  |  0     | error active                  |
 *  |  1     | error passive                 |
 *  |  2     | bus-off                       |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_get_error_status(mpfs_can_instance_t* priv)
{
  return ((uint8_t)((priv->hw_reg->ErrorStatus.L >> CAN_ERROR_STATUS_SHIFT) &
          CAN_ERROR_STATUS_MASK));
}

/****************************************************************************
 * Name: mpfs_can_print_status
 *
 * Description:
 *  The mpfs_can_print_status() function prints the CAN controller status
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_print_status (mpfs_can_instance_t* priv)
{
  caninfo("=====================================================\n");
  caninfo(">> CAN Settings:\n");
  caninfo("  Bitrate: %u\n", priv->bitrate_value);
  print_uint32_t("  Interrupt Enabled: ", mpfs_can_get_int_ebl(priv));
  caninfo(">> CAN TX/RX buffer, Interrupt status:\n");
  print_uint32_t("  TX Buffer Status: ", mpfs_can_get_tx_buffer_status(priv));
  print_uint32_t("  RX Buffer Status: ", mpfs_can_get_rx_buffer_status(priv));
  print_uint32_t("  Interrupt status register: ", priv->isr);
  caninfo(">> CAN RX/TX Error Status:\n");
  caninfo("  TX Error Count: %d\n", priv->tx_err_count);
  caninfo("  RX Error Count: %d\n", priv->rx_err_count);
  caninfo("  TX GTE 96: %s\n", mpfs_can_get_tx_gte96(priv) ? "Yes" : "No");
  caninfo("  RX GTE 96: %s\n", mpfs_can_get_rx_gte96(priv) ? "Yes" : "No");
  caninfo(" CAN Stats:\n");
  caninfo("  Error Passive: %u\n", priv->stats.error_passive);
  caninfo("  Bus Off: %u\n", priv->stats.bus_off);
  caninfo("  Arbitration Lost: %u\n", priv->stats.arbitration_lost);
  caninfo("  Rx Overload: %u\n", priv->stats.rx_overload);
  caninfo("  Restarts: %u\n", priv->stats.restarts);
  caninfo(" TX message sent: %u\n", priv->stats.txb_sent);
  caninfo("=====================================================\n");
}

/****************************************************************************
 * Name: mpfs_can_set_id
 *
 * Description:
 *  The mpfs_can_set_id() function returns ID bits left justified based on IDE
 *  type. IDE type might be either standard or extended
 *
 * Input Parameters:
 *  pmsg  - The pmsg parameter is a pointer to the message object
 *
 * Returned Value:
 *  This function returns message identifier
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_set_id(pmpfs_can_msgobject_t pmsg)
{
  if (pmsg->IDE)
  {
    return (pmsg->ID);
  }
  else
  {
    return (pmsg->ID << CAN_ID_SHIFT);
  }
}

/****************************************************************************
 * Name: mpfs_can_get_msg_filter_mask
 *
 * Description:
 *  The mpfs_can_get_msg_filter_mask() function  packs the ID, IDE, and RTR
 *  bits together as they are used in the message filter mask and returns
 *  packed identifier
 *
 * Input Parameters:
 *  id  - The id parameter is a 4 byte variable to hold message identifier
 *  ide  - The ide parameter is a 1 byte variable to indicate IDE type.
 *         Acceptable values are as mentioned below:
 *  |  Value     |   Description                 |
 *  |------------|-------------------------------|
 *  |  0         | Standard format               |
 *  |  1         | Extended format               |
 *
 *  rtr  - The rtr parameter is a 1 byte variable to indicate message type.
 *         Acceptable values are as mentioned below:
 *
 *  |  Value     |   Description                 |
 *  |------------|-------------------------------|
 *  |   0        | Regular message (data frame)  |
 *  |   1        | RTR message (remote frame)    |
 *
 * Returned Value:
 *  This function returns packed id
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_msg_filter_mask(uint32_t id,
                                      uint8_t ide,
                                      uint8_t rtr)
{
  id |= ((uint32_t)(ide << 31u) | (uint32_t)(rtr << 30u));

  return (id);
}

/****************************************************************************
 * Name: mpfs_can_set_bitrate
 *
 * Description:
 *  The mpfs_can_set_bitrate() function returns the current set bitrate
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *  bitrate  - The bitrate value in bit/s
 *  reset  - Reset the CAN controller after setting the bitrate
 *
 * Returned Value:
 *  This function returns CAN_OK on successful bitrate set, otherwise it will
 *  returns CAN_ERR
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_set_bitrate(mpfs_can_instance_t* priv,
                                 uint32_t bitrate,
                                 bool reset)
{
  uint32_t bitrate_constant;
  switch (bitrate)
    {
      case 5000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_5K;
        break;
      case 10000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_10K;
        break;
      case 20000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_20K;
        break;
      case 50000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_50K;
        break;
      case 100000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_100K;
        break;
      case 125000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_125K;
        break;
      case 250000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_250K;
        break;
      case 500000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_500K;
        break;
      case 1000000:
        priv->bitrate_value = bitrate;
        bitrate_constant = CAN_SPEED_8M_1M;
        break;
      default:
        canerr("Invalid bitrate %u\n", bitrate);
        return CAN_ERR;
    }

  if (reset)
    {
      mpfs_can_reset(priv, bitrate_constant);
    }
  else
    {
      priv->hw_reg->Config.L = bitrate_constant;
    }

  return CAN_OK;
}

/****************************************************************************
 * Name: mpfs_can_get_sample_point
 *
 * Description:
 *  The mpfs_can_get_sample_point() function returns the current set sample
 *  point. The sample point % can be calculated as follows:
 *  sp = [Tsync + (Tseg1 + 1) * tq]/[Tsync + (Tseq1 + 1) * tq
 *       + (Tseq2 + 1) * tq] where Tsync = 1 tq
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns the current set sample point %
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_sample_point(mpfs_can_instance_t* priv)
{
  uint32_t tseq1 = priv->hw_reg->Config.CFG_TSEG1;
  uint32_t tseq2 = priv->hw_reg->Config.CFG_TSEG2;

  return (tseq1 + 2) / (tseq1 + tseq2 + 3);
}

/****************************************************************************
 * Name: mpfs_can_config_buffer
 *
 * Description:
 *  The mpfs_can_config_buffer() function configures receive buffers
 *  initialized for Basic CAN operation
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful execution, otherwise it will
 *  returns following error codes:
 *
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_NO_MSG            | Indicates that there is no message received |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number            |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_config_buffer (mpfs_can_instance_t* priv)
{
  uint8_t success = CAN_NO_MSG;
  uint8_t buffer_number;

  /* Is a buffer configured for Basic CAN? */

  if (priv->basic_can_rxb_count == 0u)
  {
    return (CAN_INVALID_BUFFER);
  }

  /* Find next BASIC CAN buffer that has a message available */

  for (buffer_number = CAN_RX_BUFFER - priv->basic_can_rxb_count; \
                        buffer_number < CAN_RX_BUFFER; buffer_number++)
  {
    /* Set filters */

    priv->hw_reg->RxMsg[buffer_number].ACR.L = priv->filter.ACR.L;
    priv->hw_reg->RxMsg[buffer_number].AMR.L = priv->filter.AMR.L;

    /* Configure buffer */

    if (buffer_number < (CAN_RX_BUFFER - 1))
    {
      /* set link flag, if not last buffer */

      priv->hw_reg->RxMsg[buffer_number].RXB.L =
                                    (CAN_RX_WPNH_EBL | CAN_RX_WPNL_EBL | \
                                    CAN_RX_BUFFER_EBL | CAN_RX_INT_EBL | \
                                    CAN_RX_LINK_EBL);
    }
    else
    {
      priv->hw_reg->RxMsg[buffer_number].RXB.L =
                                    (CAN_RX_WPNH_EBL | CAN_RX_WPNL_EBL | \
                                    CAN_RX_BUFFER_EBL | CAN_RX_INT_EBL);
    }
    success = CAN_OK;
  }

  return (success);
}

/****************************************************************************
 * Name: mpfs_can_get_message
 *
 * Description:
 *  The mpfs_can_get_message() function read message from the first buffer set
 *  for Basic CAN  operation that contains a message. Once the message has been
 *  read from the buffer, the message receipt is acknowledged.
 *  Note: Since neither a hardware nor a software FIFO exists, message
 *    inversion
 *    might happen (example, a newer message might be read from the receive
 *    buffer prior to an older message)
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_VALID_MSG on successful execution, otherwise it
 *  will returns following error codes:
 *
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_NO_MSG            | Indicates that there is no message received |
 *  | CAN_INVALID_BUFFER    | Indicates invalid buffer number             |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_get_message(mpfs_can_instance_t* priv)
{
  uint8_t success = CAN_NO_MSG;
  uint8_t buffer_number;
  pmpfs_can_msgobject_t pmsg = priv->rx_msg;

  /* Is a buffer configured for Basic CAN? */

  if (priv->basic_can_rxb_count == 0u)
  {
    return (CAN_INVALID_BUFFER);
  }

  /* Find next BASIC CAN buffer that has a message available */

  for (buffer_number = CAN_RX_BUFFER-priv->basic_can_rxb_count;  \
                        buffer_number < CAN_RX_BUFFER; buffer_number++)
  {
    /* Check that if there is a valid message */

    if (priv->hw_reg->RxMsg[buffer_number].RXB.MSGAV)
    {
      /* Copy ID */

      pmsg->ID = priv->hw_reg->RxMsg[buffer_number].ID;

      /* Copy 4 of the data bytes */

      pmsg->DATALOW = priv->hw_reg->RxMsg[buffer_number].DATALOW;

      /* Copy the other 4 data bytes.*/

      pmsg->DATAHIGH = priv->hw_reg->RxMsg[buffer_number].DATAHIGH;

      /* Get DLC, IDE and RTR and time stamp.*/

      pmsg->L = priv->hw_reg->RxMsg[buffer_number].RXB.L;

      /* Ack that it's been removed from the FIFO */

      priv->hw_reg->RxMsg[buffer_number].RXB.MSGAV = ENABLE;
      success = CAN_VALID_MSG;
      break;
    }
  }
  return (success);
}

/****************************************************************************
 * Name: mpfs_can_get_rx_buffer_status
 *
 * Description:
 *  The mpfs_can_get_rx_buffer_status() function returns the buffer status
 *  of all receive(32) buffers
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value: This function returns status of receive buffers
 *   (32 buffers)
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

uint32_t mpfs_can_get_rx_buffer_status(mpfs_can_instance_t* priv)
{
  return (priv->hw_reg->BufferStatus.RXMSGAV);
}

/****************************************************************************
 * Name: mpfs_can_get_rx_error_count
 *
 * Description:
 *  The mpfs_can_get_rx_error_count() function returns the current receive
 *  error counter value. Counter value ranges from 0x00 - 0xFF
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns the receive error counter value
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_rx_error_count(mpfs_can_instance_t* priv)
{
  return ((priv->hw_reg->ErrorStatus.L >> CAN_ERROR_COUNT_SHIFT) & \
          CAN_ERROR_COUNT_MASK);
}

/****************************************************************************
 * Name: mpfs_can_get_rx_gte96
 *
 * Description:
 *  The mpfs_can_get_rx_gte96() function provides information about receive
 *  error count. It identifies that receive error count is greater than or
 *  equal to 96, and reports 1 if count exceeds 96
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  | Value |  Description                                         |
 *  |-------|------------------------------------------------------|
 *  |  0    | if receive error count less than 96.                 |
 *  |  1    | if receive error count greater than or equals to 96. |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_rx_gte96(mpfs_can_instance_t* priv)
{
  return ((priv->hw_reg->ErrorStatus.L >> CAN_RX_GTE96_SHIFT) & \
          CAN_FLAG_MASK);
}

/****************************************************************************
 * Name: mpfs_can_send_message_ready
 *
 * Description:
 *  The mpfs_can_send_message_ready() function will identify the availability
 *  of buffer to fill with new message in basic CAN operation
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful identification of free buffer,
 *  otherwise it will returns following error codes:
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_ERR               | Indicates error condition                   |
 *  | CAN_INVALID_BUFFER   | Indicates invalid buffer number            |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_send_message_ready(mpfs_can_instance_t* priv)
{
  uint8_t success = CAN_ERR;
  uint8_t buffer_number;

  /* Is a buffer configured for Basic CAN? */

  if (priv->basic_can_txb_count == 0u)
  {
    return (CAN_INVALID_BUFFER);
  }

  /* Find next BASIC CAN buffer that is available */

  for (buffer_number = CAN_TX_BUFFER-priv->basic_can_txb_count; \
       buffer_number < CAN_TX_BUFFER; buffer_number++)
  {
    if (priv->hw_reg->TxMsg[buffer_number].TXB.TXREQ == 0u)
    {
      /* Tx buffer isn't busy */

      success = CAN_OK;
      break;
    }
  }

  return (success);
}

/****************************************************************************
 * Name: mpfs_can_send_message
 *
 * Description:
 *  The mpfs_can_send_message() function will copy the data to the first
 *  available buffer set for Basic CAN operation and send data on to the bus.
 *  Note: Since neither a hardware nor a software FIFO exists, message
 *  inversion might happen (example, a newer message might be send from the
 *  transmit buffer prior to an older message)
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  This function returns CAN_OK on successful identification of free buffer,
 *  otherwise it will returns following error codes:
 *  |  Constants            |  Description                                |
 *  |-----------------------|---------------------------------------------|
 *  | CAN_ERR               | Indicates error condition                   |
 *  | CAN_INVALID_BUFFER   | Indicates invalid buffer number            |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint8_t mpfs_can_send_message(mpfs_can_instance_t* priv)
{
  uint8_t success = CAN_NO_MSG;
  uint8_t buffer_number;
  pmpfs_can_msgobject_t pmsg = priv->tx_msg;

  /* Is a buffer configured for Basic CAN? */

  if (priv->basic_can_txb_count == 0u)
  {
    return (CAN_INVALID_BUFFER);
  }

  /* Find next BASIC CAN buffer that is available */

  for (buffer_number = CAN_TX_BUFFER - priv->basic_can_txb_count; \
       buffer_number < CAN_TX_BUFFER; buffer_number++)
  {
    /* Check which transmit buffer is not busy and use it. */

    if ((mpfs_can_get_tx_buffer_status(priv) & (1u << buffer_number)) == 0)
    {
      /* If the Tx buffer isn't busy.... */

      priv->hw_reg->TxMsg[buffer_number].ID = pmsg->ID;
      priv->hw_reg->TxMsg[buffer_number].DATALOW = pmsg->DATALOW;
      priv->hw_reg->TxMsg[buffer_number].DATAHIGH = pmsg->DATAHIGH;
      priv->hw_reg->TxMsg[buffer_number].TXB.L = (pmsg->L | \
                                                  CAN_TX_WPNH_EBL | \
                                                  CAN_TX_REQ);
      success = CAN_VALID_MSG;
      break;
    }
  }

  return (success);
}

/****************************************************************************
 * Name: mpfs_can_get_tx_buffer_status
 *
 * Description:
 *  The mpfs_can_get_tx_buffer_status() function returns the buffer status
 *  of all transmit(32) buffers
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value: This function returns status of transmit buffers
 *   (32 buffers)
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_tx_buffer_status(mpfs_can_instance_t* priv)
{
  return (priv->hw_reg->BufferStatus.TXREQ);
}

/****************************************************************************
 * Name: mpfs_can_get_tx_error_count
 *
 * Description:
 *  The mpfs_can_get_tx_error_count() function returns the current transmit
 *  error counter value. Counter value ranges from 0x00 - 0xFF
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value: This function returns the transmit error counter value
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_tx_error_count(mpfs_can_instance_t* priv)
{
  return (priv->hw_reg->ErrorStatus.L & CAN_ERROR_COUNT_MASK);
}

/****************************************************************************
 * Name: mpfs_can_get_tx_gte96
 *
 * Description:
 *  The mpfs_can_get_tx_gte96() function provides information about transmit
 *  error count. It identifies that transmit error count is greater than or
 *  equals to 96, and reports 1 if count exceeds 96
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  | Value |  Description                                          |
 *  |-------|-------------------------------------------------------|
 *  |  0    | if transmit error count less than 96.                 |
 *  |  1    | if transmit error count greater than or equals to 96. |
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static uint32_t mpfs_can_get_tx_gte96(mpfs_can_instance_t* priv)
{
  return ((priv->hw_reg->ErrorStatus.L >> CAN_TXGTE96_SHIFT) & \
          CAN_FLAG_MASK);
}

/****************************************************************************
 * Name: mpfs_can_start
 *
 * Description:
 *  This routine starts the driver. It clears all pending interrupts and enable
 *  CAN controller to perform normal operation. It enables receive interrupts
 *  also
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_start(mpfs_can_instance_t *priv)
{
  /* Clear all pending interrupts */

  priv->hw_reg->IntStatus.L = DISABLE;

  /* Enable CAN Device */

  priv->hw_reg->Command.RUN_STOP = ENABLE;

  /* Enable receive interrupts */

  priv->hw_reg->IntEbl.RX_MSG = ENABLE;

  /* Enable interrupts from CAN device */

  priv->hw_reg->IntEbl.INT_EBL = ENABLE;
}

/****************************************************************************
 * Name: mpfs_can_stop
 *
 * Description:
 *  This routine stops the driver. This is the drivers stop routine. It will
 * disable the CAN controller
 *
 * Input Parameters:
 *  priv  - Pointer to the private CAN driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_stop(mpfs_can_instance_t *priv)
{
  priv->hw_reg->Command.RUN_STOP = DISABLE;
}

/****************************************************************************
 * Name: mpfs_ifup
 *
 * Description:
 *  NuttX Callback: Start the CAN interface
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  mpfs_can_start(priv);

  priv->bifup = true;

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Enable interrupts */

  up_enable_irq(priv->irqn);

  return OK;
}

/****************************************************************************
 * Name: mpfs_ifdown
 *
 * Description:
 *  NuttX Callback: Stop the CAN interface
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;

  mpfs_can_stop(priv);

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Name: mpfs_ioctl
 *
 * Description:
 *  PHY ioctl command handler
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *  cmd  - ioctl command
 *  arg  - Argument accompanying the command
 *
 * Returned Value:
 *  Zero (CAN_OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#if defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL) || \
defined(CONFIG_NETDEV_CAN_FILTER_IOCTL)
  mpfs_can_instance_t *priv =
    (mpfs_can_instance_t *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
    case SIOCGCANBITRATE:

      /* Get bitrate from the CAN controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);
        req->arbi_bitrate = priv->bitrate_value / 1000; /* kbit/s */
        req->arbi_samplep = mpfs_can_get_sample_point(priv); /* % */
        ret = OK;
      }
      break;

    case SIOCSCANBITRATE:

      /* Set bitrate of the CAN controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);

        if(CAN_OK != mpfs_can_set_bitrate(priv,
                     req->arbi_bitrate * 1000,
                     false))
          {
            canerr("CAN controller bitrate set failed");
            ret = -1;
            break;
          }

        ret = OK;
      }
      break;
#endif /* CONFIG_NETDEV_CAN_BITRATE_IOCTL */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
    case SIOCACANSTDFILTER:

      {
        struct can_ioctl_filter_s *req =
          (struct can_ioctl_filter_s *)((uintptr_t)arg);

        if (CAN_OK != mpfs_can_add_sw_filter(priv, req->ftype,
                                             req->fid1, req->fid2))
          {
            canerr("CAN filter add failed");
            ret = -1;
            break;
          }

        ret = OK;
      }
      break;

    case SIOCDCANSTDFILTER:

      /* Reset all SW/HW filters */

      {
        mpfs_can_reset_filter(priv);
        ret = OK;
      }
      break;

    case SIOCACANEXTFILTER:
    case SIOCDCANEXTFILTER:
      ret = -ENOTTY;
      break;

#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_can_init
 *
 * Description:
 *  Initialize the CAN controller and driver
 *
 * Input Parameters:
 *  ncan    - CAN controller number
 *  bitrate - CAN controller bitrate
 *
 * Returned Value:
 *  On success, a pointer to the MPFS CANFD driver is
 *  returned. NULL is returned on any failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

int mpfs_can_init(int ncan, uint32_t bitrate)
{
  mpfs_can_instance_t *priv;
  priv = &g_can;

  switch (ncan)
    {
# ifdef CONFIG_MPFS_MSS_CAN0
      case 0:
        memset(priv, 0, sizeof(mpfs_can_instance_t));
        priv->hw_reg = MPFS_CAN_0_LO_BASE;
        priv->irqn = MPFS_IRQ_CAN0;
        break;
# endif
# ifdef CONFIG_MPFS_MSS_CAN1
      case 1:
        memset(priv, 0, sizeof(mpfs_can_instance_t));
        priv->hw_reg = MPFS_CAN_1_LO_BASE;
        priv->irqn = MPFS_IRQ_CAN1;
        break;
# endif
      default:
        canerr("No such CAN%d available\n", ncan);
        return -ENOTTY;
    }

  /* Initialize the CAN bitrate */

  mpfs_can_set_bitrate(priv, bitrate, false);
  priv->hw_reg->Config.L = bitrate;

  /* Configure CAN modes */

  mpfs_can_set_mode(priv, CANOP_MODE_NORMAL);

  /* Initialize the number of buffers */

  priv->basic_can_rxb_count = CAN_RX_BUFFER;
  priv->basic_can_txb_count = CAN_TX_BUFFER;

  /* Initialize TX/RX descriptor structure */

  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;

  /* Initialize TX/RX message object structure */

  priv->tx_msg = &g_tx_msg;
  priv->rx_msg = &g_rx_msg;

  /* Initialize the rx buffer and hw filter */

  if (CAN_OK != mpfs_can_reset_filter(priv))
    {
      canerr("CAN filter reset and RX buffer initialization failed\n");
      return -EAGAIN;
    }

  /* Configure interrupts */
  mpfs_can_set_int_ebl(priv, CAN_INT_GLOBAL | CAN_INT_TX_MSG | CAN_INT_RX_MSG
                            | CAN_INT_ARB_LOSS | CAN_INT_OVR_LOAD
                            | CAN_INT_BIT_ERR | CAN_INT_STUFF_ERR
                            | CAN_INT_ACK_ERR | CAN_INT_FORM_ERR
                            | CAN_INT_CRC_ERR | CAN_INT_BUS_OFF);

  /* Disable interrupts */

  priv->hw_reg->IntEbl.L = DISABLE;

  /* Attach the interrupt handler */

  if (irq_attach(priv->irqn, mpfs_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      canerr("Failed to attach to CAN%d IRQ\n", ncan);
      return -EAGAIN;
    }

  /* Initialize the driver network device structure */

  priv->dev.d_ifup    = mpfs_ifup;    /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;  /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail; /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;   /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = priv;     /* Used to recover private state from dev */

  caninfo("CAN driver init done for CAN instance %d\n", ncan);

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling mpfs_ifdown().
   */

  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  return CAN_OK;
}