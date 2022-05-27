/****************************************************************************
 * arch/riscv/src/mpfs/mpfs_fpga_canfd.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
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

#include "hardware/mpfs_fpga_canfd.h"
#include "riscv_internal.h"

#include <arch/board/board.h>

#include <sys/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#ifndef OK
#  define OK 0
#endif


/* This module only compiles if the CAN-FD IP core instance
 * is configured to the FPGA
 */
#ifndef CONFIG_MPFS_CANFD
#  error This should not be compiled as CAN-FD FPGA block is not defined/configured
#endif


/* This module only compiles if the Nuttx socketCAN interface support CAN-FD
 */
#ifndef CONFIG_NET_CAN_CANFD
#  error This should not be compiled as CAN-FD driver relies on socket CAN 
#endif


#define CANWORK LPWORK

/*** NEW ***/
#define MPFS_CANFD_FLAG_RX_FFW_BUFFERED     1

#define MPFS_CANFD_ID                       0xCAFD


/* CONFIG_mpfs_FLEXCAN_NETHIFS determines the number of physical
 * interfaces that will be supported.
 */

#define MASKSTDID                   0x000007ff
#define MASKEXTID                   0x1fffffff
#define FLAGEFF                     (1 << 31) /* Extended frame format */
#define FLAGRTR                     (1 << 30) /* Remote transmission request */

#define RXMBCOUNT                   5
#define TXMBCOUNT                   2
#define TOTALMBCOUNT                RXMBCOUNT + TXMBCOUNT

#define IFLAG1_RX                   ((1 << RXMBCOUNT)-1)
#define IFLAG1_TX                   (((1 << TXMBCOUNT)-1) << RXMBCOUNT)

#define CAN_FIFO_NE                 (1 << 5)
#define CAN_FIFO_OV                 (1 << 6)
#define CAN_FIFO_WARN               (1 << 7)
#define CAN_EFF_FLAG                0x80000000 /* EFF/SFF is set in the MSB */

#define POOL_SIZE                   1

#define MSG_DATA                    sizeof(struct timeval)

/* CAN bit timing values  */
#define CLK_FREQ                    80000000
#define PRESDIV_MAX                 256

#define SEG_MAX                     8
#define SEG_MIN                     1
#define TSEG_MIN                    2
#define TSEG1_MAX                   17
#define TSEG2_MAX                   9
#define NUMTQ_MAX                   26

#define SEG_FD_MAX                  32
#define SEG_FD_MIN                  1
#define TSEG_FD_MIN                 2
#define TSEG1_FD_MAX                39
#define TSEG2_FD_MAX                9
#define NUMTQ_FD_MAX                49

/* CAN control mode */
#define CAN_CTRLMODE_LOOPBACK		    0x01	/* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY		  0x02	/* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES		  0x04	/* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT		    0x08	/* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING	0x10	/* Bus-error reporting */
#define CAN_CTRLMODE_FD			        0x20	/* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK	  0x40	/* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO		  0x80	/* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC	  0x100	/* Classic CAN DLC option */
#define CAN_CTRLMODE_TDC_AUTO		    0x200	/* CAN transiver automatically calculates TDCV */
#define CAN_CTRLMODE_TDC_MANUAL     0x400	/* TDCV is manually set up by user */

/* Interrupt flags for RX fifo */
#define IFLAG1_RXFIFO               (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)

#define MPFS_CAN_STATE_TO_TEXT_ENTRY(st) #st

static int peak_tx_mailbox_index_ = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/*
 * CAN operational and error states
 */

/*
 * CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */
struct mpfs_can_bittiming_s {
  uint32_t bitrate;       /* Bit-rate in bits/second */
  uint32_t sample_point;  /* Sample point in one-tenth of a percent */
  uint32_t tq;            /* Time quanta (TQ) in nanoseconds */
  uint32_t prop_seg;      /* Propagation segment in TQs */
  uint32_t phase_seg1;    /* Phase buffer segment 1 in TQs */
  uint32_t phase_seg2;    /* Phase buffer segment 2 in TQs */
  uint32_t sjw;           /* Synchronisation jump width in TQs */
  uint32_t brp;           /* Bit-rate prescaler */
};

/*
 * CAN harware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters
 */
struct mpfs_can_bittiming_const_s {
  char name[16];          /* Name of the CAN controller hardware */
  uint32_t tseg1_min;     /* Time segement 1 = prop_seg + phase_seg1 */
  uint32_t tseg1_max;
  uint32_t tseg2_min;     /* Time segement 2 = phase_seg2 */
  uint32_t tseg2_max;
  uint32_t sjw_max;       /* Synchronisation jump width */
  uint32_t brp_min;       /* Bit-rate prescaler */
  uint32_t brp_max;
  uint32_t brp_inc;
};

struct mpfs_can_clock_s {
  uint32_t freq;     /* CAN system clock frequency in Hz */
};

enum mpfs_can_state_e {
	CAN_STATE_ERROR_ACTIVE = 0,	/* RX/TX error count < 96 */
	CAN_STATE_ERROR_WARNING,	/* RX/TX error count < 128 */
	CAN_STATE_ERROR_PASSIVE,	/* RX/TX error count < 256 */
	CAN_STATE_BUS_OFF,		/* RX/TX error count >= 256 */
	CAN_STATE_STOPPED,		/* Device is stopped */
	CAN_STATE_SLEEPING,		/* Device is sleeping */
	CAN_STATE_MAX
};

static const char * const mpfs_can_state_strings[CAN_STATE_MAX] = {
	MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_ACTIVE),
	MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_WARNING),
	MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_PASSIVE),
	MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_BUS_OFF),
	MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_STOPPED),
	MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_SLEEPING)
};

struct mpfs_can_ctrlmode_s {
  uint32_t mask;
  uint32_t flags;
};

struct mpfs_can_berr_counter_s {
	uint16_t txerr;
	uint16_t rxerr;
};

struct mpfs_can_device_stats_s {
  uint32_t bus_error;    /* Bus errors */
  uint32_t error_warning;    /* Changes to error warning state */
  uint32_t error_passive;    /* Changes to error passive state */
  uint32_t bus_off;      /* Changes to bus off state */
  uint32_t arbitration_lost; /* Arbitration lost errors */
  uint32_t restarts;     /* CAN controller re-starts */
};

enum mpfs_can_mode_s {
  CAN_MODE_STOP = 0,
  CAN_MODE_START,
  CAN_MODE_SLEEP
};


/*
 * CAN common private data
 */
struct mpfs_can_priv_s {
  struct mpfs_can_device_stats_s can_stats;
  struct mpfs_can_bittiming_s bittiming, data_bittiming;
  const struct mpfs_can_bittiming_const_s *bittiming_const, *data_bittiming_const;
  struct mpfs_can_clock_s clock;

  enum mpfs_can_state_e state;
  uint32_t ctrlmode;
  uint32_t ctrlmode_support;

  int (*do_set_bittiming)(struct net_driver_s *dev);
  int (*do_set_data_bittiming)(struct net_device *dev);
  int (*do_set_mode)(struct net_driver_s *dev, enum can_mode mode);
  int (*do_get_berr_counter)(const struct net_device *dev, struct mpfs_can_berr_counter_s *bec);
};


union cs_u
{
  volatile uint32_t cs;
  struct
  {
    volatile uint32_t dlc : 4;  // data length code
    volatile uint32_t rtr : 1;  // remote transmission request (0 for data, 1 for remote frame)
    volatile uint32_t ide : 1;  // identifier extension bit (0 for 11 standard bits id, 1 for ext id)
    volatile uint32_t srr : 1;  // substitute remote request (must be 1)
    volatile uint32_t esi : 1; // error state indicator (0 for error active state)
    volatile uint32_t brs : 1;  // bit rate switch (0 for normal rate of 1mbps, 1 for higher rate of 5 (practical) or 8 (theoretical) mbps)
    volatile uint32_t edl : 1;  // extended data length (0 for CAN, 1 for CAN-FD)
  };
};

union id_u
{
  volatile uint32_t w;
  struct
  {
    volatile uint32_t ext : 29;
    volatile uint32_t resex : 3;
  };
  struct
  {
    volatile uint32_t res : 18;
    volatile uint32_t std : 11;
    volatile uint32_t resstd : 3;
  };
};

union data_u
{
  volatile uint32_t w00;
  struct
  {
    volatile uint32_t b03 : 8;
    volatile uint32_t b02 : 8;
    volatile uint32_t b01 : 8;
    volatile uint32_t b00 : 8;
  };
};

struct mb_s
{
  union cs_u cs;
  union id_u id;
  union data_u data[16];
};


/**
 * CANFD Frame Format
 */
/* CAN_Frame_format memory map */
enum mpfs_canfd_can_frame_format {
	MPFS_CANFD_FRAME_FORMAT_W       = 0x0,
	MPFS_CANFD_IDENTIFIER_W         = 0x4,
	MPFS_CANFD_TIMESTAMP_L_W        = 0x8,
	MPFS_CANFD_TIMESTAMP_U_W        = 0xc,
	MPFS_CANFD_DATA_1_4_W           = 0x10,
	MPFS_CANFD_DATA_5_8_W           = 0x14,
	MPFS_CANFD_DATA_61_64_W         = 0x4c,
};
/* CANFD_Frame_format memory region */

/*  FRAME_FORMAT_W registers */
#define MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT           (0)
#define MPFS_CANFD_FRAME_FORMAT_W_DLC                 (0x0F << MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT)
#define MPFS_CANFD_FRAME_FORMAT_W_RTR                 (1 << 5)
#define MPFS_CANFD_FRAME_FORMAT_W_IDE                 (1 << 6)
#define MPFS_CANFD_FRAME_FORMAT_W_FDF                 (1 << 7)
#define MPFS_CANFD_FRAME_FORMAT_W_BRS                 (1 << 9)
#define MPFS_CANFD_FRAME_FORMAT_W_ESI_RSV             (1 << 10)
#define MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT         (11)
#define MPFS_CANFD_FRAME_FORMAT_W_RWCNT               (0x1F << MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT)

/*  IDENTIFIER_W registers */
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT  (0)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT        (0x03FFFF << MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT (18)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE       (0x07FF << MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT)


/* FPGA CANFD device hardware configuration */
struct mpfs_config_s
{
  uint32_t canfd_fpga_irq;           /* the only CAN-FD FPGA IRQ */
};

static const struct mpfs_config_s mpfs_fpga_canfd_config =
{
  .canfd_fpga_irq = MPFS_IRQ_FABRIC_F2H_0,
};


/* CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */
struct mpfs_timeseg_s
{
  uint32_t bitrate;
  int32_t samplep;
  uint8_t propseg;
  uint8_t pseg1;
  uint8_t pseg2;
  uint8_t presdiv;
};


/* The mpfs_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct mpfs_driver_s
{
  struct mpfs_can_priv_s can;

  uint32_t base;                /* CANFD FPGA base address */
  bool bifup;                   /* true:ifup false:ifdown */

  struct work_s irqwork;        /* For deferring interrupt work to the work wq */
  struct work_s pollwork;       /* For deferring poll work to the work wq */

  struct canfd_frame *txdesc;   /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc;   /* A pointer to the list of RX descriptors */

  uint32_t drv_flags;           /* driver flag */


  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;      /* Interface understood by the network */

  const struct mpfs_config_s *config;

  struct mb_s *rx;
  struct mb_s *tx;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mpfs_driver_s g_canfd;

static uint8_t g_tx_pool[(sizeof(struct canfd_frame) + MSG_DATA) * POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame) + MSG_DATA) * POOL_SIZE];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/




/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: can_state_to_str
 *
 * Description:
 *  Converts CAN controller state code to corresponding text
 *
 * Input Parameters:
 *  state  - CAN controller state code
 *
 * Returned Value:
 *  Pointer to string representation of the error state
 * 
 * Assumptions:
 *  None
 ****************************************************************************/

static const char *can_state_to_str(enum can_state state)
{
	const char *txt = NULL;

	if (state >= 0 && state < CAN_STATE_MAX)
		txt = mpfs_can_state_strings[state];

  return txt ? txt : "UNKNOWN";
}


/****************************************************************************
 * Function: mpfs_txdone
 *
 * Description:
 *  Check transmit interrupt flags and clear them
 *
 * Input Parameters:
 *  priv  - Reference to the driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_txdone(FAR struct mpfs_driver_s *priv)
{
  uint32_t flags;
  uint32_t mbi;
  uint32_t mb_bit;

  flags  = getreg32(priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
  flags &= IFLAG1_TX;

  /* TODO First Process Error aborts */

  /* Process TX completions */

  mb_bit = 1 << RXMBCOUNT;
  for (mbi = 0; flags && mbi < TXMBCOUNT; mbi++)
    {
      if (flags & mb_bit)
        {
          putreg32(mb_bit, priv->base + S32K1XX_CAN_IFLAG1_OFFSET);
          flags &= ~mb_bit;
          NETDEV_TXDONE(&priv->dev);
#ifdef TX_TIMEOUT_WQ
          /* We are here because a transmission completed, so the
           * corresponding watchdog can be canceled
           * mailbox be set to inactive
           */

          wd_cancel(&priv->txtimeout[mbi]);
          struct mb_s *mb = &priv->tx[mbi];
          mb->cs.code = CAN_TXMB_INACTIVE;
#endif
        }

      mb_bit <<= 1;
    }
}

/****************************************************************************
 * Function: mpfs_txdone_work
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *Returned Value:
 *
 *    None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *   We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/

static void mpfs_txdone_work(FAR void *arg)
{
  FAR struct mpfs_driver_s *priv = (FAR struct mpfs_driver_s *)arg;

  mpfs_txdone(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_timer(&priv->dev, 0, mpfs_txpoll);
  net_unlock();
}


/****************************************************************************
 * Function: mpfs_read_fault_state
 * 
 * Description:
 *    Reads FPGA CANFD fault confinement state 
 *    
 * Input Parameters:
 *    priv  - Reference to the driver state structure
 *
 * Returned Value:
 *    Fault confinement state of controller
 *
 ****************************************************************************/

static enum can_state mpfs_read_fault_state(FAR struct mpfs_driver_s *priv)
{
	u_int32_t ewl_erp_fs_reg, rec_tec_reg, ew_limit, rec_val, tec_val;

  ewl_erp_fs_reg = getreg32(priv->base + MPFS_CANFD_EWL_OFFSET);
  rec_tec_reg = getreg32(priv->base + MPFS_CANFD_REC_OFFSET);

	ew_limit = ((ewl_erp_fs_reg & MPFS_CANFD_EWL_EW_LIMIT) >> MPFS_CANFD_EWL_EW_LIMIT_SHIFT);
  rec_val = ((rec_tec_reg & MPFS_CANFD_REC_REC_VAL) >> MPFS_CANFD_REC_REC_VAL_SHIFT);
  tec_val = ((rec_tec_reg & MPFS_CANFD_REC_TEC_VAL) >> MPFS_CANFD_REC_TEC_VAL_SHIFT);

	if (ewl_erp_fs_reg & MPFS_CANFD_EWL_ERA)
  {
		if (rec_val < ew_limit && tec_val < ew_limit)
			return CAN_STATE_ERROR_ACTIVE;
		else
			return CAN_STATE_ERROR_WARNING;
	} 
  else if (ewl_erp_fs_reg & MPFS_CANFD_EWL_ERP)
		return CAN_STATE_ERROR_PASSIVE;
	else if (ewl_erp_fs_reg & MPFS_CANFD_EWL_BOF)
		return CAN_STATE_BUS_OFF;

	nwarn("Invalid FPGA CAN-FD error state");
	return CAN_STATE_ERROR_PASSIVE;
}


/****************************************************************************
 * Function: mpfs_read_rec_tec
 * 
 * Description:
 *    Reads FPGA CANFD RX/TX error counter
 *    
 * Input Parameters:
 *    priv  - Reference to the driver state structure
 *    bec   - Pointer to Error counter structure
 * 
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void mpfs_get_rec_tec(FAR struct mpfs_driver_s *priv, struct mpfs_can_berr_counter_s *bec)
{
	uint32_t rec_tec_reg = getreg32(priv->base + MPFS_CANFD_REC_OFFSET);

	bec->rxerr = ((rec_tec_reg & MPFS_CANFD_REC_REC_VAL) >> MPFS_CANFD_REC_REC_VAL_SHIFT);
	bec->txerr = ((rec_tec_reg & MPFS_CANFD_REC_TEC_VAL) >> MPFS_CANFD_REC_TEC_VAL_SHIFT);
}


/****************************************************************************
 * Function: mpfs_err_interrupt
 * 
 * Description:
 *    Error frame ISR
 *    
 * Input Parameters:
 *    priv  - Reference to the driver state structure
 *    isr   - Interrupt status register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_err_interrupt(FAR struct mpfs_driver_s *priv, uint32_t isr)
{
	struct canfd_frame *cf = (struct canfd_frame *)priv->rxdesc;
	
	enum mpfs_can_state_e state;
	struct mpfs_can_berr_counter_s bec;
	uint32_t err_capt_retr_ctr_alc_reg;
  uint32_t error_type, error_pos, alc_id_field, alc_bit;

	ctucan_get_rec_tec(priv, &bec);
	state = ctucan_read_fault_state(priv);
	err_capt_retr_ctr_alc_reg = getreg32(priv->base + MPFS_CANFD_ERR_CAPT_OFFSET);

  error_type = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ERR_TYPE) >> MPFS_CANFD_ERR_CAPT_ERR_TYPE_SHIFT);
  error_pos = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ERR_POS) >> MPFS_CANFD_ERR_CAPT_ERR_POS_SHIFT);
  alc_id_field = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD) >> MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD_SHIFT);
  alc_bit = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ALC_BIT) >> MPFS_CANFD_ERR_CAPT_ALC_BIT_SHIFT);
  
  ninfo("%s: ISR = 0x%08x, rxerr %d, txerr %d, error type %lu, pos %lu, ALC id_field %lu, bit %lu\n",
    __func__, isr, bec.rxerr, bec.txerr, err_type, error_pos, alc_id_field, alc_bit);


	/* EWLI: error warning limit condition met
	 * FCSI: fault confinement state changed
	 * ALI:  arbitration lost (just informative)
	 * BEI:  bus error interrupt
	 */
	if (MPFS_CANFD_INT_STAT_FCSI & isr || MPFS_CANFD_INT_STAT_EWLI & isr)
  {
		if (priv->can.state == state)
			nwarn("%s: current and previous state is the same! (missed interrupt?)\n", __func__);
    else
      ninfo("%s: state changes from %s to %s\n", __func__, can_state_to_str(priv->can.state),
            can_state_to_str(state));

		priv->can.state = state;
    
		switch (state)
    {
		case CAN_STATE_BUS_OFF:
			// TODO : can_bus_off(priv);
      cf->can_id |= CAN_ERR_BUSOFF;
			break;
		case CAN_STATE_ERROR_PASSIVE:
      cf->can_id |= CAN_ERR_CRTL;
      cf->data[1] = (bec.rxerr > 127) ? CAN_ERR_CRTL_RX_PASSIVE : CAN_ERR_CRTL_TX_PASSIVE;
      cf->data[6] = bec.txerr;
      cf->data[7] = bec.rxerr;
			break;
		case CAN_STATE_ERROR_WARNING:
      cf->can_id |= CAN_ERR_CRTL;
      cf->data[1] |= (bec.txerr > bec.rxerr) ? CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
      cf->data[6] = bec.txerr;
      cf->data[7] = bec.rxerr;
			break;
		case CAN_STATE_ERROR_ACTIVE:
			cf->data[1] = CAN_ERR_CRTL_ACTIVE;
			cf->data[6] = bec.txerr;
			cf->data[7] = bec.rxerr;
			break;
		default:
			nwarn("%s: unhandled error state (%d:%s)!\n", __func__, state, can_state_to_str(state));
			break;
		}
	}

	/* Check for Arbitration Lost interrupt */
	if (MPFS_CANFD_INT_STAT_ALI & isr) {
    ninfo("%s: arbitration lost\n", __func__);
    cf->can_id |= CAN_ERR_LOSTARB;
    cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	}

	/* Check for Bus Error interrupt */
	if (FIELD_GET(REG_INT_STAT_BEI, isr)) {
		ninfo("%s: bus error\n", __func__);
    cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
    cf->data[2] = CAN_ERR_PROT_UNSPEC;
    cf->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	}

  // Send to socket interface
  priv->dev.d_len = sizeof(struct canfd_frame);
  priv->dev.d_buf = (uint8_t *)cf;
  NETDEV_RXPACKETS(&priv->dev);
  can_input(&priv->dev);

  /* Point the packet buffer back to the next Tx buffer that will be
   * used during the next write. 
   */
  priv->dev.d_buf = (uint8_t *)priv->txdesc;
}


/****************************************************************************
 * Function: mpfs_fpga_interrupt
 *
 * Description:
 *   Three interrupt sources will vector to this function:
 *   1. CAN MB transmit interrupt handler
 *   2. CAN MB receive interrupt handler
 *   3.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int mpfs_fpga_interrupt(int irq, FAR void *context,
                                     FAR void *arg)
{
  FAR struct mpfs_driver_s *priv = (struct mpfs_driver_s *)arg;

  uint32_t isr, icr, imask;
  int irq_loops;

  for (irq_loops = 0; irq_loops < 10000; irq_loops++)
  {
    /* Get the interrupt status */
    isr  = getreg32(priv->base + MPFS_CANFD_INT_STAT_OFFSET);
    
    if (!isr)
      return irq_loops ? OK : -1;

    /* Receive Buffer Not Empty Interrupt */
    if (isr & MPFS_CANFD_INT_STAT_RBNEI)
    {
      /* Mask RXBNEI the first, then clear interrupt and schedule NAPI. Even if
      * another IRQ fires, RBNEI will always be 0 (masked).
      */
      icr = MPFS_CANFD_INT_STAT_RBNEI;
      putreg32(icr, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);
      putreg32(icr, priv->base + MPFS_CANFD_INT_STAT_OFFSET);

      /* TODO: check receive routine */  
      mpfs_receive(priv, isr);
    }

    /* TXT Buffer HW Command Interrupt */
    if (isr & MPFS_CANFD_INT_STAT_TXBHCI)
    {
      // /* Disable further TXT Buffer HW Command Interrupts. TODO: should we mask TXT INT here?
      //  * There can be no race condition here
      //  */
      // flags  = getreg32(priv->base + mpfs_CAN_IMASK1_OFFSET);
      // flags &= ~(IFLAG1_TX);
      // putreg32(flags, priv->base + mpfs_CAN_IMASK1_OFFSET);
      work_queue(CANWORK, &priv->irqwork, mpfs_txdone_work, priv, 0);
    }

    /* Error Interrupts */
    if (isr & MPFS_CANFD_INT_STAT_EWLI ||
        isr & MPFS_CANFD_INT_STAT_FCSI ||
        isr & MPFS_CANFD_INT_STAT_ALI ||
        isr & MPFS_CANFD_INT_STAT_BEI)
    {
      icr = isr & (MPFS_CANFD_INT_STAT_EWLI | MPFS_CANFD_INT_STAT_FCSI | MPFS_CANFD_INT_STAT_ALI |
                    MPFS_CANFD_INT_STAT_BEI);
      putreg32(icr, priv->base + MPFS_CANFD_INT_STAT_OFFSET);
      mpfs_err_interrupt(priv, isr)
    }
  }

  nerr("%s: stuck interrupt (isr=%08x), stopping\n", __func__, isr);

  // TODO: print out nerr for txb status

  imask = 0xFFFFFFFF;
  putreg32(imask, priv->base + MPFS_CANFD_INT_ENA_CLR_INT_ENA_CLR);
  putreg32(imask, priv->base + MPFS_CANFD_INT_ENA_SET_INT_ENA_SET);      

  return OK;
}


/****************************************************************************
 * Function: mpfs_txavail_work
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

static void mpfs_txavail_work(FAR void *arg)
{
  FAR struct mpfs_driver_s *priv = (FAR struct mpfs_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (mpfs_waitesr2_change(priv->base,
                             (CAN_ESR2_IMB | CAN_ESR2_VPS),
                             (CAN_ESR2_IMB | CAN_ESR2_VPS)))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_timer(&priv->dev, 0, mpfs_txpoll);
        }
    }

  net_unlock();
}


/****************************************************************************
 * Function: mpfs_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int mpfs_txavail(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      mpfs_txavail_work(priv);
    }

  return OK;
}


/****************************************************************************
 * Function: mpfs_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *   OK for success and -ETIMEDOUT for failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_reset(struct mpfs_driver_s *priv)
{
  uint32_t regval;
  uint32_t i = 100;

  /* Reset FPGA CANFD device */
  putreg32(MPFS_CANFD_MODE_RST, priv->base + MPFS_CANFD_MODE_OFFSET);
  
  /* Clear the RX FFW BUFFERED flag */
  regval = 1 << MPFS_CANFD_FLAG_RX_FFW_BUFFERED;
  priv->drv_flags &= ~regval;

  /* Check if the device is up again */
	do {
		uint16_t device_id = (getreg32(priv->base + MPFS_CANFD_DEVICE_ID_OFFSET) & 
                          MPFS_CANFD_DEVICE_ID_DEVICE_ID) >> MPFS_CANFD_DEVICE_ID_DEVICE_ID_SHIFT;
		if (device_id == MPFS_CANFD_ID)
			return OK;
		if (!i--) {
			nwarn("%s: device did not leave reset\n", __func__);
			return -ETIMEDOUT;
		}
		nxsig_usleep(200);
	} while (1);

  /* Initialize all MB rx and tx TODO: needed?? */

  /* TODO: module config register settings?? */

  /* TODO: control 2 register settings?? */
}


/****************************************************************************
 * Function: mpfs_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  if (mpfs_chip_start(priv) < 0)
    {
      nerr("chip start failed");
      return -1;
    }

  priv->bifup = true;

  priv->txdesc = (struct canfd_frame *)&g_tx_pool;
  priv->rxdesc = (struct canfd_frame *)&g_rx_pool;

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */
  up_enable_irq(priv->config->canfd_fpga_irq);

  return OK;
}

/****************************************************************************
 * Function: mpfs_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  // Stop chip
  mpfs_chip_stop(priv);
  
  priv->bifup = false;
  return OK;
}


/****************************************************************************
 * Function: mpfs_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
static int mpfs_ioctl(struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct mpfs_driver_s *priv =
      (FAR struct mpfs_driver_s *)dev->d_private;

  int ret;

  switch (cmd)
    {
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->arbi_timing.bitrate / 1000; /* kbit/s */
          req->arbi_samplep = priv->arbi_timing.samplep;
#ifdef CONFIG_NET_CAN_CANFD
          req->data_bitrate = priv->data_timing.bitrate / 1000; /* kbit/s */
          req->data_samplep = priv->data_timing.samplep;
#else
          req->data_bitrate = 0;
          req->data_samplep = 0;
#endif
          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);

          struct flexcan_timeseg arbi_timing;
          arbi_timing.bitrate = req->arbi_bitrate * 1000;
          arbi_timing.samplep = req->arbi_samplep;

          if (mpfs_bitratetotimeseg(&arbi_timing, 10, 0))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }

#ifdef CONFIG_NET_CAN_CANFD
          struct flexcan_timeseg data_timing;
          data_timing.bitrate = req->data_bitrate * 1000;
          data_timing.samplep = req->data_samplep;

          if (ret == OK && mpfs_bitratetotimeseg(&data_timing, 10, 1))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }
#endif

          if (ret == OK)
            {
              /* Reset CAN controller and start with new timings */

              priv->arbi_timing = arbi_timing;
#ifdef CONFIG_NET_CAN_CANFD
              priv->data_timing = data_timing;
#endif
              mpfs_ifup(dev);
            }
        }
        break;

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
 * Function: mpfs_canfd_init
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Returned Value:
 *   On success, a pointer to the MPFS CAN-FD driver is
 *   returned. NULL is returned on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int mpfs_canfd_init()
{
  struct mpfs_driver_s *priv;

  priv         = &g_canfd;
  memset(priv, 0, sizeof(struct mpfs_driver_s));
  priv->base   = MPFS_CANFD_BASE;
  priv->config = &mpfs_fpga_canfd_config;

  /* TODO: Default bitrate configuration?? */
  if (!mpfs_bitratetotimeseg(&priv->data_timing, 1, 1))
    {
      nerr("ERROR: Invalid CAN data phase timings please try another "
           "sample point or refer to the reference manual\n");
      return -1;
    }


  /* Initialize the CAN common private data structure */
  priv->can.state = CAN_STATE_ERROR_ACTIVE;
  priv->can.bittiming_const = &mpfs_can_bit_timing_max;
	priv->can.data_bittiming_const = &mpfs_can_bit_timing_data_max;
	
  priv->can.do_set_mode = mpfs_can_do_set_mode;
  priv->can.do_get_berr_counter = mpfs_can_get_berr_counter;

  /* Needed for timing adjustment to be performed as soon as possible */
  priv->can.do_set_bittiming = mpfs_can_set_bittiming;
  priv->can.do_set_data_bittiming = mpfs_can_set_data_bittiming;
  
  priv->can.ctrlmode_support = CAN_CTRLMODE_LOOPBACK					
                              | CAN_CTRLMODE_LISTENONLY
                              | CAN_CTRLMODE_FD
                              | CAN_CTRLMODE_PRESUME_ACK
                              | CAN_CTRLMODE_BERR_REPORTING
                              | CAN_CTRLMODE_FD_NON_ISO
                              | CAN_CTRLMODE_ONE_SHOT;


  /* Attach the interrupt handler */
  if (irq_attach(priv->config->canfd_fpga_irq, mpfs_fpga_interrupt, priv))
  {
    /* We could not attach the ISR to the interrupt */
    nerr("ERROR: Failed to attach to FPGA CANFD IRQ\n");
    return -EAGAIN;
  }


  /* Initialize the driver structure */
  priv->dev.d_ifup    = mpfs_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;     /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = priv;              /* Used to recover private state from dev */
  priv->rx            = (struct mb_s *)(priv->base + mpfs_CAN_MB_OFFSET); // todo
  priv->tx            = (struct mb_s *)(priv->base + mpfs_CAN_MB_OFFSET +
                          (sizeof(struct mb_s) * RXMBCOUNT));             // todo


	/* Getting the can_clk info */
	if (!can_clk_rate) {
		priv->can_clk = devm_clk_get(dev, NULL);
		if (IS_ERR(priv->can_clk)) {
			dev_err(dev, "Device clock not found.\n");
			ret = PTR_ERR(priv->can_clk);
			goto err_free;
		}
		can_clk_rate = clk_get_rate(priv->can_clk);
	}
	priv->can.clock.freq = can_clk_rate;


  /* Reset chip */
  if (!mpfs_reset(priv))
    return -1;
  ninfo("%s: CAN-FD driver init done\n", __func__);

  
  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling mpfs_ifdown().
   */
  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */
  netdev_register(&priv->dev, NET_LL_CAN);

  return OK;
}


/****************************************************************************
 * Name: riscv_netinitialize
 *
 * Description:
 *   Initialize the enabled CAN device interfaces.  If there are more
 *   different network devices in the chip, then board-specific logic will
 *   have to provide this function to determine which, if any, network
 *   devices should be initialized.
 *
 ****************************************************************************/

#ifndef CONFIG_NETDEV_LATEINIT
void riscv_netinitialize(void)
{
  mpfs_canfd_init();
}
#endif
