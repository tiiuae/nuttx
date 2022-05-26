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


/* Interrupt flags for RX fifo */
#define IFLAG1_RXFIFO               (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)

#define CTUCAN_STATE_TO_TEXT_ENTRY(st) #st

static int peak_tx_mailbox_index_ = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/*
 * CAN operational and error states
 */
enum mpfs_can_state {
	CAN_STATE_ERROR_ACTIVE = 0,	/* RX/TX error count < 96 */
	CAN_STATE_ERROR_WARNING,	/* RX/TX error count < 128 */
	CAN_STATE_ERROR_PASSIVE,	/* RX/TX error count < 256 */
	CAN_STATE_BUS_OFF,		/* RX/TX error count >= 256 */
	CAN_STATE_STOPPED,		/* Device is stopped */
	CAN_STATE_SLEEPING,		/* Device is sleeping */
	CAN_STATE_MAX
};

static const char * const mpfs_can_state_strings[CAN_STATE_MAX] = {
	CTUCAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_ACTIVE),
	CTUCAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_WARNING),
	CTUCAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_PASSIVE),
	CTUCAN_STATE_TO_TEXT_ENTRY(CAN_STATE_BUS_OFF),
	CTUCAN_STATE_TO_TEXT_ENTRY(CAN_STATE_STOPPED),
	CTUCAN_STATE_TO_TEXT_ENTRY(CAN_STATE_SLEEPING)
};

/*
 * CAN bus error counters
 */
struct mpfs_can_berr_counter {
	__u16 txerr;
	__u16 rxerr;
};

union cs_e
{
  volatile uint32_t cs;
  struct
  {
    volatile uint32_t time_stamp : 16;
    volatile uint32_t dlc : 4;
    volatile uint32_t rtr : 1;
    volatile uint32_t ide : 1;
    volatile uint32_t srr : 1;
    volatile uint32_t res : 1;
    volatile uint32_t code : 4;
    volatile uint32_t res2 : 1;
    volatile uint32_t esi : 1;
    volatile uint32_t brs : 1;
    volatile uint32_t edl : 1;
  };
};

union id_e
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

union data_e
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
  union cs_e cs;
  union id_e id;
#ifdef CONFIG_NET_CAN_CANFD
  union data_e data[16];
#else
  union data_e data[2];
#endif
};


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
struct mpfs_timeseg
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
  uint32_t base;                /* CANFD FPGA base address */
  bool bifup;                   /* true:ifup false:ifdown */

  struct work_s irqwork;        /* For deferring interrupt work to the work wq */
  struct work_s pollwork;       /* For deferring poll work to the work wq */

  struct canfd_frame *txdesc;   /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc;   /* A pointer to the list of RX descriptors */

  enum mpfs_can_state state;              /* CAN state

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

static uint8_t g_tx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];

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
 *   Converts CAN controller state code to corresponding text
 *
 * Input Parameters:
 *   state  - CAN controller state code
 *
 * Returned Value:
 *   Pointer to string representation of the error state
 *
 ****************************************************************************/

static const char *can_state_to_str(enum can_state state)
{
	const char *txt = NULL;

	if (state >= 0 && state < CAN_STATE_MAX)
		txt = mpfs_can_state_strings[state];

  return txt ? txt : "UNKNOWN";
}


/****************************************************************************
 * Function: mpfs_tx_interrupt
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *   We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/
static void mpfs_tx_interrupt(FAR void *arg)
{

	struct ctucan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	bool first = true;
	bool some_buffers_processed;
	unsigned long flags;
	enum ctucan_txtb_status txtb_status;
	u32 txtb_id;

	/*  read tx_status
	 *  if txb[n].finished (bit 2)
	 *	if ok -> echo
	 *	if error / aborted -> ?? (find how to handle oneshot mode)
	 *	txb_tail++
	 */
	do {
		spin_lock_irqsave(&priv->tx_lock, flags);

		some_buffers_processed = false;
		while ((int)(priv->txb_head - priv->txb_tail) > 0) {
			txtb_id = priv->txb_tail % priv->ntxbufs;
			txtb_status = ctucan_get_tx_status(priv, txtb_id);

			ctucan_netdev_dbg(ndev, "TXI: TXB#%u: status 0x%x\n", txtb_id, txtb_status);

			switch (txtb_status) {
			case TXT_TOK:
				ctucan_netdev_dbg(ndev, "TXT_OK\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
				can_get_echo_skb(ndev, txtb_id, NULL);
#else /* < 5.12.0 */
				can_get_echo_skb(ndev, txtb_id);
#endif /* < 5.12.0 */
				stats->tx_packets++;
				break;
			case TXT_ERR:
				/* This indicated that retransmit limit has been reached. Obviously
				 * we should not echo the frame, but also not indicate any kind of
				 * error. If desired, it was already reported (possible multiple
				 * times) on each arbitration lost.
				 */
				netdev_warn(ndev, "TXB in Error state\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
				can_free_echo_skb(ndev, txtb_id, NULL);
#else /* < 5.12.0 */
				can_free_echo_skb(ndev, txtb_id);
#endif /* < 5.12.0 */
				stats->tx_dropped++;
				break;
			case TXT_ABT:
				/* Same as for TXT_ERR, only with different cause. We *could*
				 * re-queue the frame, but multiqueue/abort is not supported yet
				 * anyway.
				 */
				netdev_warn(ndev, "TXB in Aborted state\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
				can_free_echo_skb(ndev, txtb_id, NULL);
#else /* < 5.12.0 */
				can_free_echo_skb(ndev, txtb_id);
#endif /* < 5.12.0 */
				stats->tx_dropped++;
				break;
			default:
				/* Bug only if the first buffer is not finished, otherwise it is
				 * pretty much expected.
				 */
				if (first) {
					netdev_err(ndev,
						   "BUG: TXB#%u not in a finished state (0x%x)!\n",
						   txtb_id, txtb_status);
					spin_unlock_irqrestore(&priv->tx_lock, flags);
					/* do not clear nor wake */
					return;
				}
				goto clear;
			}
			priv->txb_tail++;
			first = false;
			some_buffers_processed = true;
			/* Adjust priorities *before* marking the buffer as empty. */
			ctucan_rotate_txb_prio(ndev);
			ctucan_give_txtb_cmd(priv, TXT_CMD_SET_EMPTY, txtb_id);
		}
clear:
		spin_unlock_irqrestore(&priv->tx_lock, flags);

		/* If no buffers were processed this time, we cannot clear - that would introduce
		 * a race condition.
		 */
		if (some_buffers_processed) {
			/* Clear the interrupt again. We do not want to receive again interrupt for
			 * the buffer already handled. If it is the last finished one then it would
			 * cause log of spurious interrupt.
			 */
			ctucan_write32(priv, CTUCANFD_INT_STAT, REG_INT_STAT_TXBHCI);
		}
	} while (some_buffers_processed);

	can_led_event(ndev, CAN_LED_EVENT_TX);

	spin_lock_irqsave(&priv->tx_lock, flags);

	/* Check if at least one TX buffer is free */
	if (CTU_CAN_FD_TXTNF(priv))
		netif_wake_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);
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

static void mpfs_get_rec_tec(FAR struct mpfs_driver_s *priv, struct mpfs_can_berr_counter *bec)
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
	
	enum mpfs_can_state state;
	struct mpfs_can_berr_counter bec;
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
      cf->data[1] = (bec.rxerr > 127) ?
          CAN_ERR_CRTL_RX_PASSIVE :
          CAN_ERR_CRTL_TX_PASSIVE;
      cf->data[6] = bec.txerr;
      cf->data[7] = bec.rxerr;
			break;
		case CAN_STATE_ERROR_WARNING:
			priv->can.can_stats.error_warning++;
			if (skb) {
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] |= (bec.txerr > bec.rxerr) ?
					CAN_ERR_CRTL_TX_WARNING :
					CAN_ERR_CRTL_RX_WARNING;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
			}
			break;
		case CAN_STATE_ERROR_ACTIVE:
			cf->data[1] = CAN_ERR_CRTL_ACTIVE;
			cf->data[6] = bec.txerr;
			cf->data[7] = bec.rxerr;
			break;
		default:
			netdev_warn(ndev, "unhandled error state (%d:%s)!\n",
				    state, ctucan_state_to_str(state));
			break;
		}
	}

	/* Check for Arbitration Lost interrupt */
	if (FIELD_GET(REG_INT_STAT_ALI, isr)) {
		if (dologerr)
			netdev_info(ndev, "arbitration lost\n");
		priv->can.can_stats.arbitration_lost++;
		if (skb) {
			cf->can_id |= CAN_ERR_LOSTARB;
			cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
		}
	}

	/* Check for Bus Error interrupt */
	if (FIELD_GET(REG_INT_STAT_BEI, isr)) {
		netdev_info(ndev, "bus error\n");
		priv->can.can_stats.bus_error++;
		stats->rx_errors++;
		if (skb) {
			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
			cf->data[2] = CAN_ERR_PROT_UNSPEC;
			cf->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
		}
	}

  // TODO: skb netif_rx
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
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  if (!mpfs_initialize(priv))
    {
      nerr("initialize failed");
      return -1;
    }

  priv->bifup = true;

#ifdef CONFIG_NET_CAN_CANFD
  priv->txdesc = (struct canfd_frame *)&g_tx_pool;
  priv->rxdesc = (struct canfd_frame *)&g_rx_pool;
#else
  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;
#endif

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */

  up_enable_irq(priv->config->bus_irq);
  up_enable_irq(priv->config->error_irq);
  if (priv->config->lprx_irq > 0)
    {
      up_enable_irq(priv->config->lprx_irq);
    }

  up_enable_irq(priv->config->mb_irq);

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
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  mpfs_reset(priv);

  priv->bifup = false;
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
 * Function: mpfs_initalize
 *
 * Description:
 *   Initialize FLEXCAN device
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_initialize(struct mpfs_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  /* initialize CAN device */

  mpfs_setenable(priv->base, 0);

  /* Set SYS_CLOCK src */

  regval  = getreg32(priv->base + mpfs_CAN_CTRL1_OFFSET);
  regval |= CAN_CTRL1_CLKSRC;
  putreg32(regval, priv->base + mpfs_CAN_CTRL1_OFFSET);

  mpfs_setenable(priv->base, 1);

  mpfs_reset(priv);

  /* Enter freeze mode */

  mpfs_setfreeze(priv->base, 1);
  if (!mpfs_waitfreezeack_change(priv->base, 1))
    {
      ninfo("FLEXCAN: freeze fail\n");
      return -1;
    }

  /* Reset CTRL1 register to reset value */

  regval  = getreg32(priv->base + mpfs_CAN_CTRL1_OFFSET);
  regval &= ~(CAN_CTRL1_LOM | CAN_CTRL1_LBUF | CAN_CTRL1_TSYN |
              CAN_CTRL1_BOFFREC | CAN_CTRL1_SMP | CAN_CTRL1_RWRNMSK |
              CAN_CTRL1_TWRNMSK | CAN_CTRL1_LPB | CAN_CTRL1_ERRMSK |
              CAN_CTRL1_BOFFMSK);
  putreg32(regval, priv->base + mpfs_CAN_CTRL1_OFFSET);

#ifndef CONFIG_NET_CAN_CANFD
  regval  = getreg32(priv->base + mpfs_CAN_CTRL1_OFFSET);

  regval &= ~(CAN_CTRL1_TIMINGMSK); /* Reset timings */

  regval |= CAN_CTRL1_PRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
            CAN_CTRL1_PROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
            CAN_CTRL1_PSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
            CAN_CTRL1_PSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
            CAN_CTRL1_RJW(1);                              /* Resynchronization jump width */
  putreg32(regval, priv->base + mpfs_CAN_CTRL1_OFFSET);

#else

  regval  = CAN_CBT_BTF |                                 /* Enable extended bit timing
                                                           * configurations for CAN-FD for setting up
                                                           * separately nominal and data phase */
            CAN_CBT_EPRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
            CAN_CBT_EPROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
            CAN_CBT_EPSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
            CAN_CBT_EPSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
            CAN_CBT_ERJW(1);                              /* Resynchronization jump width */
  putreg32(regval, priv->base + mpfs_CAN_CBT_OFFSET);

  /* Enable CAN FD feature */

  regval  = getreg32(priv->base + mpfs_CAN_MCR_OFFSET);
  regval |= CAN_MCR_FDEN;
  putreg32(regval, priv->base + mpfs_CAN_MCR_OFFSET);

  regval  = CAN_FDCBT_FPRESDIV(priv->data_timing.presdiv) |  /* Prescaler divisor factor of 1 */
            CAN_FDCBT_FPROPSEG(priv->data_timing.propseg) |  /* Propagation
                                                              * segment (only register that doesn't add 1) */
            CAN_FDCBT_FPSEG1(priv->data_timing.pseg1) |      /* Phase buffer segment 1 */
            CAN_FDCBT_FPSEG2(priv->data_timing.pseg2) |      /* Phase buffer segment 2 */
            CAN_FDCBT_FRJW(priv->data_timing.pseg2);         /* Resynchorinzation jump width same as PSEG2 */
  putreg32(regval, priv->base + mpfs_CAN_FDCBT_OFFSET);

  /* Additional CAN-FD configurations */

  regval  = CAN_FDCTRL_FDRATE |     /* Enable bit rate switch in data phase of frame */
            CAN_FDCTRL_TDCEN |      /* Enable transceiver delay compensation */
            CAN_FDCTRL_TDCOFF(5) |  /* Setup 5 cycles for data phase sampling delay */
            CAN_FDCTRL_MBDSR0(3);   /* Setup 64 bytes per message buffer (7 MB's) */
  putreg32(regval, priv->base + mpfs_CAN_FDCTRL_OFFSET);

  regval  = getreg32(priv->base + mpfs_CAN_CTRL2_OFFSET);
  regval |= CAN_CTRL2_ISOCANFDEN;
  putreg32(regval, priv->base + mpfs_CAN_CTRL2_OFFSET);
#endif

  for (i = TXMBCOUNT; i < TOTALMBCOUNT; i++)
    {
      priv->rx[i].id.w = 0x0;

      /* FIXME sometimes we get a hard fault here */
    }

  putreg32(0x0, priv->base + mpfs_CAN_RXFGMASK_OFFSET);

  for (i = 0; i < mpfs_CAN_RXIMR_COUNT; i++)
    {
      putreg32(0, priv->base + mpfs_CAN_RXIMR_OFFSET(i));
    }

  for (i = 0; i < RXMBCOUNT; i++)
    {
      ninfo("Set MB%" PRIu32 " to receive %p\n", i, &priv->rx[i]);
      priv->rx[i].cs.edl = 0x1;
      priv->rx[i].cs.brs = 0x1;
      priv->rx[i].cs.esi = 0x0;
      priv->rx[i].cs.code = 4;
      priv->rx[i].cs.srr = 0x0;
      priv->rx[i].cs.ide = 0x1;
      priv->rx[i].cs.rtr = 0x0;
    }

  putreg32(IFLAG1_RX, priv->base + mpfs_CAN_IFLAG1_OFFSET);
  putreg32(IFLAG1_RX, priv->base + mpfs_CAN_IMASK1_OFFSET);

  /* Exit freeze mode */

  mpfs_setfreeze(priv->base, 0);
  if (!mpfs_waitfreezeack_change(priv->base, 0))
    {
      ninfo("FLEXCAN: unfreeze fail\n");
      return -1;
    }

  return 1;
}

/****************************************************************************
 * Function: mpfs_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mpfs_reset(struct mpfs_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  regval  = getreg32(priv->base + mpfs_CAN_MCR_OFFSET);
  regval |= CAN_MCR_SOFTRST;
  putreg32(regval, priv->base + mpfs_CAN_MCR_OFFSET);

  if (!mpfs_waitmcr_change(priv->base, CAN_MCR_SOFTRST, 0))
    {
      nerr("Reset failed");
      return;
    }

  regval  = getreg32(priv->base + mpfs_CAN_MCR_OFFSET);
  regval &= ~(CAN_MCR_SUPV);
  putreg32(regval, priv->base + mpfs_CAN_MCR_OFFSET);

  /* Initialize all MB rx and tx */

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      ninfo("MB %" PRIu32 " %p\n", i, &priv->rx[i]);
      ninfo("MB %" PRIu32 " %p\n", i, &priv->rx[i].id.w);
      priv->rx[i].cs.cs = 0x0;
      priv->rx[i].id.w = 0x0;
      priv->rx[i].data[0].w00 = 0x0;
      priv->rx[i].data[1].w00 = 0x0;
    }

  regval  = getreg32(priv->base + mpfs_CAN_MCR_OFFSET);
  regval |= CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_SRXDIS |
            CAN_MCR_IRMQ | CAN_MCR_AEN |
            (((TOTALMBCOUNT - 1) << CAN_MCR_MAXMB_SHIFT) &
            CAN_MCR_MAXMB_MASK);
  putreg32(regval, priv->base + mpfs_CAN_MCR_OFFSET);

  regval  = CAN_CTRL2_RRS | CAN_CTRL2_EACEN;
  putreg32(regval, priv->base + mpfs_CAN_CTRL2_OFFSET);

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      putreg32(0, priv->base + mpfs_CAN_RXIMR_OFFSET(i));
    }

  /* Filtering catchall */

  putreg32(0x3fffffff, priv->base + mpfs_CAN_RX14MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + mpfs_CAN_RX15MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + mpfs_CAN_RXMGMASK_OFFSET);
  putreg32(0x0, priv->base + mpfs_CAN_RXFGMASK_OFFSET);
}

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
  int ret;

  priv         = &g_canfd;
  memset(priv, 0, sizeof(struct mpfs_driver_s));
  priv->base   = MPFS_CANFD_BASE;
  priv->config = &mpfs_fpga_canfd_config;
  priv->state = CAN_STATE_ERROR_ACTIVE;

  /* TODO */
  /* Default bitrate configuration */
  priv->arbi_timing.bitrate = CONFIG_FLEXCAN1_ARBI_BITRATE;
  priv->arbi_timing.samplep = CONFIG_FLEXCAN1_ARBI_SAMPLEP;
  priv->data_timing.bitrate = CONFIG_FLEXCAN1_DATA_BITRATE;
  priv->data_timing.samplep = CONFIG_FLEXCAN1_DATA_SAMPLEP;


  if (!mpfs_bitratetotimeseg(&priv->data_timing, 1, 1))
    {
      nerr("ERROR: Invalid CAN data phase timings please try another "
           "sample point or refer to the reference manual\n");
      return -1;
    }




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

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling mpfs_ifdown().
   */

  ninfo("callbacks done\n");

  mpfs_initialize(priv);  // todo

  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
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
