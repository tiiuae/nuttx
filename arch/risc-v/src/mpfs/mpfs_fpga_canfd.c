/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_fpga_canfd.c
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

#include "hardware/mpfs_fpga_canfd.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef OK
#  define OK 0
#endif

/* This module only compiles if the CAN-FD IP core instance
 * is configured to the FPGA
 */

#ifndef CONFIG_MPFS_CANFD
#  error This should not be compiled as CAN-FD FPGA block is not defined
#endif

/* This module only compiles if Nuttx socketCAN interface supports CANFD */

#ifndef CONFIG_NET_CAN_CANFD
#  error This should not be compiled as CAN-FD driver relies on socket CAN
#endif

/* Clock reset and enabling */

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#define CANWORK                 LPWORK

#define MPFS_CANFD_ID           0xCAFD

/* For allocating the tx and rx CAN-FD frame buffer */

#define POOL_SIZE               1
#define TIMESTAMP_SIZE          sizeof(struct timeval)  /* To support
                                                         * timestamping frame */

/* For bittiming calculation */

#define CAN_CALC_MAX_ERROR      50 /* in one-tenth of a percent */
#define CAN_CALC_SYNC_SEG       1

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
/* CAN hw filter support */

#define HW_FILTER_A             0
#define HW_FILTER_B             1
#define HW_FILTER_C             2
#define HW_FILTER_RANGE         3

#define CAN_STD_ID              0
#define CAN_EXT_ID              1
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/* Special address description flags for the CAN_ID */

#define CAN_EFF_FLAG            0x80000000  /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG            0x40000000  /* remote transmission request */
#define CAN_ERR_FLAG            0x20000000  /* error message frame */

/* Valid bits in CAN ID for frame formats */

#define CAN_SFF_MASK            0x000007FF /* standard frame format (SFF) */
#define CAN_EFF_MASK            0x1FFFFFFF /* extended frame format (EFF) */
#define CAN_ERR_MASK            0x1FFFFFFF /* omit EFF, RTR, ERR flags */

/* CAN control mode */

#define CAN_CTRLMODE_LOOPBACK       0x01  /* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY     0x02  /* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES      0x04  /* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT       0x08  /* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING 0x10  /* Bus-error reporting */
#define CAN_CTRLMODE_FD             0x20  /* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK    0x40  /* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO     0x80  /* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC    0x100 /* Classic CAN DLC option */
#define CAN_CTRLMODE_TDC_AUTO       0x200 /* CAN transiver automatically
                                           * calculates TDCV */
#define CAN_CTRLMODE_TDC_MANUAL     0x400 /* TDCV is manually set up by user */

/* TXT buffer */

enum mpfs_can_txtb_status
{
  TXT_NOT_EXIST       = 0x0,
  TXT_RDY             = 0x1,
  TXT_TRAN            = 0x2,
  TXT_ABTP            = 0x3,
  TXT_TOK             = 0x4,
  TXT_ERR             = 0x6,
  TXT_ABT             = 0x7,
  TXT_ETY             = 0x8,
};

enum mpfs_can_txtb_command
{
  TXT_CMD_SET_EMPTY   = 0x01,
  TXT_CMD_SET_READY   = 0x02,
  TXT_CMD_SET_ABORT   = 0x04
};

/****************************************************************************
 * Utility definitions
 ****************************************************************************/

#define MPFS_CAN_STATE_TO_TEXT_ENTRY(st) #st

#define min(a, b)  (a) < (b) ? a : b

#define max(a, b)  (a) > (b) ? a : b

#define clamp(val, lo, hi)  min(max(val, lo), hi)

/**
 * do_div - returns 2 values: calculate remainder and update new dividend
 * @n: pointer to uint64_t dividend (will be updated)
 * @base: uint32_t divisor
 *
 * Summary:
 * ``uint32_t remainder = *n % base;``
 * ``*n = *n / base;``
 *
 * Return: (uint32_t)remainder
 *
 * NOTE: macro parameter @n is evaluated multiple times,
 * beware of side effects!
 */

# define do_div(n, base) ({       \
  uint32_t __base = (base);       \
  uint32_t __rem;         \
  __rem = ((uint64_t)(n)) % __base;   \
  (n) = ((uint64_t)(n)) / __base;     \
  __rem;            \
 })

#define MPFS_CAN_FD_TXTNF(priv) \
  (!!(getreg32(priv->base + MPFS_CANFD_STATUS_OFFSET) & MPFS_CANFD_STATUS_TXNF))
#define MPFS_CAN_FD_ENABLED(priv) \
  (!!(getreg32(priv->base + MPFS_CANFD_MODE_OFFSET) & MPFS_CANFD_MODE_ENA))

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

/* CAN operational and error states */

/* CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */

struct mpfs_can_bittiming_s
{
  uint32_t bitrate;       /* Bit-rate in bits/second */
  uint32_t sample_point;  /* Sample point in one-tenth of a percent */
  uint32_t tq;            /* Time quanta (TQ) in nanoseconds */
  uint32_t prop_seg;      /* Propagation segment in TQs */
  uint32_t phase_seg1;    /* Phase buffer segment 1 in TQs */
  uint32_t phase_seg2;    /* Phase buffer segment 2 in TQs */
  uint32_t sjw;           /* Synchronisation jump width in TQs */
  uint32_t brp;           /* Bit-rate prescaler */
};

/* CAN harware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters
 */

struct mpfs_can_bittiming_const_s
{
  char name[16];          /* Name of the CAN controller hardware */
  uint32_t tseg1_min;     /* Time segment 1 = prop_seg + phase_seg1 */
  uint32_t tseg1_max;
  uint32_t tseg2_min;     /* Time segment 2 = phase_seg2 */
  uint32_t tseg2_max;
  uint32_t sjw_max;       /* Synchronisation jump width */
  uint32_t brp_min;       /* Bit-rate prescaler */
  uint32_t brp_max;
  uint32_t brp_inc;
};

static const struct mpfs_can_bittiming_const_s mpfs_can_bit_timing_max =
{
  .name = "mpfs_can_fd",
  .tseg1_min = 2,
  .tseg1_max = 190,
  .tseg2_min = 1,
  .tseg2_max = 63,
  .sjw_max = 31,
  .brp_min = 1,
  .brp_max = 8,
  .brp_inc = 1,
};

static const struct mpfs_can_bittiming_const_s mpfs_can_bit_timing_data_max =
{
  .name = "mpfs_can_fd",
  .tseg1_min = 2,
  .tseg1_max = 94,
  .tseg2_min = 1,
  .tseg2_max = 31,
  .sjw_max = 31,
  .brp_min = 1,
  .brp_max = 2,
  .brp_inc = 1,
};

struct mpfs_can_clock_s
{
  uint32_t freq;     /* CAN system clock frequency in Hz */
};

enum mpfs_can_state_e
{
  CAN_STATE_ERROR_ACTIVE = 0, /* RX/TX error count < 96 */
  CAN_STATE_ERROR_WARNING,    /* RX/TX error count < 128 */
  CAN_STATE_ERROR_PASSIVE,    /* RX/TX error count < 256 */
  CAN_STATE_BUS_OFF,          /* RX/TX error count >= 256 */
  CAN_STATE_STOPPED,          /* Device is stopped */
  CAN_STATE_SLEEPING,         /* Device is sleeping */
  CAN_STATE_MAX
};

static const char * const mpfs_can_state_strings[CAN_STATE_MAX] =
{
  MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_ACTIVE),
  MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_WARNING),
  MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_ERROR_PASSIVE),
  MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_BUS_OFF),
  MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_STOPPED),
  MPFS_CAN_STATE_TO_TEXT_ENTRY(CAN_STATE_SLEEPING)
};

struct mpfs_can_ctrlmode_s
{
  uint32_t mask;
  uint32_t flags;
};

struct mpfs_can_berr_counter_s
{
  uint16_t txerr;
  uint16_t rxerr;
};

struct mpfs_can_device_stats_s
{
  uint32_t bus_error;         /* Bus errors */
  uint32_t error_warning;     /* Changes to error warning state */
  uint32_t error_passive;     /* Changes to error passive state */
  uint32_t bus_off;           /* Changes to bus off state */
  uint32_t arbitration_lost;  /* Arbitration lost errors */
  uint32_t restarts;          /* CAN controller re-starts */
};

/* CAN common private data */

struct mpfs_can_priv_s
{
  struct mpfs_can_device_stats_s can_stats;
  struct mpfs_can_bittiming_s bittiming;
  struct mpfs_can_bittiming_s data_bittiming;
  const struct mpfs_can_bittiming_const_s *bittiming_const;
  const struct mpfs_can_bittiming_const_s *data_bittiming_const;
  struct mpfs_can_clock_s clock;

  enum mpfs_can_state_e state;
  uint32_t ctrlmode;
};

/****************************************************************************
 * CANFD Frame Format
 ****************************************************************************/

/* CAN frame format memory map */

enum mpfs_canfd_can_frame_format
{
  MPFS_CANFD_FRAME_FORMAT_W_OFFSET       = 0x0,
  MPFS_CANFD_IDENTIFIER_W_OFFSET         = 0x4,
  MPFS_CANFD_TIMESTAMP_L_W_OFFSET        = 0x8,
  MPFS_CANFD_TIMESTAMP_U_W_OFFSET        = 0xc,
  MPFS_CANFD_DATA_1_4_W_OFFSET           = 0x10,
  MPFS_CANFD_DATA_5_8_W_OFFSET           = 0x14,
  MPFS_CANFD_DATA_61_64_W_OFFSET         = 0x4c,
};

/* CANFD_Frame_format memory region */

/*  FRAME_FORMAT_W registers */

#define MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT           (0)
#define MPFS_CANFD_FRAME_FORMAT_W_DLC                 (0x0F << \
  MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT)
#define MPFS_CANFD_FRAME_FORMAT_W_RTR                 (1 << 5)
#define MPFS_CANFD_FRAME_FORMAT_W_IDE                 (1 << 6)
#define MPFS_CANFD_FRAME_FORMAT_W_FDF                 (1 << 7)
#define MPFS_CANFD_FRAME_FORMAT_W_BRS                 (1 << 9)
#define MPFS_CANFD_FRAME_FORMAT_W_ESI_RSV             (1 << 10)
#define MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT         (11)
#define MPFS_CANFD_FRAME_FORMAT_W_RWCNT               (0x1F << \
  MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT)

/*  IDENTIFIER_W registers */

#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT  (0)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT        (0x03FFFF << \
  MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_EXT_SHIFT)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT (18)
#define MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE       (0x07FF << \
  MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT)

/****************************************************************************
 * FPGA CANFD device hardware configuration
 ****************************************************************************/

struct mpfs_config_s
{
  uint32_t canfd_fpga_irq;           /* the only CAN-FD FPGA IRQ */
};

static const struct mpfs_config_s mpfs_fpga_canfd_config =
{
  .canfd_fpga_irq = MPFS_IRQ_FABRIC_F2H_0,
};

/****************************************************************************
 * The mpfs_driver_s encapsulates all state information for a single
 * hw interface
 ****************************************************************************/

struct mpfs_driver_s
{
  struct mpfs_can_priv_s can;

  const struct mpfs_config_s *config;

  uintptr_t base;             /* CANFD FPGA base address */
  bool bifup;                 /* true:ifup false:ifdown */

  struct work_s irqwork;      /* For deferring interrupt work to the work wq */
  struct work_s pollwork;     /* For deferring poll work to the work wq */

  struct canfd_frame *txdesc; /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc; /* A pointer to the list of RX descriptors */

  /* rx */

  uint32_t drv_flags;         /* driver flag */
  uint32_t rxfrm_first_word;  /* rx frame first word (usually a FFW) */

  /* tx */

  unsigned int txb_head;
  unsigned int txb_tail;
  uint32_t txb_prio;
  unsigned int ntxbufs;

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
  /* hw filter */

  uint8_t used_bit_filter_number;
  bool used_range_filter;
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;    /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mpfs_driver_s g_canfd;

static uint8_t g_tx_pool[(sizeof(struct canfd_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame) + TIMESTAMP_SIZE) *
                         POOL_SIZE];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Util functions */

static int
  mpfs_can_update_sample_point(const struct mpfs_can_bittiming_const_s *btc,
                               const unsigned int sample_point_nominal,
                               const unsigned int tseg,
                               unsigned int *tseg1_ptr,
                               unsigned int *tseg2_ptr,
                               unsigned int *sample_point_error_ptr);
static int
  mpfs_can_calc_bittiming(FAR struct mpfs_driver_s *priv,
                          struct mpfs_can_bittiming_s *bt,
                          const struct mpfs_can_bittiming_const_s *btc);
static const char *can_state_to_str(enum mpfs_can_state_e state);
static int
  mpfs_can_set_secondary_sample_point(FAR struct mpfs_driver_s *priv);
static void mpfs_can_set_mode(FAR struct mpfs_driver_s *priv,
                              const struct mpfs_can_ctrlmode_s *mode);

/* (from interrupt) RX related functions */

static void mpfs_can_read_rx_frame(FAR struct mpfs_driver_s *priv,
                                   struct canfd_frame *cf,
                                   uint32_t ffw);
static void mpfs_receive(FAR struct mpfs_driver_s *priv);

/* (from interrupt) TX related functions */

static void mpfs_can_rotate_txb_prio(FAR struct mpfs_driver_s *priv);
static void mpfs_can_give_txtb_cmd(FAR struct mpfs_driver_s *priv,
                                   enum mpfs_can_txtb_command cmd,
                                   uint8_t buf);
static void mpfs_txdone(FAR struct mpfs_driver_s *priv);
static void mpfs_txdone_work(FAR void *arg);

/* (from interrupt) Error handling related functions */

static enum mpfs_can_state_e
  mpfs_can_read_fault_state(FAR struct mpfs_driver_s *priv);
static void mpfs_can_get_rec_tec(FAR struct mpfs_driver_s *priv,
                                 struct mpfs_can_berr_counter_s *bec);
static void mpfs_err_interrupt(FAR struct mpfs_driver_s *priv, uint32_t isr);

/* Interrupt service routine */

static int mpfs_fpga_interrupt(int irq, FAR void *context, FAR void *arg);

/* (Nuttx network driver interface callback when TX packet available) Tx
 * related functions
 */

static enum mpfs_can_txtb_status
  mpfs_can_get_tx_status(FAR struct mpfs_driver_s *priv,
                         uint8_t buf);
static bool mpfs_can_is_txt_buf_writable(FAR struct mpfs_driver_s *priv,
                                         uint8_t buf);
static bool mpfs_can_insert_frame(FAR struct mpfs_driver_s *priv,
                                  const struct canfd_frame *cf,
                                  uint8_t buf,
                                  bool is_ccf);
static int mpfs_transmit(FAR struct mpfs_driver_s *priv);
static int mpfs_txpoll(struct net_driver_s *dev);
static void mpfs_txavail_work(FAR void *arg);
static int mpfs_txavail(struct net_driver_s *dev);

/* Bit timing related functions */

static int mpfs_can_set_btr(FAR struct mpfs_driver_s *priv,
                            struct mpfs_can_bittiming_s *bt,
                            bool nominal);
static int mpfs_can_set_bittiming(FAR struct mpfs_driver_s *priv);
static int mpfs_can_set_data_bittiming(FAR struct mpfs_driver_s *priv);

/* HW filter related functions */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static void mpfs_can_add_hw_filter(FAR struct mpfs_driver_s *priv,
                                   uint8_t filter_type,
                                   uint8_t can_id_type,
                                   uint8_t can_type,
                                   uint32_t fid1,
                                   uint32_t fid2);
static void mpfs_can_reset_hw_filter(FAR struct mpfs_driver_s *priv);
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/* FPGA CAN-FD controller life cycle routines */

static int mpfs_can_chip_start(FAR struct mpfs_driver_s *priv);
static void mpfs_can_chip_stop(FAR struct mpfs_driver_s *priv);
static int mpfs_reset(struct mpfs_driver_s *priv);

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
 * Name: mpfs_can_update_sample_point
 *
 * Description:
 *  Update sample point
 *
 * Input Parameters:
 *  btc                   - Bit timing const params
 *  sample_point_nominal  - The nominal sample point
 *  tseg                  -
 *  tseg1_ptr             -
 *  tseg2_ptr             -
 *  sample_point_error_ptr-
 *
 * Returned Value:
 *  Sample point value in one-tenth of a percent
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int
  mpfs_can_update_sample_point(const struct mpfs_can_bittiming_const_s *btc,
                               const unsigned int sample_point_nominal,
                               const unsigned int tseg,
                               unsigned int *tseg1_ptr,
                               unsigned int *tseg2_ptr,
                               unsigned int *sample_point_error_ptr)
{
  unsigned int sample_point_error;
  unsigned int best_sample_point_error = UINT_MAX;
  unsigned int sample_point;
  unsigned int best_sample_point = 0;
  unsigned int tseg1;
  unsigned int tseg2;
  int i;

  for (i = 0; i <= 1; i++)
    {
      tseg2 = tseg + CAN_CALC_SYNC_SEG - (sample_point_nominal *
        (tseg + CAN_CALC_SYNC_SEG)) / 1000 - i;
      tseg2 = clamp(tseg2, btc->tseg2_min, btc->tseg2_max);
      tseg1 = tseg - tseg2;
      if (tseg1 > btc->tseg1_max)
        {
          tseg1 = btc->tseg1_max;
          tseg2 = tseg - tseg1;
        }

      sample_point = 1000 * (tseg + CAN_CALC_SYNC_SEG - tseg2)
        / (tseg + CAN_CALC_SYNC_SEG);
      sample_point_error = sample_point_nominal - sample_point;

      if (sample_point <= sample_point_nominal &&
          sample_point_error < best_sample_point_error)
        {
          best_sample_point = sample_point;
          best_sample_point_error = sample_point_error;
          *tseg1_ptr = tseg1;
          *tseg2_ptr = tseg2;
        }
    }

  if (sample_point_error_ptr)
    {
      *sample_point_error_ptr = best_sample_point_error;
    }

  return best_sample_point;
}

/****************************************************************************
 * Name: mpfs_calc_bittiming
 *
 * Description:
 *  Calculate bittiming params from bittiming const params
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  bt    - Pointer to the bittiming structure to be set
 *  btc   - Pointer to the constant bittiming structure to be used to set
 *          the bittiming params
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int
  mpfs_can_calc_bittiming(FAR struct mpfs_driver_s *priv,
                          struct mpfs_can_bittiming_s *bt,
                          const struct mpfs_can_bittiming_const_s *btc)
{
  unsigned int bitrate;               /* current bitrate */
  unsigned int bitrate_error;         /* difference between current and
                                       * nominal value */
  unsigned int best_bitrate_error = UINT_MAX;
  unsigned int sample_point_error;    /* difference between current and
                                       * nominal value */
  unsigned int best_sample_point_error = UINT_MAX;
  unsigned int sample_point_nominal;  /* nominal sample point */
  unsigned int best_tseg = 0;         /* current best value for tseg */
  unsigned int best_brp = 0;          /* current best value for brp */
  unsigned int brp;
  unsigned int tsegall;
  unsigned int tseg;
  unsigned int tseg1 = 0;
  unsigned int tseg2 = 0;
  uint64_t v64;

  /* Use CiA recommended sample points */

  if (bt->sample_point)
    {
      sample_point_nominal = bt->sample_point;
    }
  else
    {
      if (bt->bitrate > 800 * 1000 /* BPS */)
        {
          sample_point_nominal = 750;
        }
      else if (bt->bitrate > 500 * 1000 /* BPS */)
        {
          sample_point_nominal = 800;
        }
      else
        {
          sample_point_nominal = 875;
        }
    }

  /* tseg even = round down, odd = round up */

  for (tseg = (btc->tseg1_max + btc->tseg2_max) * 2 + 1;
        tseg >= (btc->tseg1_min + btc->tseg2_min) * 2; tseg--)
    {
      tsegall = CAN_CALC_SYNC_SEG + tseg / 2;

      /* Compute all possible tseg choices (tseg = tseg1 + tseg2) */

      brp = priv->can.clock.freq / (tsegall * bt->bitrate) + tseg % 2;

      /* Choose brp step which is possible in system */

      brp = (brp / btc->brp_inc) * btc->brp_inc;
      if (brp < btc->brp_min || brp > btc->brp_max)
        {
          continue;
        }

      bitrate = priv->can.clock.freq / (brp * tsegall);
      bitrate_error = bt->bitrate - bitrate;

      /* tseg brp biterror */

      if (bitrate_error > best_bitrate_error)
        {
          continue;
        }

      /* reset sample point error if we have a better bitrate */

      if (bitrate_error < best_bitrate_error)
        {
          best_sample_point_error = UINT_MAX;
        }

      mpfs_can_update_sample_point(btc, sample_point_nominal, tseg / 2,
            &tseg1, &tseg2, &sample_point_error);
      if (sample_point_error >= best_sample_point_error)
        {
          continue;
        }

      best_sample_point_error = sample_point_error;
      best_bitrate_error = bitrate_error;
      best_tseg = tseg / 2;
      best_brp = brp;

      if (bitrate_error == 0 && sample_point_error == 0)
        {
          break;
        }
    }

  if (best_bitrate_error)
    {
      /* Error in one-tenth of a percent */

      v64 = (uint64_t)best_bitrate_error * 1000;
      do_div(v64, bt->bitrate);
      bitrate_error = (uint32_t)v64;
      if (bitrate_error > CAN_CALC_MAX_ERROR)
        {
          canerr("bitrate error %d.%d%% too high\n",
               bitrate_error / 10,
               bitrate_error % 10);
          return -EDOM;
        }
      canwarn("bitrate: %u, bitrate error %d.%d%%\n",
            bt->bitrate,
            bitrate_error / 10,
            bitrate_error % 10);
    }

  /* real sample point */

  bt->sample_point = mpfs_can_update_sample_point(btc, sample_point_nominal,
                                                  best_tseg, &tseg1,
                                                  &tseg2, NULL);

  v64 = (uint64_t)best_brp * 1000 * 1000 * 1000;
  do_div(v64, priv->can.clock.freq);
  bt->tq = (uint32_t)v64;
  bt->prop_seg = tseg1 / 2;
  bt->phase_seg1 = tseg1 - bt->prop_seg;
  bt->phase_seg2 = tseg2;

  /* check for sjw user settings */

  if (!bt->sjw || !btc->sjw_max)
    {
      bt->sjw = 5;
    }
  else
    {
      /* bt->sjw is at least 1 -> sanitize upper bound to sjw_max */

      if (bt->sjw > btc->sjw_max)
        {
          bt->sjw = btc->sjw_max;
        }

      /* bt->sjw must not be higher than tseg2 */

      if (tseg2 < bt->sjw)
        {
          bt->sjw = tseg2;
        }
    }

  bt->brp = best_brp;

  /* real bitrate */

  bt->bitrate =
    priv->can.clock.freq / (bt->brp * (CAN_CALC_SYNC_SEG + tseg1 + tseg2));

  return OK;
}

/****************************************************************************
 * Name: can_state_to_str
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
 *
 ****************************************************************************/

static const char *can_state_to_str(enum mpfs_can_state_e state)
{
  const char *txt = NULL;

  if (state >= 0 && state < CAN_STATE_MAX)
    {
      txt = mpfs_can_state_strings[state];
    }

  return txt ? txt : "UNKNOWN";
}

/****************************************************************************
 * Name: mpfs_can_read_rx_frame
 *
 * Description:
 *  Read frame from RX FIFO
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  cf    - Pointer to CANFD frame structure
 *  ffw   - Previously read frame format word
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Frame format word must be read separately before this and provided in
 *  'ffw'
 *
 ****************************************************************************/

static void mpfs_can_read_rx_frame(FAR struct mpfs_driver_s *priv,
                                   struct canfd_frame *cf,
                                   uint32_t ffw)
{
  uint32_t idw;
  unsigned int i, data_wc, dlc, len;

  /* CAN ID */

  idw = getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);
  if (MPFS_CANFD_FRAME_FORMAT_W_IDE & ffw)
    {
      cf->can_id = (idw & CAN_EFF_MASK) | CAN_EFF_FLAG;
    }
  else
    {
      cf->can_id =
        (idw >> MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT) &
        CAN_SFF_MASK;
    }

  /* BRS, ESI, RTR Flags */

  cf->flags = 0;
  if (MPFS_CANFD_FRAME_FORMAT_W_FDF & ffw)
    {
      /* Enable bitrate switch by default if frame is CANFD */

      cf->flags |= CANFD_BRS;

      if (MPFS_CANFD_FRAME_FORMAT_W_ESI_RSV & ffw)
        {
          cf->flags |= CANFD_ESI;
        }
    }
  else if (MPFS_CANFD_FRAME_FORMAT_W_RTR & ffw)
    {
      cf->can_id |= CAN_RTR_FLAG;
    }

  /* RWCNT : RX Count of Words without FRAME_FORMAT WORD. Minus the 3 words
   * for 1 IDW, 2 timestamp words
   */

  data_wc = ((MPFS_CANFD_FRAME_FORMAT_W_RWCNT & ffw) >>
    MPFS_CANFD_FRAME_FORMAT_W_RWCNT_SHIFT) - 3;

  /* DLC */

  dlc = (MPFS_CANFD_FRAME_FORMAT_W_DLC & ffw) >>
    MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT;
  if (dlc <= 8)
    {
     len = dlc;
    }
  else
    {
      if (MPFS_CANFD_FRAME_FORMAT_W_FDF & ffw)
        {
          len = data_wc << 2;
        }
      else
        {
          len = 8;
        }
    }
  cf->len = len;
  if (expect_false(len > data_wc * 4))
    {
      len = data_wc * 4;
    }

  /* Timestamp - Read and throw away */

  getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);
  getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);

  /* Data */

  for (i = 0; i < len; i += 4)
    {
      uint32_t data = getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);

      *(uint32_t *)(cf->data + i) = data;
    }

  /* Read and discard exceeding data that does not fit any frame */

  while (expect_false(i < data_wc * 4))
    {
      getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);
      i += 4;
    }
}

/****************************************************************************
 * Name: mpfs_receive
 *
 * Description:
 *  An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void mpfs_receive(FAR struct mpfs_driver_s *priv)
{
  uint32_t status;
  uint32_t frame_count;
  int work_done = 0;
  bool is_classical_can_frame = false;

  frame_count = (getreg32(priv->base + MPFS_CANFD_RX_STATUS_OFFSET) &
    MPFS_CANFD_RX_STATUS_RXFRC) >> MPFS_CANFD_RX_STATUS_RXFRC_SHIFT;
  while (frame_count)
    {
      struct canfd_frame *cf = (struct canfd_frame *)priv->rxdesc;
      uint32_t ffw;

      ffw = getreg32(priv->base + MPFS_CANFD_RX_DATA_OFFSET);

      if (!(MPFS_CANFD_FRAME_FORMAT_W_RWCNT & ffw))
        {
          canerr("rx word count is 0\n");
          break;
        }

      if (!(MPFS_CANFD_FRAME_FORMAT_W_FDF & ffw))
        {
          if (MPFS_CANFD_FRAME_FORMAT_W_RTR & ffw)
            {
              caninfo("Remote Frame received\n");
            }
          else
            {
              /* caninfo("Classical CAN Frame received\n"); */
            }

          is_classical_can_frame = true;
        }
      else
        {
          /* caninfo("CANFD Frame received\n"); */
        }

      /* Read the classical or CANFD or remote frame */

      mpfs_can_read_rx_frame(priv, cf, ffw);

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = is_classical_can_frame ?
        sizeof(struct can_frame) : sizeof(struct canfd_frame);
      priv->dev.d_buf = (uint8_t *)cf;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);
      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;

      work_done++;
      frame_count = (getreg32(priv->base + MPFS_CANFD_RX_STATUS_OFFSET) &
        MPFS_CANFD_RX_STATUS_RXFRC) >> MPFS_CANFD_RX_STATUS_RXFRC_SHIFT;
    }

  /* Check for RX FIFO Overflow */

  status = getreg32(priv->base + MPFS_CANFD_STATUS_OFFSET);
  if (MPFS_CANFD_STATUS_DOR & status)
    {
      struct canfd_frame *cf = (struct canfd_frame *)priv->rxdesc;

      caninfo("rx fifo overflow\n");

      cf->can_id = CAN_ERR_CRTL;
      cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = sizeof(struct canfd_frame);
      priv->dev.d_buf = (uint8_t *)cf;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);
      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;

      /* Clear Data Overrun */

      putreg32(MPFS_CANFD_COMMAND_CDO,
               priv->base + MPFS_CANFD_COMMAND_OFFSET);
    }

  /* Clear and enable RBNEI. It is level-triggered, so
   * there is no race condition.
   */

  putreg32(MPFS_CANFD_INT_STAT_RBNEI,
           priv->base + MPFS_CANFD_INT_STAT_OFFSET);
  putreg32(MPFS_CANFD_INT_STAT_RBNEI,
           priv->base + MPFS_CANFD_INT_MASK_CLR_OFFSET);
}

/****************************************************************************
 * Name: mpfs_rotate_txb_prio
 *
 * Description:
 *  Rotates priorities of TXT Buffers
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_rotate_txb_prio(FAR struct mpfs_driver_s *priv)
{
  uint32_t prio = priv->txb_prio;

  prio = (prio << 4) | ((prio >> ((priv->ntxbufs - 1) * 4)) & 0xf);
  priv->txb_prio = prio;
  putreg32(prio, priv->base + MPFS_CANFD_TX_PRIORITY_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_give_txtb_cmd
 *
 * Description:
 *  Apply command on a TXT buffer
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  cmd   - Cmd to give
 *  buf   - The TXT buffer index (0-based)
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_give_txtb_cmd(FAR struct mpfs_driver_s *priv,
                                   enum mpfs_can_txtb_command cmd,
                                   uint8_t buf)
{
  uint32_t tx_cmd = cmd;

  tx_cmd |= 1 << (buf + 8);
  putreg32(tx_cmd, priv->base + MPFS_CANFD_TX_COMMAND_OFFSET);
}

/****************************************************************************
 * Name: mpfs_txdone
 *
 * Description:
 *  Tx done interrupt service rountine
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
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
  bool first = true;
  bool some_buffers_processed;
  enum mpfs_can_txtb_status txtb_status;
  uint8_t txtb_id;
  irqstate_t flags;

  do
    {
      flags = up_irq_save();
      some_buffers_processed = false;
      while ((int)(priv->txb_head - priv->txb_tail) > 0 ||
        (priv->txb_head == 0 && priv->txb_tail > 0))
        {
          txtb_id = priv->txb_tail % priv->ntxbufs;
          txtb_status = mpfs_can_get_tx_status(priv, txtb_id);
          bool other_status = false;

          switch (txtb_status)
            {
            case TXT_TOK:
              break;
            case TXT_ERR:
              canwarn("TXB in Error state\n");
              break;
            case TXT_ABT:
              canwarn("TXB in Aborted state\n");
              break;
            default:
              other_status = true;

              /* Bug only if the first buffer is not finished, otherwise it
               * is pretty much expected.
               */

              if (first)
                {
                  canerr("BUG, TXB#%u not in a finished state (0x%x)!\n",
                         txtb_id, txtb_status);

                  priv->txb_tail++;

                  /* Adjust priorities before marking the buffer as empty */

                  mpfs_can_rotate_txb_prio(priv);
                  mpfs_can_give_txtb_cmd(priv, TXT_CMD_SET_EMPTY, txtb_id);

                  /* Clear txb status change interrupt */

                  putreg32(MPFS_CANFD_INT_STAT_TXBHCI,
                           priv->base + MPFS_CANFD_INT_STAT_OFFSET);

                  up_irq_restore(flags);
                  return;
                }
              break;
            }

          if (other_status)
            {
              break;
            }
          else
            {
              priv->txb_tail++;
              first = false;
              some_buffers_processed = true;

              /* Adjust priorities before marking the buffer as empty */

              mpfs_can_rotate_txb_prio(priv);
              mpfs_can_give_txtb_cmd(priv, TXT_CMD_SET_EMPTY, txtb_id);
            }
        }

      up_irq_restore(flags);
      if (some_buffers_processed)
        {
          /* Clear the interrupt again. We do not want to receive again
           * interrupt for the buffer already handled. If it is the last
           * finished one then it would cause log of spurious interrupt.
           */

          putreg32(MPFS_CANFD_INT_STAT_TXBHCI,
                   priv->base + MPFS_CANFD_INT_STAT_OFFSET);
        }
    }
  while (some_buffers_processed);
}

/****************************************************************************
 * Name: mpfs_txdone_work
 *
 * Description:
 *  An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Global interrupts are disabled by the watchdog logic.
 *  We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/

static void mpfs_txdone_work(FAR void *arg)
{
  FAR struct mpfs_driver_s *priv = (FAR struct mpfs_driver_s *)arg;

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, mpfs_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: mpfs_can_read_fault_state
 *
 * Description:
 *    Reads FPGA CANFD fault confinement state
 *
 * Input Parameters:
 *    priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *    Fault confinement state of controller
 *
 ****************************************************************************/

static enum mpfs_can_state_e
  mpfs_can_read_fault_state(FAR struct mpfs_driver_s *priv)
{
  u_int32_t ewl_erp_fs_reg, rec_tec_reg, ew_limit, rec_val, tec_val;

  ewl_erp_fs_reg = getreg32(priv->base + MPFS_CANFD_EWL_OFFSET);
  rec_tec_reg = getreg32(priv->base + MPFS_CANFD_REC_OFFSET);

  ew_limit = ((ewl_erp_fs_reg & MPFS_CANFD_EWL_EW_LIMIT) >>
    MPFS_CANFD_EWL_EW_LIMIT_SHIFT);
  rec_val = ((rec_tec_reg & MPFS_CANFD_REC_REC_VAL) >>
    MPFS_CANFD_REC_REC_VAL_SHIFT);
  tec_val = ((rec_tec_reg & MPFS_CANFD_REC_TEC_VAL) >>
    MPFS_CANFD_REC_TEC_VAL_SHIFT);

  if (ewl_erp_fs_reg & MPFS_CANFD_EWL_ERA)
    {
      if (rec_val < ew_limit && tec_val < ew_limit)
        {
          return CAN_STATE_ERROR_ACTIVE;
        }
      else
        {
          return CAN_STATE_ERROR_WARNING;
        }
    }
  else if (ewl_erp_fs_reg & MPFS_CANFD_EWL_ERP)
    {
     return CAN_STATE_ERROR_PASSIVE;
    }
  else if (ewl_erp_fs_reg & MPFS_CANFD_EWL_BOF)
    {
     return CAN_STATE_BUS_OFF;
    }

  canwarn("Invalid FPGA CAN-FD error state\n");
  return CAN_STATE_ERROR_PASSIVE;
}

/****************************************************************************
 * Name: mpfs_read_rec_tec
 *
 * Description:
 *    Reads FPGA CANFD RX/TX error counter
 *
 * Input Parameters:
 *    priv  - Pointer to the private FPGA CANFD driver state structure
 *    bec   - Pointer to Error counter structure
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void mpfs_can_get_rec_tec(FAR struct mpfs_driver_s *priv,
                                 struct mpfs_can_berr_counter_s *bec)
{
  uint32_t rec_tec_reg = getreg32(priv->base + MPFS_CANFD_REC_OFFSET);

  bec->rxerr = ((rec_tec_reg & MPFS_CANFD_REC_REC_VAL) >>
    MPFS_CANFD_REC_REC_VAL_SHIFT);
  bec->txerr = ((rec_tec_reg & MPFS_CANFD_REC_TEC_VAL) >>
    MPFS_CANFD_REC_TEC_VAL_SHIFT);
}

/****************************************************************************
 * Name: mpfs_err_interrupt
 *
 * Description:
 *    Error frame ISR
 *
 * Input Parameters:
 *    priv  - Pointer to the private FPGA CANFD driver state structure
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
  uint32_t err_type, err_pos, alc_id_field, alc_bit;

  mpfs_can_get_rec_tec(priv, &bec);
  state = mpfs_can_read_fault_state(priv);
  err_capt_retr_ctr_alc_reg =
    getreg32(priv->base + MPFS_CANFD_ERR_CAPT_OFFSET);

  err_type = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ERR_TYPE) >>
    MPFS_CANFD_ERR_CAPT_ERR_TYPE_SHIFT);
  err_pos = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ERR_POS) >>
    MPFS_CANFD_ERR_CAPT_ERR_POS_SHIFT);
  alc_id_field =
    ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD) >>
      MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD_SHIFT);
  alc_bit = ((err_capt_retr_ctr_alc_reg & MPFS_CANFD_ERR_CAPT_ALC_BIT) >>
    MPFS_CANFD_ERR_CAPT_ALC_BIT_SHIFT);

  caninfo("ISR = 0x%08x, rxerr %d, txerr %d, error type %u, pos %u, ALC "
          "id_field %u, bit %u\n", isr, bec.rxerr, bec.txerr, err_type,
          err_pos, alc_id_field, alc_bit);

  /* EWLI: error warning limit condition met
   * FCSI: fault confinement state changed
   * ALI:  arbitration lost (just informative)
   * BEI:  bus error interrupt
   */

  if (MPFS_CANFD_INT_STAT_FCSI & isr || MPFS_CANFD_INT_STAT_EWLI & isr)
    {
      if (priv->can.state == state)
        {
          canwarn("current and previous state is the same! "
                  "(missed interrupt?)\n");
        }
      else
        {
          caninfo("state changes from %s to %s\n",
                can_state_to_str(priv->can.state),
                can_state_to_str(state));
        }

      priv->can.state = state;

      switch (state)
        {
        case CAN_STATE_BUS_OFF:
          priv->can.can_stats.bus_off++;
          cf->can_id = CAN_ERR_BUSOFF;
          break;
        case CAN_STATE_ERROR_PASSIVE:
          priv->can.can_stats.error_passive++;
          cf->can_id = CAN_ERR_CRTL;
          cf->data[1] = (bec.rxerr > 127) ?
            CAN_ERR_CRTL_RX_PASSIVE : CAN_ERR_CRTL_TX_PASSIVE;
          cf->data[6] = bec.txerr;
          cf->data[7] = bec.rxerr;
          break;
        case CAN_STATE_ERROR_WARNING:
          priv->can.can_stats.error_warning++;
          cf->can_id = CAN_ERR_CRTL;
          cf->data[1] = (bec.txerr > bec.rxerr) ?
            CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
          cf->data[6] = bec.txerr;
          cf->data[7] = bec.rxerr;
          break;
        case CAN_STATE_ERROR_ACTIVE:
          cf->data[1] = CAN_ERR_CRTL_UNSPEC;
          cf->data[6] = bec.txerr;
          cf->data[7] = bec.rxerr;
          break;
        default:
          canwarn("unhandled error state (%d:%s)!\n", state,
                  can_state_to_str(state));
          break;
        }
    }

  /* Check for Arbitration Lost interrupt. */

  if (MPFS_CANFD_INT_STAT_ALI & isr)
    {
      caninfo("arbitration lost\n");
      priv->can.can_stats.arbitration_lost++;
      cf->can_id = CAN_ERR_LOSTARB;
      cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
    }

  /* Check for Bus Error interrupt. */

  if (MPFS_CANFD_INT_STAT_BEI & isr)
    {
      caninfo("bus error\n");
      priv->can.can_stats.bus_error++;
      cf->can_id = CAN_ERR_PROT | CAN_ERR_BUSERROR;
      cf->data[2] = CAN_ERR_PROT_UNSPEC;
      cf->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
    }

  /* Send to socket interface. */

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
 * Name: mpfs_fpga_interrupt
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

static int mpfs_fpga_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct mpfs_driver_s *priv = (FAR struct mpfs_driver_s *)arg;

  uint32_t isr, icr, imask;
  int irq_loops;

  for (irq_loops = 0; irq_loops < 10000; irq_loops++)
    {
      /* Get the interrupt status */

      isr  = getreg32(priv->base + MPFS_CANFD_INT_STAT_OFFSET);

      /* Check and exit interrupt service routine if there is no int flag in
       * INT_STAT reg.
       */

      if (!isr)
        {
          return irq_loops ? OK : -1;
        }

      /* Receive Buffer Not Empty Interrupt */

      if (isr & MPFS_CANFD_INT_STAT_RBNEI)
        {
          /* Mask RXBNEI first, then clear interrupt. Even if
          * another IRQ fires, RBNEI will always be 0 (masked).
          */

          icr = MPFS_CANFD_INT_STAT_RBNEI;
          putreg32(icr, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);
          putreg32(icr, priv->base + MPFS_CANFD_INT_STAT_OFFSET);

          /* Main receiving routine */

          mpfs_receive(priv);
        }

      /* TXT Buffer HW Command Interrupt */

      if (isr & MPFS_CANFD_INT_STAT_TXBHCI)
        {
          /* Clear TX interrupt flags */

          mpfs_txdone(priv);

          /* Schedule work to poll for next available tx frame from the
           * network.
           */

          work_queue(CANWORK, &priv->irqwork, mpfs_txdone_work, priv, 0);
        }

      /* Error Interrupts */

      if (isr & MPFS_CANFD_INT_STAT_EWLI ||
          isr & MPFS_CANFD_INT_STAT_FCSI ||
          isr & MPFS_CANFD_INT_STAT_ALI ||
          isr & MPFS_CANFD_INT_STAT_BEI)
        {
          icr = isr & (MPFS_CANFD_INT_STAT_EWLI |
                      MPFS_CANFD_INT_STAT_FCSI |
                      MPFS_CANFD_INT_STAT_ALI |
                      MPFS_CANFD_INT_STAT_BEI);
          caninfo("Some error interrupts. Clearing 0x%08x\n", icr);
          putreg32(icr, priv->base + MPFS_CANFD_INT_STAT_OFFSET);
          mpfs_err_interrupt(priv, isr);
        }
    }

  /* Now, it seems that there are still some interrupt flags that remain
   * stuck in INT_STAT reg.
   */

  canerr("Stuck interrupt (isr=%08x)\n", isr);

  /* Check if any of the stuck one belongs to txb status. */

  if (isr & MPFS_CANFD_INT_STAT_TXBHCI)
    {
      caninfo("txb_head=0x%08x txb_tail=0x%08x\n", priv->txb_head,
              priv->txb_tail);
      for (int i = 0; i < priv->ntxbufs; i++)
        {
          uint32_t status = mpfs_can_get_tx_status(priv, i);
          caninfo("txb[%d] txb status=0x%08x\n", i, status);
        }

      /* Clear txb status change interrupt */

      putreg32(MPFS_CANFD_INT_STAT_TXBHCI,
               priv->base + MPFS_CANFD_INT_STAT_OFFSET);
    }

  /* Check if any of the stuck one belongs to RX buffer data overrun */

  if (isr & MPFS_CANFD_INT_STAT_DOI)
    {
      struct canfd_frame *cf = (struct canfd_frame *)priv->rxdesc;

      caninfo("rx fifo overflow\n");

      cf->can_id = CAN_ERR_CRTL;
      cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = sizeof(struct canfd_frame);
      priv->dev.d_buf = (uint8_t *)cf;

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);
      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;

      /* Clear Data Overrun interrupt */

      putreg32(MPFS_CANFD_INT_STAT_DOI,
               priv->base + MPFS_CANFD_INT_STAT_OFFSET);
    }

  /* Clear and reset all interrupt. */

  caninfo("Reset all interrupts...\n");
  imask = 0xffffffff;
  putreg32(imask, priv->base + MPFS_CANFD_INT_ENA_CLR_OFFSET);
  putreg32(imask, priv->base + MPFS_CANFD_INT_ENA_SET_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: mpfs_get_tx_status
 *
 * Description:
 *  Get status of txt buffer
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  buf   - txt buffer index to get status of (0-based)
 *
 * Returned Value:
 *  Status of txt buffer
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static enum mpfs_can_txtb_status
  mpfs_can_get_tx_status(FAR struct mpfs_driver_s *priv,
                         uint8_t buf)
{
  uint32_t tx_status = getreg32(priv->base + MPFS_CANFD_TX_STATUS_OFFSET);
  enum mpfs_can_txtb_status status = (tx_status >> (buf * 4)) & 0xf;

  return status;
}

/****************************************************************************
 * Name: mpfs_can_is_txt_buf_writable
 *
 * Description:
 *  Check if frame can be inserted to TXT Buffer
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  buf   - txt buffer index to get status of (0-based)
 *
 * Returned Value:
 *  True  - Frame can be inserted to txt buffer
 *  False - If attempted, frame will not be inserted to txt buffer
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static bool mpfs_can_is_txt_buf_writable(FAR struct mpfs_driver_s *priv,
                                         uint8_t buf)
{
  enum mpfs_can_txtb_status buf_status;

  buf_status = mpfs_can_get_tx_status(priv, buf);
  if (buf_status == TXT_RDY || buf_status == TXT_TRAN ||
      buf_status == TXT_ABTP)
    {
      canwarn("TXTB status %d\n", buf_status);
     return false;
    }

  return true;
}

/****************************************************************************
 * Name: mpfs_can_insert_frame
 *
 * Description:
 *  Insert frame to txt buffer on the FPGA CANFD controller
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  cf    - Pointer to the CANFD frame to be inserted
 *  buf   - txt buffer index to which the cf frame is inserted (0-based)
 *  is_ccf- is classical can frame (bool)
 *
 * Returned Value:
 *  True  - Frame inserted successfully
 *  False - Frame was not inserted due to one of:
 *            1. Txt buffer is not writable (it is in a wrong state)
 *            2. Invalid txt buffer index
 *            3. Invalid frame length
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static bool mpfs_can_insert_frame(FAR struct mpfs_driver_s *priv,
                                  const struct canfd_frame *cf,
                                  uint8_t buf,
                                  bool is_ccf)
{
  uint32_t buf_base;
  uint32_t ffw = 0;
  uint32_t idw = 0;
  unsigned int i;

  /* Check for invalid txt buffer index */

  if (buf >= priv->ntxbufs)
    {
      canerr("invalid txt buffer index...\n");
      return false;
    }

  /* Check if it is possible to insert frame to txt buffer */

  if (!mpfs_can_is_txt_buf_writable(priv, buf))
    {
      canwarn("not possible to insert frame to txt buffer...\n");
      return false;
    }

  /* Check for invalid classical CAN / CANFD frame length */

  if (cf->len > CANFD_MAX_DLEN || (cf->len > CAN_MAX_DLEN && is_ccf))
    {
      canerr("invalid classical / CANFD CAN frame length...\n");
      return false;
    }

  /* Prepare Frame format */

  if (cf->can_id & CAN_RTR_FLAG)  /* remote transmission request */
    {
      ffw |= MPFS_CANFD_FRAME_FORMAT_W_RTR;
    }

  if (cf->can_id & CAN_EFF_FLAG)  /* extended frame format (29 bit long id) */
    {
      ffw |= MPFS_CANFD_FRAME_FORMAT_W_IDE;
    }

  if (!is_ccf)
    {
      ffw |= MPFS_CANFD_FRAME_FORMAT_W_FDF; /* FD Frame */
      if (cf->flags & CANFD_BRS)
        {
          ffw |= MPFS_CANFD_FRAME_FORMAT_W_BRS; /* Bit rate switch */
        }
    }

  ffw |= MPFS_CANFD_FRAME_FORMAT_W_DLC & (len_to_can_dlc[cf->len] <<
    MPFS_CANFD_FRAME_FORMAT_W_DLC_SHIFT);

  /* Prepare identifier */

  if (cf->can_id & CAN_EFF_FLAG)
    {
     idw = cf->can_id & CAN_EFF_MASK;
    }
  else
    {
      idw = MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE &
        ((cf->can_id & CAN_SFF_MASK) <<
          MPFS_CANFD_IDENTIFIER_W_IDENTIFIER_BASE_SHIFT);
    }

  /* Write ID, Frame format, Don't write timestamp -> Time triggered
   * transmission disabled.
   */

  buf_base = (buf + 1) * 0x100;
  putreg32(ffw, priv->base + buf_base + MPFS_CANFD_FRAME_FORMAT_W_OFFSET);
  putreg32(idw, priv->base + buf_base + MPFS_CANFD_IDENTIFIER_W_OFFSET);

  /* Write Data payload */

  if (!(cf->can_id & CAN_RTR_FLAG))
    {
      for (i = 0; i < cf->len; i += 4)
        {
          uint32_t data = *(uint32_t *)(cf->data + i);
          putreg32(data,
                   priv->base + buf_base + MPFS_CANFD_DATA_1_4_W_OFFSET + i);
        }
    }

  return true;
}

/****************************************************************************
 * Name: mpfs_transmit
 *
 * Description:
 *  Start hardware transmission.  Called either from the txdone interrupt
 *  handling or from watchdog based polling.
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno on failure
 *
 * Assumptions:
 *  May or may not be called from an interrupt handler.  In either case,
 *  global interrupts are disabled, either explicitly or indirectly through
 *  interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_transmit(FAR struct mpfs_driver_s *priv)
{
  uint32_t txtb_id;
  bool ok, is_classical_can_frame;

  /* Retrieve the classical CAN / CANFD frame from network device buffer */

  is_classical_can_frame =
    priv->dev.d_len <= sizeof(struct can_frame) ? true : false;
  struct canfd_frame *cf = (struct canfd_frame *)priv->dev.d_buf;

  /* Get the current txt buffer ID */

  txtb_id = priv->txb_head % priv->ntxbufs;

  /* Insert classical CAN/CANFD frame into controller txt bf at txtb_id */

  ok = mpfs_can_insert_frame(priv, cf, txtb_id, is_classical_can_frame);
  if (!ok)
    {
      canwarn("BUG! TXNF set but cannot insert frame into TXTB! HW Bug?\n");
      NETDEV_TXERRORS(&priv->dev);
      return OK;
    }

  /* Now, write to txt buffer seems ok, use txt command to set buffer state
   * to READY for xmit.
   */

  mpfs_can_give_txtb_cmd(priv, TXT_CMD_SET_READY, txtb_id);
  priv->txb_head++;

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  return OK;
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
 *   Zero (OK) on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int mpfs_txpoll(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          mpfs_transmit(priv);

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if (!MPFS_CAN_FD_TXTNF(priv))
            {
              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return OK;
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

static void mpfs_txavail_work(FAR void *arg)
{
  FAR struct mpfs_driver_s *priv = (FAR struct mpfs_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the controller to hold another outgoing
       * packet.
       */

      if (MPFS_CAN_FD_TXTNF(priv))
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
 *  0 - OK
 *
 * Assumptions:
 *  Called in normal user mode
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
 * Name: mpfs_can_btr
 *
 * Description:
 *  Set FPGA CAN controller data bittiming
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  bt      - Pointer to Bit timing structure
 *  nominal - True - Nominal bit timing, False - Data bit timing
 *
 * Returned Value:
 *  Zero (OK) on success, -%EPERM if controller is enabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_set_btr(FAR struct mpfs_driver_s *priv,
                            struct mpfs_can_bittiming_s *bt,
                            bool nominal)
{
  int max_ph1_len = 31;
  uint32_t btr = 0;

  if (MPFS_CAN_FD_ENABLED(priv))
    {
      canerr("BUG! Cannot set bittiming - CAN is enabled\n");
      return -EPERM;
    }

  if (nominal)
    {
     max_ph1_len = 63;
    }

  /* The timing calculation functions have only constraints on tseg1, which
   * is prop_seg + phase1_seg combined. tseg1 is then split in half and
   * stored into prog_seg and phase_seg1. In FPGA CAN FD controller, PROP is
   * 6 bits wide but PH1 only 5, so we must re-distribute the values here.
   */

  if (bt->phase_seg1 > max_ph1_len)
    {
      bt->prop_seg += bt->phase_seg1 - max_ph1_len;
      bt->phase_seg1 = max_ph1_len;
    }

  if (nominal)
    {
      btr = bt->prop_seg << MPFS_CANFD_BTR_PROP_SHIFT;
      btr |= bt->phase_seg1 << MPFS_CANFD_BTR_PH1_SHIFT;
      btr |= bt->phase_seg2 << MPFS_CANFD_BTR_PH2_SHIFT;
      btr |= bt->brp << MPFS_CANFD_BTR_BRP_SHIFT;
      btr |= bt->sjw << MPFS_CANFD_BTR_SJW_SHIFT;
      caninfo("prop_seg: %u, phase_seg1: %u, phase_seg2: %u, brp: %u, "
              "sjw: %u \n", bt->prop_seg, bt->phase_seg1, bt->phase_seg2,
              bt->brp, bt->sjw);
      putreg32(btr, priv->base + MPFS_CANFD_BTR_OFFSET);
    }
  else
    {
      btr = bt->prop_seg << MPFS_CANFD_BTR_FD_PROP_FD_SHIFT;
      btr |= bt->phase_seg1 << MPFS_CANFD_BTR_FD_PH1_FD_SHIFT;
      btr |= bt->phase_seg2 << MPFS_CANFD_BTR_FD_PH2_FD_SHIFT;
      btr |= bt->brp << MPFS_CANFD_BTR_FD_BRP_FD_SHIFT;
      btr |= bt->sjw << MPFS_CANFD_BTR_FD_SJW_FD_SHIFT;
      caninfo("prop_seg: %u, phase_seg1: %u, phase_seg2: %u, brp: %u, "
              "sjw: %u \n", bt->prop_seg, bt->phase_seg1, bt->phase_seg2,
              bt->brp, bt->sjw);

      putreg32(btr, priv->base + MPFS_CANFD_BTR_FD_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_set_bittiming
 *
 * Description:
 *  Set FPGA CAN controller nominal (arbitration) bittiming
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success, -%EPERM if controller is enabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_set_bittiming(FAR struct mpfs_driver_s *priv)
{
  struct mpfs_can_bittiming_s *bt = &priv->can.bittiming;

  /* Note that bt may be modified here */

  return mpfs_can_set_btr(priv, bt, true);
}

/****************************************************************************
 * Name: mpfs_can_set_data_bittiming
 *
 * Description:
 *  Set FPGA CAN controller data bittiming
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success, -%EPERM if controller is enabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_set_data_bittiming(FAR struct mpfs_driver_s *priv)
{
  struct mpfs_can_bittiming_s *dbt = &priv->can.data_bittiming;

  /* Note that dbt may be modified here */

  return mpfs_can_set_btr(priv, dbt, false);
}

/****************************************************************************
 * Name: mpfs_can_set_secondary_sample_point
 *
 * Description:
 *  Set FPGA CAN controller secondary sample point.
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success, -%EPERM if controller is enabled
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_set_secondary_sample_point(FAR struct
                                               mpfs_driver_s *priv)
{
  struct mpfs_can_bittiming_s *dbt = &(priv->can.data_bittiming);
  int ssp_offset = 0;
  uint32_t ssp_cfg = 0; /* No SSP by default */

  if (MPFS_CAN_FD_ENABLED(priv))
    {
      canerr("BUG! Cannot set SSP - CAN is enabled\n");
      return -EPERM;
    }

  /* Use SSP for bit-rates above 1 Mbits/s */

  if (dbt->bitrate > 1000000)
    {
      /* Calculate SSP in minimal time quanta */

      ssp_offset = (priv->can.clock.freq / 1000) *
                   dbt->sample_point / dbt->bitrate;

      if (ssp_offset > 127)
        {
          canwarn("SSP offset saturated to 127\n");
          ssp_offset = 127;
        }

      ssp_cfg = ssp_offset << MPFS_CANFD_TRV_DELAY_SSP_OFFSET_SHIFT;
      ssp_cfg |= 0x1 << MPFS_CANFD_TRV_DELAY_SSP_SRC_SHIFT;
    }

  putreg32(ssp_cfg, priv->base + MPFS_CANFD_TRV_DELAY_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_set_mode
 *
 * Description:
 *  Set FPGA CAN FD controller mode
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *  mode  - Pointer to controller modes to be set
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_set_mode(FAR struct mpfs_driver_s *priv,
                              const struct mpfs_can_ctrlmode_s *mode)
{
  uint32_t mode_reg = getreg32(priv->base + MPFS_CANFD_MODE_OFFSET);

  mode_reg = (mode->flags & CAN_CTRLMODE_LOOPBACK) ?
    (mode_reg | MPFS_CANFD_MODE_ILBP) :
    (mode_reg & ~MPFS_CANFD_MODE_ILBP);

  mode_reg = (mode->flags & CAN_CTRLMODE_LISTENONLY) ?
    (mode_reg | MPFS_CANFD_MODE_BMM) :
    (mode_reg & ~MPFS_CANFD_MODE_BMM);

  mode_reg = (mode->flags & CAN_CTRLMODE_FD) ?
    (mode_reg | MPFS_CANFD_MODE_FDE) :
    (mode_reg & ~MPFS_CANFD_MODE_FDE);

  mode_reg = (mode->flags & CAN_CTRLMODE_PRESUME_ACK) ?
    (mode_reg | MPFS_CANFD_MODE_ACF) :
    (mode_reg & ~MPFS_CANFD_MODE_ACF);

  mode_reg = (mode->flags & CAN_CTRLMODE_FD_NON_ISO) ?
    (mode_reg | MPFS_CANFD_MODE_NISOFD) :
    (mode_reg & ~MPFS_CANFD_MODE_NISOFD);

  /* One shot mode supported indirectly via Retransmit limit */

  mode_reg &= ~MPFS_CANFD_MODE_RTRTH;
  mode_reg = (mode->flags & CAN_CTRLMODE_ONE_SHOT) ?
    (mode_reg | MPFS_CANFD_MODE_RTRLE) :
    (mode_reg & ~MPFS_CANFD_MODE_RTRLE);

  /* Acceptance filter mode supported if FILTER IOCTL CONFIG is used */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
  mode_reg |= MPFS_CANFD_MODE_AFM;
#endif

  /* Some bits fixed:
   * TSTM  - Off, User shall not be able to change REC/TEC by hand during
   * operation
   */

  mode_reg &= ~MPFS_CANFD_MODE_TSTM;

  putreg32(mode_reg, priv->base + MPFS_CANFD_MODE_OFFSET);
}

/****************************************************************************
 * Name: mpfs_can_add_hw_filter
 *
 * Description:
 *  Add new hw filter to FGPA CANFD Controller
 *
 * Input Parameters:
 *  priv          - Pointer to the private FPGA CANFD driver state structure
 *  filter_type   - Filter A, B, C or Range
 *  can_id_type   - std CAN ID / ext CAN ID
 *  can_type      - classical CAN / CANFD
 *  filter_id1    - filter id 1 (can be filter value for bit filter or range
 * low for range filter)
 *  filter_id2    - filter id 2 (can be filter mask for bit filter or range
 * high for range filter)
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static void mpfs_can_add_hw_filter(FAR struct mpfs_driver_s *priv,
                                   uint8_t filter_type,
                                   uint8_t can_id_type,
                                   uint8_t can_type,
                                   uint32_t fid1,
                                   uint32_t fid2)
{
  uint32_t fc_reg;

  fc_reg = getreg32(priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

  switch (filter_type)
    {
      case HW_FILTER_A:

        /* Set filter control reg for filter A */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FANE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FANB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FAFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FAFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FAFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FAFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FANE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FANB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set bit filter value / mask */

        putreg32(fid1, priv->base + MPFS_CANFD_FILTER_A_VAL_OFFSET);
        putreg32(fid2, priv->base + MPFS_CANFD_FILTER_A_MASK_OFFSET);
        break;

      case HW_FILTER_B:

        /* Set filter control reg for filter B */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBNE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBNB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FBFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBNE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FBNB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set bit filter value / mask */

        putreg32(fid1, priv->base + MPFS_CANFD_FILTER_B_VAL_OFFSET);
        putreg32(fid2, priv->base + MPFS_CANFD_FILTER_B_MASK_OFFSET);
        break;

      case HW_FILTER_C:

        /* Set filter control reg for filter C */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCNE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCNB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FCFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCNE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FCNB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set bit filter value / mask */

        putreg32(fid1, priv->base + MPFS_CANFD_FILTER_C_VAL_OFFSET);
        putreg32(fid2, priv->base + MPFS_CANFD_FILTER_C_MASK_OFFSET);
        break;

      case HW_FILTER_RANGE:

        /* Set filter control reg for filter range */

        if (can_type == CAN_MSGPRIO_LOW)
          {
            /* Classical CAN frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRNE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRNB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRFE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRFB;
          }
        else if (can_type == CAN_MSGPRIO_HIGH)
          {
            /* CANFD frame filter */

            fc_reg = (can_id_type == CAN_EXT_ID) ?
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRFE) :
              (fc_reg | MPFS_CANFD_FILTER_CONTROL_FRFB);

            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRNE;
            fc_reg &= ~MPFS_CANFD_FILTER_CONTROL_FRNB;
          }
        putreg32(fc_reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

        /* Set range filter low / high */

        putreg32(fid1, priv->base + MPFS_CANFD_FILTER_RAN_LOW_OFFSET);
        putreg32(fid2, priv->base + MPFS_CANFD_FILTER_RAN_HIGH_OFFSET);
        break;

      default:

        /* Unsupported filter type */

        break;
    }
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/****************************************************************************
 * Name: mpfs_can_reset_hw_filter
 *
 * Description:
 *  Reset all hw filters (both bit and range filter) to default settings
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/
#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static void mpfs_can_reset_hw_filter(FAR struct mpfs_driver_s *priv)
{
  uint32_t reg;

  /* Reset filter control */

  reg = 0;
  reg |= MPFS_CANFD_FILTER_CONTROL_FANB;
  reg |= MPFS_CANFD_FILTER_CONTROL_FANE;
  reg |= MPFS_CANFD_FILTER_CONTROL_FAFB;
  reg |= MPFS_CANFD_FILTER_CONTROL_FAFE;
  putreg32(reg, priv->base + MPFS_CANFD_FILTER_CONTROL_OFFSET);

  /* Reset bit filter A */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_A_VAL_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_A_MASK_OFFSET);

  /* Reset bit filter B */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_B_VAL_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_B_MASK_OFFSET);

  /* Reset bit filter C */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_C_VAL_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_C_MASK_OFFSET);

  /* Reset range filter */

  putreg32(0, priv->base + MPFS_CANFD_FILTER_RAN_LOW_OFFSET);
  putreg32(0, priv->base + MPFS_CANFD_FILTER_RAN_HIGH_OFFSET);

  priv->used_bit_filter_number = 0;
  priv->used_range_filter = false;
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

/****************************************************************************
 * Name: mpfs_can_chip_start
 *
 * Description:
 *  This routine starts the driver. Routine expects that chip is in reset
 *  state. It setups initial Tx buffers for FIFO priorities, sets bittiming,
 *  enables interrupts, switches core to operational mode and changes
 *  controller state to %CAN_STATE_STOPPED.
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success and failure value on error
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_can_chip_start(FAR struct mpfs_driver_s *priv)
{
  uint32_t int_ena, int_msk;
  uint32_t mode_reg;
  int err;
  struct mpfs_can_ctrlmode_s mode;

  /* Initialize txt buffer head tail marker values */

  priv->txb_head = 0;
  priv->txb_tail = 0;

  /* Configure TXT buffers priority */

  priv->txb_prio = 0x01234567;
  putreg32(priv->base, priv->base + MPFS_CANFD_TX_PRIORITY_OFFSET);

  /* Configure bit-rates and ssp */

  err = mpfs_can_set_bittiming(priv);
  if (err < 0)
    {
     return err;
    }

  err = mpfs_can_set_data_bittiming(priv);
  if (err < 0)
    {
      return err;
    }

  err = mpfs_can_set_secondary_sample_point(priv);
  if (err < 0)
    {
      return err;
    }

  /* Configure modes */

  mode.flags = priv->can.ctrlmode;
  mode.mask = 0xffffffff;
  mpfs_can_set_mode(priv, &mode);

  /* Configure interrupts */

  int_ena = MPFS_CANFD_INT_STAT_RBNEI |
            MPFS_CANFD_INT_STAT_TXBHCI |
            MPFS_CANFD_INT_STAT_EWLI |
            MPFS_CANFD_INT_STAT_FCSI |
            MPFS_CANFD_INT_STAT_DOI;

  /* Bus error reporting -> Allow Error/Arb.lost interrupts */

  if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
    {
      int_ena |= MPFS_CANFD_INT_STAT_ALI | MPFS_CANFD_INT_STAT_BEI;
    }

  int_msk = ~int_ena; /* Mask all disabled interrupts */

  /* It's after reset, so there is no need to clear anything */

  uint32_t mask = 0xffffffff;
  putreg32(mask, priv->base + MPFS_CANFD_INT_MASK_CLR_OFFSET);
  putreg32(int_msk, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);
  putreg32(int_ena, priv->base + MPFS_CANFD_INT_ENA_SET_OFFSET);

  /* Put CAN driver to STOPPED state first, CAN controller will enters
   * ERROR_ACTIVE on initial FCSI
   */

  priv->can.state = CAN_STATE_STOPPED;

  /* Enable the CAN controller */

  mode_reg = getreg32(priv->base + MPFS_CANFD_MODE_OFFSET);
  mode_reg |= MPFS_CANFD_MODE_ENA;
  putreg32(mode_reg, priv->base + MPFS_CANFD_MODE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: mpfs_can_chip_stop
 *
 * Description:
 *  This routine stops the driver. This is the drivers stop routine. It will
 * disable the interrupts and disable the controller
 *
 * Input Parameters:
 *  priv  - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static void mpfs_can_chip_stop(FAR struct mpfs_driver_s *priv)
{
  uint32_t mask = 0xffffffff;
  uint32_t mode;

  /* Disable interrupts */

  putreg32(mask, priv->base + MPFS_CANFD_INT_ENA_CLR_OFFSET);
  putreg32(mask, priv->base + MPFS_CANFD_INT_MASK_SET_OFFSET);

  /* Disable the CAN controller */

  mode = getreg32(priv->base + MPFS_CANFD_MODE_OFFSET);
  mode &= ~MPFS_CANFD_MODE_ENA;
  putreg32(mode, priv->base + MPFS_CANFD_MODE_OFFSET);

  /* Set CAN driver state to STOPPED */

  priv->can.state = CAN_STATE_STOPPED;
}

/****************************************************************************
 * Name: mpfs_reset
 *
 * Description:
 *  Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *  priv - Pointer to the private FPGA CANFD driver state structure
 *
 * Returned Value:
 *  OK for success and -ETIMEDOUT for failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_reset(struct mpfs_driver_s *priv)
{
  uint32_t i = 100;
  uint32_t device_id;

  /* Reset FPGA and FIC3, enable clock for FIC3 before RD WR */

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
              SYSREG_SOFT_RESET_CR_FPGA | SYSREG_SOFT_RESET_CR_FIC3,
              0);
  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0, SYSREG_SUBBLK_CLOCK_CR_FIC3);

  /* Reset FPGA CANFD device */

  putreg32(MPFS_CANFD_MODE_RST, priv->base + MPFS_CANFD_MODE_OFFSET);

  /* Check if the device is up again */

  do
    {
      device_id = (getreg32(priv->base + MPFS_CANFD_DEVICE_ID_OFFSET) &
        MPFS_CANFD_DEVICE_ID_DEVICE_ID) >>
        MPFS_CANFD_DEVICE_ID_DEVICE_ID_SHIFT;

      if (device_id == MPFS_CANFD_ID)
        {
          return OK;
        }

      if (!i--)
        {
          canwarn("device did not leave reset\n");
          return -ETIMEDOUT;
        }

      nxsig_usleep(200);
    }
  while (1);
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
 *  Zero (OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ifup(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  if (mpfs_can_chip_start(priv) < 0)
    {
      canerr("chip start failed\n");
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
 * Name: mpfs_ifdown
 *
 * Description:
 *  NuttX Callback: Stop the CAN interface.
 *
 * Input Parameters:
 *  dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno value on failure
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

static int mpfs_ifdown(struct net_driver_s *dev)
{
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;

  /* Stop chip */

  mpfs_can_chip_stop(priv);

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
 *  Zero (OK) on success; a negated errno value on failure.
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
  FAR struct mpfs_driver_s *priv =
    (FAR struct mpfs_driver_s *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
    case SIOCGCANBITRATE:

      /* Get bitrate from the FPGA CANFD controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);
        req->arbi_bitrate = priv->can.bittiming.bitrate / 1000; /* kbit/s */
        req->arbi_samplep = priv->can.bittiming.sample_point / 10;
        req->data_bitrate = priv->can.data_bittiming.bitrate / 1000; /* kbit/s */
        req->data_samplep = priv->can.data_bittiming.sample_point / 10;
        ret = OK;
      }
      break;

    case SIOCSCANBITRATE:

      /* Set bitrate of the FPGA CANFD controller */

      {
        struct can_ioctl_data_s *req =
          (struct can_ioctl_data_s *)((uintptr_t)arg);

        if (req->arbi_bitrate > 1000)
          {
            canerr("Arbitration bitrate > 1Mbps is not supported.");
            ret = -EINVAL;
            break;
          }
        priv->can.bittiming.bitrate = req->arbi_bitrate * 1000; /* bit/s */

        if (req->arbi_samplep > 100 || req->arbi_samplep <= 0)
          {
            canerr("Invalid arbitration sample point. "
                   "Range should be (0,100]%%.");
            ret = -EINVAL;
            break;
          }
        priv->can.bittiming.sample_point =
          req->arbi_samplep * 10; /* In one-tenth of a percent */

        if (req->data_bitrate > 4000 ||
            req->data_bitrate < req->arbi_bitrate)
          {
            canerr("Data bitrate higher than 4Mbps is yet to be supported. "
                   "Data br cannot be smaller than arbitration br.");
            ret = -EINVAL;
            break;
          }
        priv->can.data_bittiming.bitrate = req->data_bitrate * 1000; /* bit/s */

        if (req->data_samplep > 100 || req->data_samplep <= 0)
          {
            canerr("Invalid data sample point. Range should be (0,100]%%.");
            ret = -EINVAL;
            break;
          }
        priv->can.data_bittiming.sample_point =
          req->data_samplep * 10; /* In one-tenth of a percent */

        /* Reset sjw to default 5 */

        priv->can.bittiming.sjw = 5;
        priv->can.data_bittiming.sjw = 5;

        /* Calculate nominal and data bit timing */

        mpfs_can_calc_bittiming(priv,
                                &priv->can.bittiming,
                                priv->can.bittiming_const);
        mpfs_can_calc_bittiming(priv,
                                &priv->can.data_bittiming,
                                priv->can.data_bittiming_const);

        /* Chip stop and start again to write bit timing register */

        mpfs_can_chip_stop(priv);
        if (mpfs_can_chip_start(priv) < 0)
          {
            canerr("Chip start failed.");
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
        uint8_t filter;
        struct can_ioctl_filter_s *req =
          (struct can_ioctl_filter_s *)((uintptr_t)arg);

        if (req->ftype == CAN_FILTER_MASK)
          {
              if (priv->used_bit_filter_number == 0)
            {
              /* Use bit filter A */

              filter = HW_FILTER_A;
            }
          else if (priv->used_bit_filter_number == 1)
            {
              /* Use bit filter B */

              filter = HW_FILTER_B;
            }
          else if (priv->used_bit_filter_number == 2)
            {
              /* Use bit filter C */

              filter = HW_FILTER_C;
            }
          else
            {
              /* All bit filters used. Return with error */

              canerr("All bit filters used. Cannot add more. Delete all and "
                     "add again.");
              ret = -1;
              break;
            }
          priv->used_bit_filter_number++;
          }
        else if (req->ftype == CAN_FILTER_RANGE)
          {
            /* Use range filter */

            filter = HW_FILTER_RANGE;
            priv->used_range_filter = true;
          }
        else
          {
            /* Dual address and other filter types not yet support */

            canerr("Dual address and other filter types not yet support");
            ret = -1;
            break;
          }

        /* Add hw filter */

        mpfs_can_add_hw_filter(priv,
                              filter,
                              CAN_STD_ID,
                              req->fprio, /* LOW for CAN, HIGH for FDCAN */
                              req->fid1,
                              req->fid2);

        ret = OK;
      }
      break;

    case SIOCACANEXTFILTER:

      /* Add CAN EXTENDED ID HW FILTER */

      {
        uint8_t filter;
        struct can_ioctl_filter_s *req =
          (struct can_ioctl_filter_s *)((uintptr_t)arg);

        if (req->ftype == CAN_FILTER_MASK)
          {
              if (priv->used_bit_filter_number == 0)
            {
              /* Use bit filter A */

              filter = HW_FILTER_A;
            }
          else if (priv->used_bit_filter_number == 1)
            {
              /* Use bit filter B */

              filter = HW_FILTER_B;
            }
          else if (priv->used_bit_filter_number == 2)
            {
              /* Use bit filter C */

              filter = HW_FILTER_C;
            }
          else
            {
              /* All bit filters used. Return with error */

              canerr("All bit filters used. Cannot add more. Delete all and "
                     "add again.");
              ret = -1;
              break;
            }
          priv->used_bit_filter_number++;
          }
        else if (req->ftype == CAN_FILTER_RANGE)
          {
            /* Use range filter */

            if (priv->used_range_filter)
              {
                /* The range filter is already used. Return with error */

                canerr("Range filter used. Cannot add more. Delete all and "
                       "add again.");
                ret = -1;
                break;
              }
            filter = HW_FILTER_RANGE;
            priv->used_range_filter = true;
          }
        else
          {
            /* Dual address and other filter types not yet support */

            canerr("Dual address and other filter types not yet support");
            ret = -1;
            break;
          }

        /* Add hw filter */

        mpfs_can_add_hw_filter(priv,
                              filter,
                              CAN_EXT_ID,
                              req->fprio, /* CAN Type: LOW for CAN, HIGH for FDCAN */
                              req->fid1,
                              req->fid2);

        ret = OK;
      }
      break;

    case SIOCDCANSTDFILTER:
    case SIOCDCANEXTFILTER:

      /* Reset all HW FILTERs */

      {
        mpfs_can_reset_hw_filter(priv);
        ret = OK;
      }
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
 * Name: mpfs_fpga_canfd_init
 *
 * Description:
 *  Initialize the CAN controller and driver
 *
 * Returned Value:
 *  On success, a pointer to the MPFS CAN-FD driver is
 *  returned. NULL is returned on any failure.
 *
 * Assumptions:
 *  None
 *
 ****************************************************************************/

int mpfs_fpga_canfd_init(void)
{
  caninfo("Initialize canfd driver...\n");
  struct mpfs_driver_s *priv;
  priv         = &g_canfd;
  memset(priv, 0, sizeof(struct mpfs_driver_s));

  priv->base   = CONFIG_MPFS_CANFD_BASE;
  priv->config = &mpfs_fpga_canfd_config;

  /* Initialize the CAN common private data structure */

  priv->can.state = CAN_STATE_ERROR_ACTIVE;
  priv->ntxbufs = 2;
  priv->can.bittiming_const = &mpfs_can_bit_timing_max;
  priv->can.data_bittiming_const = &mpfs_can_bit_timing_data_max;

  /* Get the can_clk info */

  priv->can.clock.freq = CONFIG_MPFS_CANFD_CLK;

  /* Needed for timing adjustment to be performed as soon as possible */

  priv->can.bittiming.bitrate = CONFIG_MPFS_CANFD_ARBI_BITRATE;
  priv->can.data_bittiming.bitrate = CONFIG_MPFS_CANFD_DATA_BITRATE;

  /* Calculate nominal and data bit timing */

  mpfs_can_calc_bittiming(priv,
                          &priv->can.bittiming,
                          priv->can.bittiming_const);
  mpfs_can_calc_bittiming(priv,
                          &priv->can.data_bittiming,
                          priv->can.data_bittiming_const);

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
  /* Init hw filter runtime var */

  priv->used_bit_filter_number = 0;
  priv->used_range_filter = false;
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

  /* Set CAN control modes */

  priv->can.ctrlmode = CAN_CTRLMODE_FD
                      | CAN_CTRLMODE_BERR_REPORTING;

  /* Attach the interrupt handler */

  if (irq_attach(priv->config->canfd_fpga_irq, mpfs_fpga_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      canerr("ERROR: Failed to attach to FPGA CANFD IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = mpfs_ifup;    /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;  /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail; /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;   /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = priv;         /* Used to recover private state from dev */

  /* Reset chip */

  if (mpfs_reset(priv) < 0)
    {
      return -1;
    }

  caninfo("CAN-FD driver init done\n");

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling mpfs_ifdown().
   */

  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  return OK;
}
