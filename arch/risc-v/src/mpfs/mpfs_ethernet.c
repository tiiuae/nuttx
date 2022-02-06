/****************************************************************************
 * arch/riscv/src/mpfs/mpfs_ethernet.c
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
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <queue.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <crc64.h>
#include "riscv_arch.h"
#include "mpfs_ethernet.h"
#include "mpfs_memorymap.h"

// TODO:
#define MPFS_NETHERNET 4

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

/* Select work queue */

#  if defined(CONFIG_MPFS_ETHMAC_HPWORK)
#    define ETHWORK HPWORK
#  elif defined(CONFIG_MPFS_ETHMAC_LPWORK)
#    define ETHWORK LPWORK
#  else
#    define ETHWORK LPWORK
#  endif
#endif

#ifndef CONFIG_MPFS_PHYADDR
#  error "CONFIG_MPFS_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_MPFS_MII) && !defined(CONFIG_MPFS_RMII)
#  warning "Neither CONFIG_MPFS_MII nor CONFIG_MPFS_RMII defined"
#endif

#if defined(CONFIG_MPFS_MII) && defined(CONFIG_MPFS_RMII)
#  error "Both CONFIG_MPFS_MII and CONFIG_MPFS_RMII defined"
#endif

#ifdef CONFIG_MPFS_AUTONEG
    // TODO:
#endif

/* Timing *******************************************************************/

/* TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define MPFS_WDDELAY     (1*CLK_TCK)

/* TX timeout = 1 minute */

#define MPFS_TXTIMEOUT   (60*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0001998)

/* MAC reset ready delays in loop counts */

#define MAC_READY_USTIMEOUT (200)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct mss_mac_tx_desc mss_mac_tx_desc_t;
struct mss_mac_tx_desc
{
  uint32_t          addr_low;     /* Buffer address low portion */
  volatile uint32_t status;       /* Status and options for transmit operation */
#if defined(MSS_MAC_64_BIT_ADDRESS_MODE)
  uint32_t          addr_high;    /* High portion of address in 64bit addressing mode */
  uint32_t          unused;       /* Unused word in 64bit mode */
#endif
};

typedef struct mss_mac_rx_desc mss_mac_rx_desc_t;
struct mss_mac_rx_desc
{
  uint32_t          addr_low;     /* Buffer address low portion */
  volatile uint32_t status;       /* Status and options for transmit operation */
#if defined(MSS_MAC_64_BIT_ADDRESS_MODE)
  uint32_t          addr_high;    /*!< High portion of address in 64bit addressing mode */
  uint32_t          unused;       /*!< Unused word in 64bit mode */
#endif

};


struct mss_mac_queue_s
{
#if defined(MSS_MAC_USE_DDR)
        mss_mac_tx_desc_t            *tx_desc_tab;  /*!< Transmit descriptor table */
#else
        mss_mac_tx_desc_t            tx_desc_tab[MSS_MAC_TX_RING_SIZE];  /*!< Transmit descriptor table */
#endif

#if defined(MSS_MAC_USE_DDR)
        mss_mac_rx_desc_t            *rx_desc_tab;  /*!< Receive descriptor table */
#else
        mss_mac_rx_desc_t            rx_desc_tab[MSS_MAC_RX_RING_SIZE];  /*!< Receive descriptor table */
#endif
        void                        *tx_caller_info[MSS_MAC_TX_RING_SIZE]; /*!< Pointers to tx user specific data */
        void                        *rx_caller_info[MSS_MAC_RX_RING_SIZE]; /*!< Pointers to rx user specific data */
        //mss_mac_transmit_callback_t  pckt_tx_callback; /*!< Pointer to transmit handler call back function */
        //mss_mac_receive_callback_t   pckt_rx_callback; /*!< Pointer to receive handler call back function */
        volatile uint32_t            nb_available_tx_desc; /*!< Number of free TX descriptors available */
        volatile uint32_t            current_tx_desc; /*!< Oldest in the queue... */
        volatile uint32_t            nb_available_rx_desc; /*!< Number of free RX descriptors available */
        volatile uint32_t            next_free_rx_desc_index; /*!< Next RX descriptor to allocate */
        volatile uint32_t            first_rx_desc_index; /*!< Descriptor to process next when receive handler called */
        uint32_t                     rx_discard; /*!< If set, silently discard incoming packets */
        uint32_t                     overflow_counter; /*!< Overflows since last normal receive operation */
        uint32_t                     tries;  /*!< Keeps track of failure to sends... */
        volatile int32_t             in_isr; /*!< Set when processing ISR so functions don't call PLIC enable/disable for protection */

        /* Queue specific register addresses to simplify the driver code */
        volatile uint32_t           *int_status;        /*!< interrupt status */
        volatile uint32_t           *int_mask;          /*!< interrupt mask */
        volatile uint32_t           *int_enable;        /*!< interrupt enable */
        volatile uint32_t           *int_disable;       /*!< interrupt disable */
        volatile uint32_t           *receive_q_ptr;     /*!< RX queue pointer */
        volatile uint32_t           *transmit_q_ptr;    /*!< TX queue pointer */
        volatile uint32_t           *dma_rxbuf_size;    /*!< RX queue buffer size */

};


/* The mpfs_ethmac_s encapsulates all state information for a single
 * hardware interface
 */

struct mpfs_ethmac_s
{
  uint32_t             is_emac;
  void                 *regbase;    /* */
  irq_t                mac_q_int[MSS_MAC_QUEUE_COUNT];

  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  uint8_t              intf;        /* Ethernet interface number */
  struct wdog_s        txpoll;      /* TX poll timer */
  struct wdog_s        txtimeout;   /* TX timeout timer */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s        pollwork;    /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s  dev;         /* Interface understood by the network */

  /* Used to track transmit and receive descriptors */

  struct mss_mac_queue_s queue[MSS_MAC_QUEUE_COUNT];

  struct eth_desc_s *txhead;        /* Next available TX descriptor */
  struct eth_desc_s *rxhead;        /* Next available RX descriptor */

  struct eth_desc_s *txchbase;      /* TX descriptor ring base address */
  struct eth_desc_s *rxchbase;      /* RX descriptor ring base address */

  struct eth_desc_s *txtail;        /* First "in_flight" TX descriptor */
  struct eth_desc_s *rxcurr;        /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */
};


/* These are the pre-allocated Ethernet device structures */

static struct mpfs_ethmac_s g_mpfsethmac[MPFS_NETHERNET];

static uint32_t *g_regbases[MPFS_NETHERNET] = {
	MSS_MAC0_BASE,
	MSS_EMAC0_BASE,
	MSS_MAC1_BASE,
	MSS_EMAC1_BASE
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_MPFS_ETHMAC_REGDEBUG

#endif

static void mpfs_poll_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  mpfs_ifup(struct net_driver_s *dev);
static int  mpfs_ifdown(struct net_driver_s *dev);

static void mpfs_txavail_work(void *arg);
static int  mpfs_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_PHY_IOCTL
static int  mpfs_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

/* PHY Initialization */

/* MAC/DMA Initialization */

static int  mpfs_macenable(struct mpfs_ethmac_s *priv);
static int  mpfs_ethconfig(struct mpfs_ethmac_s *priv);
static void mpfs_ethreset(struct mpfs_ethmac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_getreg32
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_ETHMAC_REGDEBUG
static uint32_t mpfs_getreg32(uint32_t *addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              ninfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          ninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  ninfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: mac_putreg32
 *
 * Description:
 *
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   offset - The register address to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mac_putreg32(struct mpfs_ethmac_s *priv,
                         uint32_t val, uint32_t offset)
{
  uint8_t *addr = priv->regbase + offset;
  ninfo("%08x<-%08x\n", addr, val);
  putreg32(val, addr);
}






static int mpfs_interrupt_0(int irq, void *context, FAR void *arg)
{
  struct mpfs_ethmac_s *priv = &g_mpfsethmac[0];

  ninfo("IRQ-0");
}
static int mpfs_interrupt_1(int irq, void *context, FAR void *arg)
{
  struct mpfs_ethmac_s *priv = &g_mpfsethmac[0];

  ninfo("IRQ-1");
}
static int mpfs_interrupt_2(int irq, void *context, FAR void *arg)
{
  struct mpfs_ethmac_s *priv = &g_mpfsethmac[0];

  ninfo("IRQ-2");
}
static int mpfs_interrupt_3(int irq, void *context, FAR void *arg)
{
  struct mpfs_ethmac_s *priv = &g_mpfsethmac[0];

  ninfo("IRQ-3");
}

/****************************************************************************
 * Function: stm32_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void mpfs_poll_work(void *arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;
  struct net_driver_s *dev  = &priv->dev;

  net_lock();



  /* Setup the watchdog poll timer again */

  wd_start(&priv->txpoll, MPFS_WDDELAY,
           mpfs_poll_expiry, (wdparm_t)priv);
  net_unlock();
}


/****************************************************************************
 * Function: mpfs_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void mpfs_poll_expiry(wdparm_t arg)
{
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  if (work_available(&priv->pollwork))
    {
      work_queue(ETHWORK, &priv->pollwork, mpfs_poll_work, priv, 0);
    }
  else
    {
      wd_start(&priv->txpoll, MPFS_WDDELAY,
               mpfs_poll_expiry, (wdparm_t)priv);
    }
}



/****************************************************************************
 * Function: mpfs_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
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
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff), (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff), (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Configure the Ethernet interface for DMA operation. */

  ret = mpfs_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set and activate a timer process */

  wd_start(&priv->txpoll, MPFS_WDDELAY,
           mpfs_poll_expiry, (wdparm_t)priv);

  /* Enable the Ethernet interrupts */

  priv->ifup = true;
  up_enable_irq(priv->mac_q_int[0]);
  up_enable_irq(priv->mac_q_int[1]);
  up_enable_irq(priv->mac_q_int[2]);
  up_enable_irq(priv->mac_q_int[3]);

  return OK;
}


/****************************************************************************
 * Function: mpfs_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
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
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(priv->mac_q_int[0]);
  up_disable_irq(priv->mac_q_int[1]);
  up_disable_irq(priv->mac_q_int[2]);
  up_disable_irq(priv->mac_q_int[3]);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txpoll);
  wd_cancel(&priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the mpfs_ifup() always
   * successfully brings the interface back up.
   */

  mpfs_ethreset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);
  return OK;
}


/****************************************************************************
 * Function: mpfs_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
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
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)arg;

  ninfo("ifup: %d\n", priv->ifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      //stm32_dopoll(priv);
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
 * Parameters:
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
  struct mpfs_ethmac_s *priv = (struct mpfs_ethmac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, mpfs_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: mpfs_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mpfs_ethreset(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  /* Reset the Ethernet */

  if (priv->intf == 0)
    {
      /* clocks on */

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET, 0,
                  SYSREG_SUBBLK_CLOCK_CR_MAC0);

      /* reset */

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  0, SYSREG_SOFT_RESET_CR_MAC0);

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_MAC0, 0);
    }

  if (priv->intf == 2)
    {
      /* clocks on */

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET, 0,
                  SYSREG_SUBBLK_CLOCK_CR_MAC1);

      /* reset */

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  0, SYSREG_SOFT_RESET_CR_MAC1);

      modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
                  SYSREG_SOFT_RESET_CR_MAC1, 0);
    }

    /* Disable all queues */

    if (! priv->is_emac)
      {
        *priv->queue[0].receive_q_ptr  = (uint32_t)((uint64_t)priv->queue[0].rx_desc_tab) | 1U;
        *priv->queue[1].receive_q_ptr  = (uint32_t)((uint64_t)priv->queue[1].rx_desc_tab) | 1U;
        *priv->queue[2].receive_q_ptr  = (uint32_t)((uint64_t)priv->queue[2].rx_desc_tab) | 1U;
        *priv->queue[3].receive_q_ptr  = (uint32_t)((uint64_t)priv->queue[3].rx_desc_tab) | 1U;
        *priv->queue[0].transmit_q_ptr = (uint32_t)((uint64_t)priv->queue[0].tx_desc_tab) | 1U;
        *priv->queue[1].transmit_q_ptr = (uint32_t)((uint64_t)priv->queue[1].tx_desc_tab) | 1U;
        *priv->queue[2].transmit_q_ptr = (uint32_t)((uint64_t)priv->queue[2].tx_desc_tab) | 1U;
        *priv->queue[3].transmit_q_ptr = (uint32_t)((uint64_t)priv->queue[3].tx_desc_tab) | 1U;
      }
    else
      {
        *priv->queue[0].receive_q_ptr  = (uint32_t)((uint64_t)priv->queue[0].rx_desc_tab) | 1U;
        *priv->queue[0].transmit_q_ptr = (uint32_t)((uint64_t)priv->queue[0].tx_desc_tab) | 1U;
      }

    //?
    /*
  if(0U == this_mac->is_emac)
  {
    this_mac->mac_base->USER_IO = cfg->tsu_clock_select & 1U;
  }
   */

}

/****************************************************************************
 * Function: mpfs_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_macconfig(struct mpfs_ethmac_s *priv)
{
  uint32_t net_config = 0U;
  uint32_t net_control = 0U;
  uint32_t dma_config = 0U;

  net_control = NETWORK_CONTROL_MAN_PORT_EN |
                NETWORK_CONTROL_CLEAR_ALL_STATS_REGS |
                NETWORK_CONTROL_PFC_ENABLE |
                NETWORK_CONTROL_ALT_SGMII_MODE; //?
  // if loopback enabled
  //net_control |= NETWORK_CONTROL_LOOPBACK_LOCAL;

  net_config = (((uint32_t)(1UL)) << NETWORK_CONFIG_DATA_BUS_WIDTH_SHIFT) |
               ((2 & NETWORK_CONFIG_MDC_CLOCK_DIVISOR_MASK) << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT);

  net_config |= NETWORK_CONFIG_FCS_REMOVE |
                NETWORK_CONFIG_RECEIVE_1536_BYTE_FRAMES;

  mac_putreg32(priv, net_control, NETWORK_CONTROL);
  mac_putreg32(priv, net_config, NETWORK_CONFIG);

  //Reset PCS?

  /* Configure MAC Network DMA Config register */

  dma_config =  (MSS_MAC_RX_BUF_VALUE << DMA_CONFIG_RX_BUF_SIZE_SHIFT) |
                DMA_CONFIG_TX_PBUF_SIZE |
                (((uint32_t)(0x3UL)) << DMA_CONFIG_RX_BUF_SIZE_SHIFT) |
                ((uint32_t)(0x04 & MSS_MAC_AMBA_BURST_MASK));

#if defined(MSS_MAC_64_BIT_ADDRESS_MODE)
  dma_config |= DMA_CONFIG_DMA_ADDR_BUS_WIDTH_1;
#endif

  mac_putreg32(priv, dma_config, DMA_CONFIG);

  for(int queue_index = 1; queue_index < MSS_MAC_QUEUE_COUNT; queue_index++)
    {
      *(priv->queue[queue_index].dma_rxbuf_size) = ((uint32_t)MSS_MAC_RX_BUF_VALUE);
    }

  /*
   * Disable the other queues as the GEM reset leaves them enabled with an
   * address pointer of 0. Setting b0 of the queue pointer disables a queue.
   */

  if (!priv->is_emac)
    {
      mac_putreg32(priv,
                   ((uint32_t)(uint64_t)priv->queue[0].tx_desc_tab) | 1U,
                   TRANSMIT_Q1_PTR);
      mac_putreg32(priv,
                   ((uint32_t)(uint64_t)priv->queue[0].tx_desc_tab) | 1U,
                   TRANSMIT_Q2_PTR);
      mac_putreg32(priv,
                   ((uint32_t)(uint64_t)priv->queue[0].tx_desc_tab) | 1U,
                   TRANSMIT_Q3_PTR);
      mac_putreg32(priv,
                   ((uint32_t)(uint64_t)priv->queue[0].rx_desc_tab) | 1U,
                   RECEIVE_Q1_PTR);
      mac_putreg32(priv,
                   ((uint32_t)(uint64_t)priv->queue[0].rx_desc_tab) | 1U,
                   RECEIVE_Q2_PTR);
      mac_putreg32(priv,
                   ((uint32_t)(uint64_t)priv->queue[0].rx_desc_tab) | 1U,
                   RECEIVE_Q3_PTR);
    }

  // TODO: set jumbo default size

  /* Disable all ints */

  if (priv->is_emac)
    {
      mac_putreg32(priv, 0xFFFFFFFFUL, INT_DISABLE);
    }
  else
    {
      int32_t queue_no;
      for (queue_no = 0; queue_no < MSS_MAC_QUEUE_COUNT; queue_no++)
        {
          // TODO: use putreg()
          *(priv->queue[queue_no].int_disable) = ((uint32_t)0xFFFFFFFFUL);
        }
    }





  return OK;
}


/****************************************************************************
 * Function: mpfs_macenable
 *
 * Description:
 *  Enable normal MAC operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_macenable(struct mpfs_ethmac_s *priv)
{
  uint32_t regval;

}



/****************************************************************************
 * Function: mpfs_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_phyinit(struct mpfs_ethmac_s *priv)
{
    ninfo("TODO");



}


/****************************************************************************
 * Function: mpfs_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mpfs_ethconfig(struct mpfs_ethmac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized early in the boot-up
   * sequence in stm32_rcc.c.
   */

#ifdef CONFIG_MPFS_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = mpfs_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Initialize the free buffer list */

  //stm32_initbuffer(priv, &g_txbuffer[priv->intf * TXBUFFER_SIZE]);

  /* Reset the Ethernet block */

  ninfo("Reset the Ethernet block\n");
  mpfs_ethreset(priv);

  /* Initialize TX Descriptors list */

  //stm32_txdescinit(priv,
  //                 &g_txtable[priv->intf * CONFIG_STM32H7_ETH_NTXDESC]);

  /* Initialize RX Descriptors list */

  //stm32_rxdescinit(priv,
  //                 &g_rxtable[priv->intf * CONFIG_STM32H7_ETH_NRXDESC],
  //                 &g_rxbuffer[priv->intf * RXBUFFER_SIZE]);

  /* Initialize the PHY */

  ninfo("Initialize the PHY\n");
  ret = mpfs_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  ninfo("Initialize the MAC and DMA\n");
  ret = mpfs_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");
  return mpfs_macenable(priv);
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement arm_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_LATEINIT)
int mpfs_ethinitialize(int intf)
#else
static inline int mpfs_ethinitialize(int intf)
#endif
{
  struct mpfs_ethmac_s *priv;
  uint8_t uid[12];
  uint64_t crc;
  int ret = OK;

  ninfo("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < MPFS_NETHERNET);
  priv = &g_mpfsethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct mpfs_ethmac_s));
  priv->dev.d_ifup    = mpfs_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = mpfs_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = mpfs_txavail;   /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = mpfs_addmac;    /* Add multicast MAC address */
  priv->dev.d_rmmac   = mpfs_rmmac;     /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_PHY_IOCTL
  priv->dev.d_ioctl   = mpfs_ioctl;     /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = g_mpfsethmac;   /* Used to recover private state */
  priv->intf          = intf;           /* Remember the interface number */

  switch (intf) {
    case 0:
      priv->is_emac = 0;
      priv->regbase = g_regbases[intf];
      priv->mac_q_int[0] = MPFS_IRQ_MAC0_INT;
      priv->mac_q_int[1] = MPFS_IRQ_MAC0_QUEUE1;
      priv->mac_q_int[2] = MPFS_IRQ_MAC0_QUEUE2;
      priv->mac_q_int[3] = MPFS_IRQ_MAC0_QUEUE3;
      break;
    case 1:
      priv->is_emac = 1;
      priv->regbase = g_regbases[intf];
      priv->mac_q_int[0] = MPFS_IRQ_MAC0_EMAC;
      priv->mac_q_int[1] = 0;
      priv->mac_q_int[2] = 0;
      priv->mac_q_int[3] = 0;
      break;
    case 2:
      priv->is_emac = 0;
      priv->regbase = g_regbases[intf];
      priv->mac_q_int[0] = MPFS_IRQ_MAC1_INT;
      priv->mac_q_int[1] = MPFS_IRQ_MAC1_QUEUE1;
      priv->mac_q_int[2] = MPFS_IRQ_MAC1_QUEUE2;
      priv->mac_q_int[3] = MPFS_IRQ_MAC1_QUEUE3;
      break;
    case 3:
      priv->is_emac = 1;
      priv->regbase = g_regbases[intf];
      priv->mac_q_int[0] = MPFS_IRQ_MAC1_EMAC;
      priv->mac_q_int[1] = 0;
      priv->mac_q_int[2] = 0;
      priv->mac_q_int[3] = 0;
      break;
  }

  if (priv->is_emac == 0)
    {
      priv->queue[0].int_status = priv->regbase + INT_STATUS;
      priv->queue[1].int_status = priv->regbase + INT_Q1_STATUS;
      priv->queue[2].int_status = priv->regbase + INT_Q2_STATUS;
      priv->queue[3].int_status = priv->regbase + INT_Q3_STATUS;
      priv->queue[0].int_mask = priv->regbase + INT_MASK;
      priv->queue[1].int_mask = priv->regbase + INT_Q1_MASK;
      priv->queue[2].int_mask = priv->regbase + INT_Q2_MASK;
      priv->queue[3].int_mask = priv->regbase + INT_Q3_MASK;
      priv->queue[0].int_enable = priv->regbase + INT_ENABLE;
      priv->queue[1].int_enable = priv->regbase + INT_Q1_ENABLE;
      priv->queue[2].int_enable = priv->regbase + INT_Q2_ENABLE;
      priv->queue[3].int_enable = priv->regbase + INT_Q3_ENABLE;
      priv->queue[0].int_disable = priv->regbase + INT_DISABLE;
      priv->queue[1].int_disable = priv->regbase + INT_Q1_DISABLE;
      priv->queue[2].int_disable = priv->regbase + INT_Q2_DISABLE;
      priv->queue[3].int_disable = priv->regbase + INT_Q3_DISABLE;
      priv->queue[0].receive_q_ptr = priv->regbase + RECEIVE_Q_PTR;
      priv->queue[1].receive_q_ptr = priv->regbase + RECEIVE_Q1_PTR;
      priv->queue[2].receive_q_ptr = priv->regbase + RECEIVE_Q2_PTR;
      priv->queue[3].receive_q_ptr = priv->regbase + RECEIVE_Q3_PTR;
      priv->queue[0].transmit_q_ptr = priv->regbase + TRANSMIT_Q_PTR;
      priv->queue[1].transmit_q_ptr = priv->regbase + TRANSMIT_Q1_PTR;
      priv->queue[2].transmit_q_ptr = priv->regbase + TRANSMIT_Q2_PTR;
      priv->queue[3].transmit_q_ptr = priv->regbase + TRANSMIT_Q3_PTR;
      priv->queue[0].dma_rxbuf_size = priv->regbase + DMA_RXBUF_SIZE_Q1;
      priv->queue[1].dma_rxbuf_size = priv->regbase + DMA_RXBUF_SIZE_Q1;
      priv->queue[2].dma_rxbuf_size = priv->regbase + DMA_RXBUF_SIZE_Q2;
      priv->queue[3].dma_rxbuf_size = priv->regbase + DMA_RXBUF_SIZE_Q3;
    }
  else
    {
      // TODO:
    }



  //stm32_get_uniqueid(uid);
  crc = crc64(uid, 12);

  /* Specify as locally administrated address */

  priv->dev.d_mac.ether.ether_addr_octet[0]  = (crc >> 0) | 0x02;
  priv->dev.d_mac.ether.ether_addr_octet[0] &= ~0x1;

  priv->dev.d_mac.ether.ether_addr_octet[1]  = crc >> 8;
  priv->dev.d_mac.ether.ether_addr_octet[2]  = crc >> 16;
  priv->dev.d_mac.ether.ether_addr_octet[3]  = crc >> 24;
  priv->dev.d_mac.ether.ether_addr_octet[4]  = crc >> 32;
  priv->dev.d_mac.ether.ether_addr_octet[5]  = crc >> 40;

  /* Attach the IRQ to the driver */

  if (irq_attach(priv->mac_q_int[0], mpfs_interrupt_0, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  if (irq_attach(priv->mac_q_int[1], mpfs_interrupt_1, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  if (irq_attach(priv->mac_q_int[2], mpfs_interrupt_2, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  if (irq_attach(priv->mac_q_int[3], mpfs_interrupt_3, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }



#ifdef CONFIG_MPFS_PHYINIT
  /* Perform any necessary, board-specific PHY initialization */

  ret = mpfs_phy_boardinitialize(0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Put the interface in the down state. */

  mpfs_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return ret;
}

/****************************************************************************
 * Function: riscv_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in riscv_initialize.c. If MPFS_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of riscv_netinitialize() that calls mpfs_ethinitialize() with
 *   the appropriate interface number.
 *
 * Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void riscv_netinitialize(void)
{
  sam_gmac_initialize();
  //mpfs_ethinitialize(0);
}
#endif


