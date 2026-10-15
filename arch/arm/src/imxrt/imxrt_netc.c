/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_IMXRT_NETC

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <endian.h>
#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/ioctl.h>
#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <arch/barriers.h>
#include "arm_internal.h"
#include "imxrt_clockconfig.h"
#include "imxrt_netc_priv.h"
#include "hardware/imxrt_netc.h"
#ifdef CONFIG_ARM_MPU
#  include "mpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_ARCH_FAMILY_IMXRT118x) || \
    !defined(CONFIG_ARCH_CORTEXM7)
#  error "NETC requires RT118x Cortex-M7"
#endif
#if defined(CONFIG_NETDEV_MULTIBUFFER) || !defined(CONFIG_SCHED_WORKQUEUE)
#  error "NETC requires single-buffer networking and work queues"
#endif
#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARM_MPU)
#  error "Cache-enabled NETC needs an MPU region for DMA memory"
#endif
#if CONFIG_IMXRT_NETC_NTXBUFFERS < 8 || \
    CONFIG_IMXRT_NETC_NTXBUFFERS > 65528 || \
    (CONFIG_IMXRT_NETC_NTXBUFFERS % 8) != 0 || \
    CONFIG_IMXRT_NETC_NRXBUFFERS < 8 || \
    CONFIG_IMXRT_NETC_NRXBUFFERS > 65528 || \
    (CONFIG_IMXRT_NETC_NRXBUFFERS % 8) != 0
#  error "NETC rings must contain 8..65528 descriptors, in multiples of 8"
#endif
#if CONFIG_IMXRT_NETC_LINK_POLL_MS < 1 || CONFIG_IMXRT_NETC_TXTIMEOUT < 1
#  error "NETC polling and timeout intervals must be positive"
#endif

#define NETC_WORK             LPWORK
#define NETC_NTX              CONFIG_IMXRT_NETC_NTXBUFFERS
#define NETC_NRX              CONFIG_IMXRT_NETC_NRXBUFFERS
#define NETC_BUFSIZE          ((CONFIG_NET_ETH_PKTSIZE + 127) & ~127)
#define NETC_TIMEOUT          SEC2TICK(CONFIG_IMXRT_NETC_TXTIMEOUT)
#define NETC_LINK_INTERVAL    MSEC2TICK(CONFIG_IMXRT_NETC_LINK_POLL_MS)
#define NETC_DMA_SIZE         (1u << (32 - __builtin_clz( \
                               sizeof(struct imxrt_netc_dma_s) - 1)))
#define NETC_RESET_US         100000
#define NETC_MDIO_US          1000
#define NETC_BUDGET           16
#define NETC_TXRING           1
#define NETC_RXRING           0
#define NETC_IRQ              IMXRT_IRQ_MSGINTR1
#define NETC_TXREG(r)         (IMXRT_ENETC1_SI0_BASE + \
                               NETC_TX_BASE(NETC_TXRING) + (r))
#define NETC_RXREG(r)         (IMXRT_ENETC1_SI0_BASE + \
                               NETC_RX_BASE(NETC_RXRING) + (r))
#define NETC_SIREG(r)         (IMXRT_ENETC1_SI0_BASE + (r))
#define NETC_EPREG(r)         (IMXRT_ENETC1_BASE + (r))

static_assert(NETC_BUFSIZE >= 128 && NETC_BUFSIZE <= 65408,
               "NETC packet buffer exceeds RBBSR");

/* Only initialization owns shared resets.  Runtime transitions reset PF4,
 * never PF2, PRB or IERB.  The network lock serializes all endpoint state.
 * Lock order: initialization lock -> network lock -> MDIO lock.  Interrupts
 * touch only interrupt routing and work items, never lifecycle or indices.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum imxrt_netc_state_e
{
  NETC_UNINITIALIZED,
  NETC_INITIALIZING,
  NETC_INITFAILED,
  NETC_DOWN,
  NETC_UP,
  NETC_RECOVERING,
  NETC_FAILED
};

enum imxrt_netc_rxstate_e
{
  NETC_RX_PACKET,
  NETC_RX_DISCARD
};

struct imxrt_netc_dma_s
{
  struct imxrt_netc_bd_s tx[NETC_NTX] aligned_data(NETC_DMA_ALIGN);
  struct imxrt_netc_bd_s rx[NETC_NRX] aligned_data(NETC_DMA_ALIGN);
  uint8_t txbuf[NETC_NTX][NETC_BUFSIZE] aligned_data(NETC_DMA_ALIGN);
  uint8_t rxbuf[NETC_NRX][NETC_BUFSIZE] aligned_data(NETC_DMA_ALIGN);
};

static_assert(offsetof(struct imxrt_netc_dma_s, rx) % 128 == 0,
               "RX ring base alignment");

union imxrt_netc_storage_u
{
  struct imxrt_netc_dma_s dma;
  uint8_t padding[NETC_DMA_SIZE];
};

static_assert(sizeof(union imxrt_netc_storage_u) == NETC_DMA_SIZE,
               "NETC DMA MPU region size");

struct imxrt_netc_s
{
  struct net_driver_s dev;
  enum imxrt_netc_state_e state;
  struct imxrt_netc_config_s config;
  struct imxrt_netc_dma_s *dma;
  enum imxrt_netc_rxstate_e rxstate;
#ifdef CONFIG_ARMV7M_DCACHE
  unsigned int mpuregion;
#endif
  uint16_t txprod;
  uint16_t txclean;
  uint16_t rxcons;
  clock_t txtime[NETC_NTX];
  struct wdog_s txwdog;
  struct work_s irqwork;
  struct work_s pollwork;
  struct work_s timeoutwork;
  struct work_s linkwork;
  uint8_t packet[CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE]
    aligned_data(4);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static union imxrt_netc_storage_u g_netc_dma
  locate_data(".netc_dma.packet") aligned_data(NETC_DMA_SIZE);
static struct imxrt_netc_s g_netc;
static mutex_t g_initlock = NXMUTEX_INITIALIZER;
static mutex_t g_mdiolock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void imxrt_netc_interrupt_work(void *arg);
static void imxrt_netc_link_work(void *arg);
static void imxrt_netc_timeout_work(void *arg);
static void imxrt_netc_txtimeout(wdparm_t arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t imxrt_netc_next(uint16_t index, uint16_t length)
{
  return index + 1 == length ? 0 : index + 1;
}

/* Busy waits are deliberately short and bounded, and are never performed
 * with interrupts disabled.  They also work during early board setup.
 */

static int imxrt_netc_wait(uintptr_t reg, uint32_t mask, uint32_t value,
                           unsigned int usec)
{
  do
    {
      if ((getreg32(reg) & mask) == value)
        {
          return OK;
        }

      up_udelay(10);
    }
  while (usec > 10 && (usec -= 10) != 0);

  return -ETIMEDOUT;
}

static void imxrt_netc_enable_function(uintptr_t base)
{
  putreg16(getreg16(base + NETC_PCI_CMD) | NETC_PCI_CMD_ENABLE,
           base + NETC_PCI_CMD);
}

static int imxrt_netc_flr(uintptr_t base)
{
  unsigned int elapsed;

  putreg16(NETC_PCI_MSIX_MASK, base + NETC_PCI_MSIXCTL);
  putreg16(getreg16(base + NETC_PCI_DEVCTL) | NETC_PCI_FLR,
           base + NETC_PCI_DEVCTL);

  for (elapsed = 0; elapsed < NETC_RESET_US; elapsed += 10)
    {
      if ((getreg16(base + NETC_PCI_DEVCTL) & NETC_PCI_FLR) == 0)
        {
          UP_DSB();
          imxrt_netc_enable_function(base);
          return OK;
        }

      up_udelay(10);
    }

  return -ETIMEDOUT;
}

static uintptr_t imxrt_netc_mdiobase(uintptr_t base)
{
  return base == IMXRT_EMDIO_BASE ? IMXRT_NETC_EMDIO : base;
}

static bool imxrt_netc_validmdio(uintptr_t base)
{
  unsigned int port;

  base = imxrt_netc_mdiobase(base);
  if (base == IMXRT_NETC_EMDIO)
    {
      return true;
    }

  /* External port MDIO register groups, not internal PCS MDIO. */

  for (port = 0; port < 4; port++)
    {
      if (base == IMXRT_NETC_PEMDIO(port))
        {
          return true;
        }
    }

  return false;
}

static int imxrt_netc_mdio(uintptr_t base, uint8_t phy, uint8_t reg,
                           uint16_t *value, bool read)
{
  uint32_t status;
  int ret;

  if (!imxrt_netc_validmdio(base) || phy > 31 || reg > 31 || value == NULL)
    {
      return -EINVAL;
    }

  base = imxrt_netc_mdiobase(base);
  ret = nxmutex_lock(&g_mdiolock);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0,
                        NETC_MDIO_US);
  if (ret == OK)
    {
      modifyreg32(base + NETC_MDIO_CFG, NETC_MDIO_C45, 0);
      putreg32(NETC_MDIO_PHY(phy) | reg | (read ? NETC_MDIO_READ : 0),
               base + NETC_MDIO_CTL);
      if (!read)
        {
          putreg32(*value, base + NETC_MDIO_DATA);
        }

      ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0,
                            NETC_MDIO_US);
      if (ret == OK)
        {
          status = getreg32(base + NETC_MDIO_CFG);
          if ((status & NETC_MDIO_ADDR_ERROR) != 0 ||
              (read && (status & NETC_MDIO_READ_ERROR) != 0))
            {
              ret = -EIO;
            }
          else if (read)
            {
              *value = getreg32(base + NETC_MDIO_DATA);
            }
        }
    }

  nxmutex_unlock(&g_mdiolock);
  return ret;
}

int imxrt_netc_mdio_read(uintptr_t base, uint8_t phy, uint8_t reg,
                         uint16_t *value)
{
  return imxrt_netc_mdio(base, phy, reg, value, true);
}

int imxrt_netc_mdio_write(uintptr_t base, uint8_t phy, uint8_t reg,
                          uint16_t value)
{
  return imxrt_netc_mdio(base, phy, reg, &value, false);
}

static int imxrt_netc_infrastructure(struct imxrt_netc_s *priv)
{
  uint32_t frequency;
  uint32_t div;
  unsigned int port;
  int ret;

  /* Board startup owns NETCMIX power, TRDC permissions, root clock setup,
   * pads and PHY reset.  Do not silently change the board's clock tree.
   */

  ret = imxrt_ccm_gate_on(CCM_LPCG_NETC, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_get_rootclock(CCM_CR_NETC, &frequency);
  if (ret < 0 || frequency == 0 || frequency > 1000000000)
    {
      return ret < 0 ? ret : -EINVAL;
    }

  /* Unlocking IERB warm-resets the entire NETC.  Refuse to take over a
   * function already in use by firmware or the other core.
   */

  for (port = 0; port < 5; port++)
    {
      if ((getreg16(0x60000000ul + port * 0x1000 + NETC_PCI_CMD) &
           NETC_PCI_CMD_ENABLE) != 0)
        {
          return -EBUSY;
        }
    }

  modifyreg32(IMXRT_NETC_PRB_BASE + NETC_PRB_NETCRR, NETC_PRB_LOCK, 0);
  ret = imxrt_netc_wait(IMXRT_NETC_PRB_BASE + NETC_PRB_NETCRR,
                        NETC_PRB_LOCK | NETC_PRB_SR, 0, NETC_RESET_US);
  if (ret < 0)
    {
      return ret;
    }

  /* SDK BOARD_NETC_Init: unlock, configure MSI AMQ and PHY bindings, lock,
   * then wait for STATE=0.  Preserve reset resource allotments and bindings.
   */

  modifyreg32(IMXRT_NETC_IERB_BASE + NETC_IERB_RCMSIAMQR,
              NETC_IERB_MSI_AMQ_MASK, NETC_IERB_MSI_AMQ_INIT);
  putreg32((frequency / 1000000) | ((1000000000 / frequency) << 16),
           IMXRT_NETC_IERB_BASE + NETC_IERB_CLKCR);
  putreg32((uint32_t)(((uint64_t)(1000000000 % frequency) << 32) /
                      frequency), IMXRT_NETC_IERB_BASE + NETC_IERB_CLKFR);

  for (port = 0; port < 4; port++)
    {
      if ((priv->config.portmask & (1 << port)) != 0)
        {
          /* Links 0..3 are switch ports 0..3; link 4 is EP0. */

          modifyreg32(IMXRT_NETC_IERB_BASE + NETC_IERB_LINKBCR(port),
                      NETC_IERB_PHYADDR_MASK,
                      (uint32_t)priv->config.ports[port].phyaddr << 8);
        }
    }

  modifyreg32(IMXRT_NETC_PRB_BASE + NETC_PRB_NETCRR, 0, NETC_PRB_LOCK);
  ret = imxrt_netc_wait(IMXRT_NETC_PRB_BASE + NETC_PRB_NETCSR,
                        NETC_PRB_STATE, 0, NETC_RESET_US);
  if (ret < 0)
    {
      return ret;
    }

  if ((getreg32(IMXRT_NETC_PRB_BASE + NETC_PRB_NETCSR) &
       NETC_PRB_ERROR) != 0)
    {
      return -EIO;
    }

  imxrt_netc_enable_function(IMXRT_NETC_PF2_BASE);
  imxrt_netc_enable_function(IMXRT_NETC_PF4_BASE);

  /* ERR051260: NTMP requires both ENETCs and the switch enabled.  PF3 is
   * enabled for register access only, without an SI or DMA ring.
   */

  imxrt_netc_enable_function(IMXRT_NETC_PF3_BASE);
  ret = imxrt_netc_flr(IMXRT_NETC_PF1_BASE);
  if (ret < 0)
    {
      return ret;
    }

  div = (frequency + 4999999) / 5000000;
  if (div > 511)
    {
      return -ERANGE;
    }

  /* 15 NETC cycles hold time, preamble enabled, Clause 22. */

  putreg32(NETC_MDIO_DIV(div) | NETC_MDIO_HOLD(7), IMXRT_NETC_EMDIO);
  for (port = 0; port < 4; port++)
    {
      if ((priv->config.portmask & (1 << port)) != 0)
        {
          putreg32(NETC_MDIO_DIV(div) | NETC_MDIO_HOLD(7),
                   imxrt_netc_mdiobase(priv->config.ports[port].mdio));
        }
    }

  return OK;
}

static int imxrt_netc_dma_initialize(struct imxrt_netc_s *priv)
{
  uintptr_t base = (uintptr_t)&g_netc_dma;
  size_t size = sizeof(g_netc_dma);
#ifdef CONFIG_ARMV7M_DCACHE
  irqstate_t flags;
#endif

  /* OCRAM uses identical CPU and NETC bus addresses. */

  if ((base & (size - 1)) != 0 ||
      !((base >= IMXRT_OCRAM_BASE &&
         base + size <= IMXRT_OCRAM1_BASE + 512 * 1024) ||
        (base >= IMXRT_OCRAM2_BASE &&
         base + size <= IMXRT_OCRAM2_BASE + 256 * 1024)))
    {
      nerr("Invalid NETC OCRAM DMA placement: %p\n", &g_netc_dma);
      return -EFAULT;
    }

#ifdef CONFIG_ARMV7M_DCACHE
  /* Map the padded storage alone, keeping USB and heap attributes intact. */

  up_clean_dcache(base, base + size);
  up_invalidate_dcache(base, base + size);
  flags = enter_critical_section();
  priv->mpuregion = mpu_configure_region(base, size,
                                        MPU_RASR_TEX_NOR | MPU_RASR_S |
                                        MPU_RASR_AP_RWNO | MPU_RASR_XN);
  leave_critical_section(flags);
  UP_DSB();
  UP_ISB();
#endif

  memset(&g_netc_dma, 0, size);
  priv->dma = &g_netc_dma.dma;
  return OK;
}

static void imxrt_netc_dma_release(struct imxrt_netc_s *priv)
{
#ifdef CONFIG_ARMV7M_DCACHE
  irqstate_t flags;
#endif

  if (priv->dma != NULL)
    {
#ifdef CONFIG_ARMV7M_DCACHE
      flags = enter_critical_section();
      mpu_freeregion(priv->mpuregion);
      leave_critical_section(flags);
      UP_DSB();
      UP_ISB();
#endif
      priv->dma = NULL;
    }
}

/* Called with net_lock held.  PF4 FLR completion is the ownership fence for
 * ALL endpoint DMA, including prefetched descriptors and outstanding writes.
 * Clearing ring EN alone is not a safe RX ownership fence.
 */

static int imxrt_netc_stop(struct imxrt_netc_s *priv)
{
  int ret;
  int resetret;

  up_disable_irq(NETC_IRQ);
  wd_cancel(&priv->txwdog);
  ret = imxrt_netc_switch_cpu(false, priv->dev.d_mac.ether.ether_addr_octet);

  /* ERR051246: switch CPU-port Rx/Tx must be disabled BEFORE PF4 FLR.
   * Do not proceed if the switch helper cannot establish that condition.
   */

  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(IMXRT_ENETC1_PORT_BASE + NETC_PORT_POR, 0,
              NETC_PORT_TXDIS | NETC_PORT_RXDIS);
  resetret = imxrt_netc_flr(IMXRT_NETC_PF4_BASE);
  if (resetret == OK)
    {
      priv->dev.d_len = 0;
      priv->txprod = 0;
      priv->txclean = 0;
      priv->rxcons = 0;
      priv->rxstate = NETC_RX_PACKET;
    }

  return resetret;
}

static int imxrt_netc_start(struct imxrt_netc_s *priv)
{
  const uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;
  uintptr_t address;
  uint32_t mac0;
  uint32_t mac1;
  unsigned int i;
  int ret;

  if ((mac[0] & 1) != 0 ||
      (mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]) == 0)
    {
      return -EINVAL;
    }

  ret = imxrt_netc_stop(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Safe to overwrite rings only after successful FLR. */

  memset(priv->dma->tx, 0, sizeof(priv->dma->tx));
  memset(priv->dma->rx, 0, sizeof(priv->dma->rx));
  for (i = 0; i < NETC_NRX; i++)
    {
      address = (uintptr_t)priv->dma->rxbuf[i];
      priv->dma->rx[i].word[0] = htole32(address);
    }

  mac0 = (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
         ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
  mac1 = (uint32_t)mac[4] | ((uint32_t)mac[5] << 8);
  putreg32(mac0, NETC_EPREG(NETC_ENETC_PSIPMAR0));
  putreg32(mac1, NETC_EPREG(NETC_ENETC_PSIPMAR1));
  putreg32(mac0, IMXRT_ENETC1_PORT_BASE + 0x20);
  putreg32(mac1, IMXRT_ENETC1_PORT_BASE + 0x24);

  /* SI0 owns rings 0/1; TX0 is reserved for direct switch enqueue.  RX1
   * remains disabled.  All host reasons map to RX0, no timestamp requests.
   */

  putreg32(2 | (2u << 16) | (1u << 28),
           NETC_EPREG(NETC_ENETC_PSICFGR0));
  putreg32(0, NETC_EPREG(NETC_ENETC_PSICFGR2));
  putreg32(NETC_ENETC_MCPROMISC, NETC_EPREG(NETC_ENETC_PSIPMMR));
#ifdef CONFIG_NET_PROMISCUOUS
  modifyreg32(NETC_EPREG(NETC_ENETC_PSIPMMR), 0, NETC_ENETC_UCPROMISC);
#endif
  putreg32(NETC_ENETC_VLANPROMISC | NETC_ENETC_VLANUNTAGGED,
           NETC_EPREG(NETC_ENETC_PSIPVMR));
  putreg32(NETC_ENETC_CHECKSUM_OFF, NETC_EPREG(NETC_ENETC_PARCSCR));
  putreg32(0, NETC_EPREG(NETC_ENETC_PRIO2TCMR0));
  for (i = 1; i <= 15; i++)
    {
      putreg32(0, NETC_EPREG(NETC_ENETC_SMHRBDRMR(i)));
    }

  putreg32(NETC_PSEUDO_TXPAD,
           IMXRT_ENETC1_PSEUDO_MAC_BASE + NETC_PSEUDO_PPMCR);
  putreg32(1, NETC_SIREG(NETC_SI_SIRBGCR));
  putreg32(0, NETC_SIREG(NETC_SI_SIIPVBDRMR0));
  putreg32(0, NETC_SIREG(NETC_SI_SIBCAR));

  address = (uintptr_t)priv->dma->tx;
  putreg32(address, NETC_TXREG(NETC_BDR_BAR0));
  putreg32(0, NETC_TXREG(NETC_BDR_BAR1));
  putreg32(0, NETC_TXREG(NETC_BDR_PIR));
  putreg32(0, NETC_TXREG(NETC_TX_CIR));
  putreg32(NETC_NTX, NETC_TXREG(NETC_BDR_LENR));
  putreg32(0, NETC_TXREG(NETC_BDR_IER));
  putreg32(0, NETC_TXREG(NETC_BDR_ICR0));
  putreg32(0, NETC_TXREG(NETC_BDR_ICR1));

  address = (uintptr_t)priv->dma->rx;
  putreg32(address, NETC_RXREG(NETC_BDR_BAR0));
  putreg32(0, NETC_RXREG(NETC_BDR_BAR1));
  putreg32(NETC_BUFSIZE, NETC_RXREG(NETC_RX_BSR));
  putreg32(0, NETC_RXREG(NETC_BDR_PIR));
  putreg32(0, NETC_RXREG(NETC_RX_CIR));
  putreg32(NETC_NRX, NETC_RXREG(NETC_BDR_LENR));
  putreg32(0, NETC_RXREG(NETC_BDR_IER));
  putreg32(NETC_RX_COALESCE | NETC_RX_THRESHOLD,
           NETC_RXREG(NETC_BDR_ICR0));
  putreg32(0, NETC_RXREG(NETC_BDR_ICR1));

  /* MSGINTR1, slot 0, message bit 0 is reserved for this driver.  MSIR
   * read clears pending bits.  NETC's functional interrupts use MSI-X;
   * the separate NETC Adv Error Reporting interrupt must not be treated
   * as a packet IRQ.
   */

  getreg32(IMXRT_MSGINTR1_BASE + NETC_MSGINTR_MSIR);
  putreg32(IMXRT_MSGINTR1_BASE + NETC_MSGINTR_MSIIR,
           IMXRT_NETC_ENETC1_MSIX);
  putreg32(0, IMXRT_NETC_ENETC1_MSIX + 4);
  putreg32(0, IMXRT_NETC_ENETC1_MSIX + 8);
  putreg32(0, IMXRT_NETC_ENETC1_MSIX + 12);
  putreg32(0, NETC_SIREG(NETC_SI_SIMSITRVR(NETC_TXRING)));
  putreg32(0, NETC_SIREG(NETC_SI_SIMSIRRVR(NETC_RXRING)));
  putreg16(NETC_PCI_MSIX_ENABLE, IMXRT_NETC_PF4_BASE + NETC_PCI_MSIXCTL);

  UP_DSB();
  putreg32(NETC_BDR_EN, NETC_TXREG(NETC_BDR_MR));
  putreg32(NETC_BDR_EN | NETC_RX_LOSSY, NETC_RXREG(NETC_BDR_MR));
  putreg32(NETC_ENETC_SI0EN, NETC_EPREG(NETC_ENETC_PMR));
  putreg32(NETC_SI_EN, NETC_SIREG(NETC_SI_SIMR));
  modifyreg32(IMXRT_ENETC1_PORT_BASE + NETC_PORT_POR,
              NETC_PORT_TXDIS | NETC_PORT_RXDIS, 0);

  ret = imxrt_netc_switch_cpu(true, mac);
  if (ret < 0)
    {
      /* Do not reuse any memory unless this stop also completes. */

      imxrt_netc_stop(priv);
    }

  return ret;
}

static bool imxrt_netc_txfull(struct imxrt_netc_s *priv)
{
  return imxrt_netc_next(priv->txprod, NETC_NTX) == priv->txclean;
}

static void imxrt_netc_watchdog(struct imxrt_netc_s *priv)
{
  clock_t age;

  wd_cancel(&priv->txwdog);
  if (priv->txclean != priv->txprod)
    {
      age = clock_systime_ticks() - priv->txtime[priv->txclean];
      wd_start(&priv->txwdog, age >= NETC_TIMEOUT ? 1 : NETC_TIMEOUT - age,
               imxrt_netc_txtimeout, (wdparm_t)priv);
    }
}

static void imxrt_netc_txdone(struct imxrt_netc_s *priv)
{
  uint32_t flags;
  uint16_t consumer = getreg32(NETC_TXREG(NETC_TX_CIR)) & 0xffff;
  unsigned int budget = NETC_BUDGET;

  if (consumer >= NETC_NTX)
    {
      return;
    }

  UP_DSB();
  while (priv->txclean != consumer && budget-- > 0)
    {
      flags = le32toh(priv->dma->tx[priv->txclean].word[3]);
      if ((flags & NETC_TX_WRITTEN) != 0 &&
          (flags & NETC_TX_STATUS_MASK) != 0)
        {
          NETDEV_TXERRORS(&priv->dev);
        }
      else
        {
          NETDEV_TXDONE(&priv->dev);
        }

      priv->txclean = imxrt_netc_next(priv->txclean, NETC_NTX);
    }

  imxrt_netc_watchdog(priv);
}

static int imxrt_netc_transmit(struct imxrt_netc_s *priv)
{
  struct imxrt_netc_bd_s *bd;
  uint16_t index = priv->txprod;
  uint16_t length = priv->dev.d_len;

  if (imxrt_netc_txfull(priv))
    {
      return -EBUSY;
    }

  if (length == 0 || length > CONFIG_NET_ETH_PKTSIZE)
    {
      NETDEV_TXERRORS(&priv->dev);
      priv->dev.d_len = 0;
      return -EMSGSIZE;
    }

  memcpy(priv->dma->txbuf[index], priv->dev.d_buf, length);
  bd = &priv->dma->tx[index];
  bd->word[0] = htole32((uintptr_t)priv->dma->txbuf[index]);
  bd->word[1] = 0;
  bd->word[2] = htole32((uint32_t)length | ((uint32_t)length << 16));
  bd->word[3] = htole32(NETC_TX_FINAL | NETC_TX_INTERRUPT);
  priv->txtime[index] = clock_systime_ticks();
  priv->txprod = imxrt_netc_next(index, NETC_NTX);

  /* Complete non-cacheable packet and BD stores before the doorbell. */

  UP_DSB();
  putreg32(priv->txprod, NETC_TXREG(NETC_BDR_PIR));
  NETDEV_TXPACKETS(&priv->dev);
  priv->dev.d_len = 0;
  imxrt_netc_watchdog(priv);
  return OK;
}

static int imxrt_netc_txpoll(struct net_driver_s *dev)
{
  struct imxrt_netc_s *priv = dev->d_private;

  if (dev->d_len > 0)
    {
      imxrt_netc_transmit(priv);
    }

  return imxrt_netc_txfull(priv) ? -EBUSY : OK;
}

static void imxrt_netc_dispatch(struct imxrt_netc_s *priv)
{
  struct eth_hdr_s *eth = (struct eth_hdr_s *)priv->dev.d_buf;

  NETDEV_RXPACKETS(&priv->dev);
#ifdef CONFIG_NET_PKT
  pkt_input(&priv->dev);
#endif
#ifdef CONFIG_NET_IPv4
  if (eth->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(&priv->dev);
      ipv4_input(&priv->dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (eth->type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(&priv->dev);
      ipv6_input(&priv->dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (eth->type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(&priv->dev);
      arp_input(&priv->dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->dev);
      priv->dev.d_len = 0;
    }

  if (priv->dev.d_len > 0)
    {
      imxrt_netc_transmit(priv);
    }
}

static void imxrt_netc_receive(struct imxrt_netc_s *priv)
{
  struct imxrt_netc_bd_s *bd;
  uint32_t flags;
  uint16_t length;
  uint16_t index;
  uint16_t producer;
  unsigned int budget = NETC_BUDGET;

  /* Reserve a TX slot for synchronous protocol replies.  On TX exhaustion,
   * leave RX owned by software rather than overwrite a deferred reply.
   */

  while (budget-- > 0 && !imxrt_netc_txfull(priv) && priv->dev.d_len == 0)
    {
      producer = getreg32(NETC_RXREG(NETC_BDR_PIR)) & 0xffff;
      if (producer >= NETC_NRX || producer == priv->rxcons)
        {
          break;
        }

      UP_DSB();
      index = priv->rxcons;
      bd = &priv->dma->rx[index];
      flags = le32toh(bd->word[3]);
      length = le32toh(bd->word[2]) & NETC_RX_LENGTH_MASK;

      /* A non-final BD starts an oversized/chained frame.  Track discard
       * across budget boundaries, never dispatch its last fragment alone.
       * Only the first BD of a chain is required to have READY set.
       */

      if (priv->rxstate == NETC_RX_DISCARD ||
          (flags & (NETC_RX_READY | NETC_RX_FINAL)) !=
          (NETC_RX_READY | NETC_RX_FINAL))
        {
          if (priv->rxstate == NETC_RX_PACKET)
            {
              NETDEV_RXERRORS(&priv->dev);
            }

          priv->rxstate = (flags & NETC_RX_FINAL) != 0 ?
                          NETC_RX_PACKET : NETC_RX_DISCARD;
        }
      else if ((flags & NETC_RX_ERROR_MASK) != 0 ||
          length < sizeof(struct eth_hdr_s) ||
          length > CONFIG_NET_ETH_PKTSIZE)
        {
          NETDEV_RXERRORS(&priv->dev);
        }
      else
        {
          memcpy(priv->dev.d_buf, priv->dma->rxbuf[index], length);
          priv->dev.d_len = length;
        }

      bd->word[0] = htole32((uintptr_t)priv->dma->rxbuf[index]);
      bd->word[1] = 0;
      bd->word[2] = 0;
      bd->word[3] = 0;
      priv->rxcons = imxrt_netc_next(index, NETC_NRX);
      UP_DSB();
      putreg32(priv->rxcons, NETC_RXREG(NETC_RX_CIR));
      if (priv->dev.d_len != 0)
        {
          imxrt_netc_dispatch(priv);
        }
    }
}

static void imxrt_netc_carrier(struct imxrt_netc_s *priv, uint8_t links)
{
  if (priv->state == NETC_UP && (links & priv->config.portmask) != 0)
    {
      netdev_carrier_on(&priv->dev);
    }
  else
    {
      netdev_carrier_off(&priv->dev);
    }
}

static void imxrt_netc_recover(struct imxrt_netc_s *priv)
{
  uint8_t links = 0;
  int ret;

  priv->state = NETC_RECOVERING;
  netdev_carrier_off(&priv->dev);
  ret = imxrt_netc_start(priv);
  priv->state = ret < 0 ? NETC_FAILED : NETC_UP;
  if (ret < 0)
    {
      nerr("NETC PF4 recovery failed: %d; DMA memory retained\n", ret);
    }
  else
    {
      imxrt_netc_switch_link(&links);
      imxrt_netc_carrier(priv, links);
    }
}

/* The generic SIOCSIFHWADDR updates d_mac without a driver callback.
 * Compare against hardware so active MAC changes do not leave a stale CPU
 * FDB entry.  Down/up and FLR also reinstall the current address in start.
 */

static int imxrt_netc_refreshmac(struct imxrt_netc_s *priv)
{
  const uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;
  uint32_t mac0;
  uint32_t mac1;
  int ret;

  mac0 = (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
         ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
  mac1 = (uint32_t)mac[4] | ((uint32_t)mac[5] << 8);
  if (getreg32(NETC_EPREG(NETC_ENETC_PSIPMAR0)) == mac0 &&
      (getreg32(NETC_EPREG(NETC_ENETC_PSIPMAR1)) & 0xffff) == mac1)
    {
      return OK;
    }

  if ((mac[0] & 1) != 0 || (mac0 == 0 && mac1 == 0))
    {
      return -EINVAL;
    }

  ret = imxrt_netc_switch_cpu(true, mac);
  if (ret == OK)
    {
      putreg32(mac0, NETC_EPREG(NETC_ENETC_PSIPMAR0));
      putreg32(mac1, NETC_EPREG(NETC_ENETC_PSIPMAR1));
      putreg32(mac0, IMXRT_ENETC1_PORT_BASE + 0x20);
      putreg32(mac1, IMXRT_ENETC1_PORT_BASE + 0x24);
    }

  return ret;
}

static void imxrt_netc_service(struct imxrt_netc_s *priv)
{
  uint32_t errors;
  int ret;

  if (priv->state != NETC_UP)
    {
      return;
    }

  ret = imxrt_netc_refreshmac(priv);
  if (ret < 0)
    {
      priv->state = NETC_FAILED;
      netdev_carrier_off(&priv->dev);
      imxrt_netc_stop(priv);
      nerr("NETC CPU MAC update failed: %d\n", ret);
      return;
    }

  errors = (getreg32(NETC_TXREG(NETC_BDR_SR)) |
            getreg32(NETC_RXREG(NETC_BDR_SR))) & NETC_BDR_SBE;
  if (errors != 0)
    {
      NETDEV_TXERRORS(&priv->dev);
      imxrt_netc_recover(priv);
      if (priv->state != NETC_UP)
        {
          return;
        }
    }

  imxrt_netc_txdone(priv);
  if (priv->dev.d_len != 0)
    {
      imxrt_netc_transmit(priv);
    }

  imxrt_netc_receive(priv);
  if (!imxrt_netc_txfull(priv) && priv->dev.d_len == 0)
    {
      devif_poll(&priv->dev, imxrt_netc_txpoll);
    }

  /* Ack BEFORE unmask, then recheck producer/consumer.  An event arriving
   * during the masked interval is found either here or by pending MSI-X.
   */

  putreg32(NETC_SI_TX_FRAME(NETC_TXRING), NETC_SIREG(NETC_SI_SITXIDR0));
  putreg32(NETC_SI_RX_RING(NETC_RXRING), NETC_SIREG(NETC_SI_SIRXIDR0));
  putreg32(NETC_TX_FRAME_IE, NETC_TXREG(NETC_BDR_IER));
  putreg32(NETC_RX_THRESHOLD_IE, NETC_RXREG(NETC_BDR_IER));
  UP_DSB();
  up_enable_irq(NETC_IRQ);

  if (!imxrt_netc_txfull(priv) &&
      ((getreg32(NETC_RXREG(NETC_BDR_PIR)) & 0xffff) != priv->rxcons ||
       (getreg32(NETC_TXREG(NETC_TX_CIR)) & 0xffff) != priv->txclean))
    {
      /* Bound packet work per invocation.  The periodic worker also services
       * rings if this enqueue fails, so a masked IRQ cannot strand packets.
       */

      work_queue(NETC_WORK, &priv->irqwork,
                 imxrt_netc_interrupt_work, priv, 0);
    }
}

static void imxrt_netc_interrupt_work(void *arg)
{
  struct imxrt_netc_s *priv = arg;

  net_lock();
  imxrt_netc_service(priv);
  net_unlock();
}

static int imxrt_netc_interrupt(int irq, void *context, void *arg)
{
  struct imxrt_netc_s *priv = arg;

  up_disable_irq(NETC_IRQ);
  getreg32(IMXRT_MSGINTR1_BASE + NETC_MSGINTR_MSIR);

  /* -EBUSY means already queued; other failures are recovered by linkwork.
   * Do not unmask a level IRQ on enqueue failure and create an IRQ storm.
   */

  work_queue(NETC_WORK, &priv->irqwork, imxrt_netc_interrupt_work, priv, 0);
  return OK;
}

static void imxrt_netc_txtimeout(wdparm_t arg)
{
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)arg;

  work_queue(NETC_WORK, &priv->timeoutwork,
             imxrt_netc_timeout_work, priv, 0);
}

static void imxrt_netc_checktimeout(struct imxrt_netc_s *priv)
{
  if (priv->state == NETC_UP && priv->txclean != priv->txprod &&
      clock_systime_ticks() - priv->txtime[priv->txclean] >= NETC_TIMEOUT)
    {
      /* Reclaim first: a delayed worker is not evidence of stalled DMA. */

      imxrt_netc_txdone(priv);
      if (priv->txclean != priv->txprod &&
          (getreg32(NETC_TXREG(NETC_TX_CIR)) & 0xffff) == priv->txclean &&
          clock_systime_ticks() - priv->txtime[priv->txclean] >=
          NETC_TIMEOUT)
        {
          NETDEV_TXTIMEOUTS(&priv->dev);
          imxrt_netc_recover(priv);
        }
    }
}

static void imxrt_netc_timeout_work(void *arg)
{
  struct imxrt_netc_s *priv = arg;

  net_lock();
  imxrt_netc_checktimeout(priv);
  imxrt_netc_service(priv);
  net_unlock();
}

static void imxrt_netc_link_work(void *arg)
{
  struct imxrt_netc_s *priv = arg;
  uint8_t links = 0;
  int ret;

  net_lock();
  if (priv->state == NETC_UNINITIALIZED ||
      priv->state == NETC_INITIALIZING || priv->state == NETC_INITFAILED)
    {
      net_unlock();
      return;
    }

  /* External switching and link updates continue even with the CPU down.
   * Switch helper performs one bounded negotiation/status sample per port.
   */

  ret = imxrt_netc_switch_link(&links);
  if (ret < 0)
    {
      nerr("NETC port polling failed: %d\n", ret);
    }

  imxrt_netc_carrier(priv, links);
  imxrt_netc_checktimeout(priv);
  imxrt_netc_service(priv);
  net_unlock();

  ret = work_queue(NETC_WORK, &priv->linkwork, imxrt_netc_link_work, priv,
                   NETC_LINK_INTERVAL > 0 ? NETC_LINK_INTERVAL : 1);
  if (ret < 0)
    {
      nerr("NETC link monitor enqueue failed: %d\n", ret);
    }
}

static int imxrt_netc_ifup(struct net_driver_s *dev)
{
  struct imxrt_netc_s *priv = dev->d_private;
  uint8_t links = 0;
  int ret;

  if (priv->state == NETC_UP)
    {
      return OK;
    }

  if (priv->state != NETC_DOWN)
    {
      return -EIO;
    }

  ret = imxrt_netc_start(priv);
  if (ret < 0)
    {
      priv->state = NETC_FAILED;
      netdev_carrier_off(dev);
      return ret;
    }

  priv->state = NETC_UP;
  imxrt_netc_switch_link(&links);
  imxrt_netc_carrier(priv, links);
  imxrt_netc_service(priv);
  return priv->state == NETC_UP ? OK : -EIO;
}

static int imxrt_netc_ifdown(struct net_driver_s *dev)
{
  struct imxrt_netc_s *priv = dev->d_private;
  int ret;

  if (priv->state == NETC_DOWN)
    {
      return OK;
    }

  /* Callbacks hold net_lock.  Never synchronously drain work needing it.
   * A worker already running/queued takes the same lock and checks state.
   * Work surviving a later ifup can service the new rings; timeout
   * work verifies current oldest submission time rather than a stale flag.
   */

  priv->state = NETC_DOWN;
  netdev_carrier_off(dev);
  work_cancel(NETC_WORK, &priv->irqwork);
  work_cancel(NETC_WORK, &priv->pollwork);
  work_cancel(NETC_WORK, &priv->timeoutwork);
  ret = imxrt_netc_stop(priv);
  if (ret < 0)
    {
      priv->state = NETC_FAILED;
    }

  return ret;
}

static int imxrt_netc_txavail(struct net_driver_s *dev)
{
  struct imxrt_netc_s *priv = dev->d_private;

  /* Work submission is safe even without net_lock; worker checks lifecycle.
   * Periodic service supplies a fallback for queue resource failure.
   */

  work_queue(NETC_WORK, &priv->pollwork, imxrt_netc_interrupt_work, priv, 0);
  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
static int imxrt_netc_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  /* All multicast is admitted on SI0 and flooded to CPU by the switch.
   * No finite hash table or reference-counted group state is needed.
   */

  return (mac[0] & 1) != 0 ? OK : -EINVAL;
}

static int imxrt_netc_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  return imxrt_netc_addmac(dev, mac);
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int imxrt_netc_ioctl(struct net_driver_s *dev, int cmd,
                            unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct imxrt_netc_s *priv = dev->d_private;
  struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)(uintptr_t)arg;
  const struct imxrt_netc_port_s *port = NULL;
  unsigned int i;

  if (req == NULL)
    {
      return -EINVAL;
    }

  if (cmd != SIOCGMIIPHY && cmd != SIOCGMIIREG && cmd != SIOCSMIIREG)
    {
      return -ENOTTY;
    }

  /* Aggregate interface: SIOCGMIIPHY selects the first enabled port.
   * Register access selects an enabled PHY by its configured address.
   * Reject ambiguous addresses on different buses rather than guess.
   */

  for (i = 0; i < 4; i++)
    {
      if ((priv->config.portmask & (1 << i)) != 0 &&
          (cmd == SIOCGMIIPHY ||
           priv->config.ports[i].phyaddr == req->phy_id))
        {
          if (port != NULL && port->mdio != priv->config.ports[i].mdio)
            {
              return -EINVAL;
            }

          port = &priv->config.ports[i];
          if (cmd == SIOCGMIIPHY)
            {
              req->phy_id = port->phyaddr;
              return OK;
            }
        }
    }

  if (port == NULL)
    {
      return -ENODEV;
    }

  if (req->reg_num > 31)
    {
      return -EINVAL;
    }

  switch (cmd)
    {
      case SIOCGMIIREG:
        return imxrt_netc_mdio_read(port->mdio, port->phyaddr, req->reg_num,
                                    &req->val_out);
      case SIOCSMIIREG:
        return imxrt_netc_mdio_write(port->mdio, port->phyaddr, req->reg_num,
                                     req->val_in);
      default:
        break;
    }
#endif

  return -ENOTTY;
}
#endif

static int imxrt_netc_validate(const struct imxrt_netc_config_s *config)
{
  const struct imxrt_netc_port_s *port;
  unsigned int i;
  unsigned int j;
  uint8_t bits = 0;

  if (config == NULL || config->portmask == 0 ||
      (config->portmask & ~15) != 0 || (config->mac[0] & 1) != 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < 6; i++)
    {
      bits |= config->mac[i];
    }

  if (bits == 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < 4; i++)
    {
      if ((config->portmask & (1 << i)) == 0)
        {
          continue;
        }

      port = &config->ports[i];
      if (!imxrt_netc_validmdio(port->mdio) || port->phyaddr > 31 ||
          (port->mode != IMXRT_NETC_MII && port->mode != IMXRT_NETC_RMII &&
           port->mode != IMXRT_NETC_RGMII) ||
          (port->phy != IMXRT_NETC_PHY_RTL8201 &&
           port->phy != IMXRT_NETC_PHY_RTL8211) ||
          (port->mode != IMXRT_NETC_RGMII &&
           (port->rxdelay || port->txdelay)))
        {
          return -EINVAL;
        }

      for (j = 0; j < i; j++)
        {
          if ((config->portmask & (1 << j)) != 0 &&
              imxrt_netc_mdiobase(config->ports[j].mdio) ==
              imxrt_netc_mdiobase(port->mdio) &&
              config->ports[j].phyaddr == port->phyaddr)
            {
              return -EINVAL;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_initialize(const struct imxrt_netc_config_s *config)
{
  struct imxrt_netc_s *priv = &g_netc;
  int ret;

  ret = imxrt_netc_validate(config);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&g_initlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->state != NETC_UNINITIALIZED)
    {
      nxmutex_unlock(&g_initlock);
      return -EALREADY;
    }

  priv->state = NETC_INITIALIZING;
  priv->config = *config;
  priv->dev.d_buf = priv->packet;
  priv->dev.d_private = priv;
  memcpy(priv->dev.d_mac.ether.ether_addr_octet, config->mac, 6);
  priv->dev.d_ifup = imxrt_netc_ifup;
  priv->dev.d_ifdown = imxrt_netc_ifdown;
  priv->dev.d_txavail = imxrt_netc_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac = imxrt_netc_addmac;
  priv->dev.d_rmmac = imxrt_netc_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl = imxrt_netc_ioctl;
#endif

  ret = imxrt_netc_dma_initialize(priv);
  if (ret < 0)
    {
      goto failed;
    }

  ret = imxrt_netc_infrastructure(priv);
  if (ret < 0)
    {
      goto free_dma;
    }

  ret = imxrt_netc_switch_initialize(&priv->config);
  if (ret < 0)
    {
      goto free_dma;
    }

  net_lock();
  ret = imxrt_netc_stop(priv);
  net_unlock();
  if (ret < 0)
    {
      goto free_dma;
    }

  ret = irq_attach(NETC_IRQ, imxrt_netc_interrupt, priv);
  if (ret < 0)
    {
      goto free_dma;
    }

  /* Queue link monitoring before exposing an interface.  Worker cannot use
   * a half-registered dev because it also takes the network lock.
   */

  net_lock();
  ret = work_queue(NETC_WORK, &priv->linkwork, imxrt_netc_link_work, priv,
                   NETC_LINK_INTERVAL > 0 ? NETC_LINK_INTERVAL : 1);
  if (ret == OK)
    {
      priv->state = NETC_DOWN;
      ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
    }

  if (ret < 0)
    {
      priv->state = NETC_INITFAILED;
      work_cancel(NETC_WORK, &priv->linkwork);
    }

  net_unlock();
  if (ret < 0)
    {
      irq_detach(NETC_IRQ);
      goto free_dma;
    }

  nxmutex_unlock(&g_initlock);
  return OK;

free_dma:

  /* Packet storage has not been published to DMA; release its MPU mapping. */

  imxrt_netc_dma_release(priv);
failed:
  priv->state = NETC_INITFAILED;
  nerr("NETC initialization failed: %d\n", ret);
  nxmutex_unlock(&g_initlock);
  return ret;
}

#endif /* CONFIG_IMXRT_NETC */
