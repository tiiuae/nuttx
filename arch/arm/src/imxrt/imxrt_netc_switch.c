/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc_switch.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <endian.h>
#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "imxrt_netc_priv.h"
#include "hardware/imxrt_netc.h"
#include "hardware/imxrt_netc_switch.h"
#ifdef CONFIG_ARM_MPU
#  include "mpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARM_MPU)
#  error "NETC switch DMA requires a non-cacheable MPU region"
#endif
#ifdef CONFIG_NET_PROMISCUOUS
#  error "NETC bridge promiscuous capture needs a switch mirroring path"
#endif

#define SWREG(r)                  (NETC_SW_BASE + (r))
#define PORTREG(p, r)             (NETC_SW_PORT(p) + (r))
#define ETHREG(p, r)              (NETC_SW_ETH(p) + (r))
#define SWITCH_DMA_SIZE           512
#define SWITCH_RING_SIZE          8
#define SWITCH_WAIT_US            10000
#define SWITCH_MODE_LINK          0x80
#define SWITCH_MODE_DRAIN         0x40
#define PHY_RESET_WAIT_US         500000
#define PHY_PAGE                  31
#define PHY_BMCR                  0
#define PHY_BMSR                  1
#define PHY_ID1                   2
#define PHY_ID2                   3
#define PHY_ADVERTISE             4
#define PHY_PARTNER               5
#define PHY_GB_CONTROL            9
#define PHY_RESET                 0x8000
#define PHY_AUTONEG_RESTART       0x1200
#define PHY_LINK_COMPLETE         0x0024
#define PHY_ADV_100_FULL          0x0100
#define PHY_ADV_100_HALF          0x0080
#define PHY_ADV_10_FULL           0x0040
#define PHY_ADV_10_HALF           0x0020
#define PHY_ADV_SELECTOR          0x0001
#define PHY_ADV_GB_FULL           0x0200
#define RTL8201_RMII_PAGE         7
#define RTL8201_RMII_REG          16
#define RTL8201_RXDV              0x0004
#define RTL8211_STATUS            26
#define RTL8211_LINK              0x0004
#define RTL8211_FULL              0x0008
#define RTL8211_DELAY_PAGE        0x0d08
#define RTL8211_TX_DELAY_REG      17
#define RTL8211_RX_DELAY_REG      21
#define RTL8211_TX_DELAY          0x0100
#define RTL8211_RX_DELAY          0x0008

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Only one owner may initialize this switch.  All public calls are task
 * context calls, serialized here.  Runtime lock order is net_lock, switch
 * lock, core MDIO lock; net_lock also excludes raw PHY ioctls during paging.
 * The core must keep calling switch_link while the interface is down.
 */

enum imxrt_switch_state_e
{
  SWITCH_UNUSED,
  SWITCH_INITIALIZING,
  SWITCH_READY,
  SWITCH_CPU_UP,
  SWITCH_FAILED
};

struct imxrt_switch_bd_s
{
  uint32_t word[8];
};

struct imxrt_switch_dma_s
{
  struct imxrt_switch_bd_s bd[SWITCH_RING_SIZE];
  uint32_t data[32];
};

union imxrt_switch_storage_u
{
  struct imxrt_switch_dma_s dma;
  uint8_t padding[SWITCH_DMA_SIZE];
};

static_assert(sizeof(union imxrt_switch_storage_u) == SWITCH_DMA_SIZE,
              "NTMP MPU region size");
static_assert(sizeof(struct imxrt_switch_bd_s) == 32, "NTMP BD size");
static_assert(offsetof(struct imxrt_switch_dma_s, data) == 256,
              "NTMP data alignment");
static_assert(sizeof(struct imxrt_switch_dma_s) <= SWITCH_DMA_SIZE,
              "NTMP MPU allocation size");

/****************************************************************************
 * Private Data
 ****************************************************************************/

static union imxrt_switch_storage_u g_switch_storage
  locate_data(".netc_dma.command") aligned_data(SWITCH_DMA_SIZE);
static mutex_t g_switch_lock = NXMUTEX_INITIALIZER;
static enum imxrt_switch_state_e g_switch_state;
static struct imxrt_netc_config_s g_switch_config;
static struct imxrt_switch_dma_s *g_switch_dma;
static uint8_t g_switch_producer;
static uint8_t g_switch_mode[4];
static uint32_t g_switch_cpueid = UINT32_MAX;
static uint8_t g_switch_mac[6];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imxrt_switch_wait(uintptr_t reg, uint32_t mask, uint32_t value,
                             unsigned int timeout)
{
  unsigned int elapsed;

  for (elapsed = 0; elapsed < timeout; elapsed += 10)
    {
      if ((getreg32(reg) & mask) == value)
        {
          return OK;
        }

      up_udelay(10);
    }

  return -ETIMEDOUT;
}

static int imxrt_switch_dma_initialize(void)
{
  uintptr_t base = (uintptr_t)&g_switch_storage;
#ifdef CONFIG_ARMV7M_DCACHE
  irqstate_t flags;
#endif

  if ((base & (SWITCH_DMA_SIZE - 1)) != 0 ||
      !((base >= IMXRT_OCRAM_BASE &&
         base + SWITCH_DMA_SIZE <= IMXRT_OCRAM1_BASE + 512 * 1024) ||
        (base >= IMXRT_OCRAM2_BASE &&
         base + SWITCH_DMA_SIZE <= IMXRT_OCRAM2_BASE + 256 * 1024)))
    {
      return -EFAULT;
    }

#ifdef CONFIG_ARMV7M_DCACHE
  up_clean_dcache(base, base + SWITCH_DMA_SIZE);
  up_invalidate_dcache(base, base + SWITCH_DMA_SIZE);
  flags = enter_critical_section();
  mpu_configure_region(base, SWITCH_DMA_SIZE,
                       MPU_RASR_TEX_NOR | MPU_RASR_S |
                       MPU_RASR_AP_RWNO | MPU_RASR_XN);
  leave_critical_section(flags);
  UP_DSB();
  UP_ISB();
#endif

  /* NOLOAD storage is initialized before its command ring is enabled. */

  memset(&g_switch_storage, 0, sizeof(g_switch_storage));
  g_switch_dma = &g_switch_storage.dma;
  return OK;
}

static int imxrt_switch_command(uint32_t operation, bool exact,
                                unsigned int request)
{
  struct imxrt_switch_bd_s *bd;
  uint32_t response;
  int ret;

  if (g_switch_state == SWITCH_FAILED)
    {
      return -EIO;
    }

  bd = &g_switch_dma->bd[g_switch_producer];
  memset(bd, 0, sizeof(*bd));
  bd->word[0] = htole32((uintptr_t)g_switch_dma->data);
  bd->word[2] = htole32((request << 20) | NETC_NTMP_FDB_RESPONSE);
  bd->word[3] = htole32(operation | (exact ? NETC_NTMP_EXACT : 0) |
                        (NETC_NTMP_FDB << 16) | NETC_NTMP_VERSION2);
  bd->word[7] = htole32(NETC_NTMP_NPF);
  g_switch_producer = (g_switch_producer + 1) % SWITCH_RING_SIZE;
  UP_DSB();
  putreg32(g_switch_producer, SWREG(NETC_SW_CBDRPIR));
  ret = imxrt_switch_wait(SWREG(NETC_SW_CBDRCIR), 0xffff,
                          g_switch_producer, SWITCH_WAIT_US);
  UP_DSB();
  if (ret < 0)
    {
      /* A late completion may still overwrite both BD and data.  Never
       * submit again or free/reinitialize this allocation without PF2 FLR.
       * We deliberately do not reset the bridge or external forwarding.
       */

      g_switch_state = SWITCH_FAILED;
      nerr("NETC switch NTMP timeout; transport quarantined\n");
      return ret;
    }

  response = le32toh(bd->word[3]);
  if ((response & NETC_NTMP_READY) == 0)
    {
      g_switch_state = SWITCH_FAILED;
      return -EIO;
    }

  if ((response & NETC_NTMP_ERROR_MASK) != 0)
    {
      nerr("NETC switch NTMP error: %03lx\n",
           (unsigned long)((response & NETC_NTMP_ERROR_MASK) >> 16));
      return -EIO;
    }

  return OK;
}

static int imxrt_switch_fdb(const uint8_t *mac)
{
  uint32_t *data = g_switch_dma->data;
  int ret;

  if (g_switch_cpueid != UINT32_MAX &&
      memcmp(mac, g_switch_mac, sizeof(g_switch_mac)) != 0)
    {
      memset(data, 0, sizeof(g_switch_dma->data));
      data[1] = htole32(g_switch_cpueid);
      ret = imxrt_switch_command(NETC_NTMP_DELETE, false,
                                 NETC_NTMP_FDB_REQUEST);
      if (ret < 0)
        {
          return ret;
        }

      g_switch_cpueid = UINT32_MAX;
    }

  /* FID 0, static entry, no egress treatment or cut-through override.
   * Add/query/update also replaces any dynamically learned CPU entry.
   */

  memset(data, 0, sizeof(g_switch_dma->data));
  data[0] = htole32(NETC_NTMP_FDB_CFGEU | NETC_NTMP_QUERY_ENTRY_ID);
  memcpy(&data[1], mac, 6);
  data[NETC_NTMP_FDB_CFG_WORD] = htole32(NETC_SW_CPU);
  data[NETC_NTMP_FDB_CFG_WORD + 2] = htole32(UINT32_MAX);
  ret = imxrt_switch_command(NETC_NTMP_ADDQUERYUPDATE, true,
                             NETC_NTMP_FDB_REQUEST);
  if (ret == OK)
    {
      g_switch_cpueid = le32toh(data[1]);
      if (g_switch_cpueid == UINT32_MAX)
        {
          return -EIO;
        }

      memcpy(g_switch_mac, mac, sizeof(g_switch_mac));
    }

  return ret;
}

static int imxrt_switch_phy_read(const struct imxrt_netc_port_s *port,
                                 uint8_t reg, uint16_t *value)
{
  return imxrt_netc_mdio_read(port->mdio, port->phyaddr, reg, value);
}

static int imxrt_switch_phy_write(const struct imxrt_netc_port_s *port,
                                  uint8_t reg, uint16_t value)
{
  return imxrt_netc_mdio_write(port->mdio, port->phyaddr, reg, value);
}

static int imxrt_switch_phy_modify(const struct imxrt_netc_port_s *port,
                                   uint8_t reg, uint16_t mask, bool set)
{
  uint16_t value;
  int ret = imxrt_switch_phy_read(port, reg, &value);

  if (ret == OK)
    {
      value = (value & ~mask) | (set ? mask : 0);
      ret = imxrt_switch_phy_write(port, reg, value);
    }

  return ret;
}

static int imxrt_switch_phy_init(const struct imxrt_netc_port_s *port)
{
  uint16_t value;
  unsigned int elapsed;
  int restore;
  int ret;

  ret = imxrt_switch_phy_write(port, PHY_PAGE, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_switch_phy_read(port, PHY_ID1, &value);
  if (ret < 0 || value != 0x001c)
    {
      return ret < 0 ? ret : -ENODEV;
    }

  ret = imxrt_switch_phy_read(port, PHY_ID2, &value);
  if (ret < 0)
    {
      return ret;
    }

  /* These delay/status registers are RTL8211F, not older RTL8211 variants. */

  if ((port->phy == IMXRT_NETC_PHY_RTL8211 && value != 0xc916) ||
      (port->phy == IMXRT_NETC_PHY_RTL8201 && (value & 0xfff0) != 0xc810))
    {
      return -ENODEV;
    }

  ret = imxrt_switch_phy_write(port, PHY_BMCR, PHY_RESET);
  if (ret < 0)
    {
      return ret;
    }

  for (elapsed = 0; elapsed < PHY_RESET_WAIT_US; elapsed += 1000)
    {
      ret = imxrt_switch_phy_read(port, PHY_BMCR, &value);
      if (ret < 0)
        {
          return ret;
        }

      if ((value & PHY_RESET) == 0)
        {
          break;
        }

      up_udelay(1000);
    }

  if ((value & PHY_RESET) != 0)
    {
      return -ETIMEDOUT;
    }

  if (port->phy == IMXRT_NETC_PHY_RTL8211)
    {
      ret = imxrt_switch_phy_write(port, PHY_PAGE, RTL8211_DELAY_PAGE);
      if (ret == OK)
        {
          ret = imxrt_switch_phy_modify(port, RTL8211_TX_DELAY_REG,
                                        RTL8211_TX_DELAY, port->txdelay);
        }

      if (ret == OK)
        {
          ret = imxrt_switch_phy_modify(port, RTL8211_RX_DELAY_REG,
                                        RTL8211_RX_DELAY, port->rxdelay);
        }

      restore = imxrt_switch_phy_write(port, PHY_PAGE, 0);
      if (ret < 0 || restore < 0)
        {
          return ret < 0 ? ret : restore;
        }

      /* Full duplex only for RGMII (ERR051936); never advertise gigabit
       * when the board selects MII/RMII.  Pause/EEE are not enabled.
       */

      value = PHY_ADV_SELECTOR | PHY_ADV_100_FULL | PHY_ADV_10_FULL;
      if (port->mode != IMXRT_NETC_RGMII)
        {
          value |= PHY_ADV_100_HALF | PHY_ADV_10_HALF;
        }

      ret = imxrt_switch_phy_write(port, PHY_ADVERTISE, value);
      if (ret == OK)
        {
          ret = imxrt_switch_phy_write(port, PHY_GB_CONTROL,
                     port->mode == IMXRT_NETC_RGMII ? PHY_ADV_GB_FULL : 0);
        }
    }
  else
    {
#ifdef CONFIG_IMXRT_NETC_RTL8201_RXDV_ON_CRS_DV
      if (port->mode == IMXRT_NETC_RMII)
        {
          /* Select RXDV on CRS_DV for boards requiring this wiring. */

          ret = imxrt_switch_phy_write(port, PHY_PAGE, RTL8201_RMII_PAGE);
          if (ret == OK)
            {
              ret = imxrt_switch_phy_modify(port, RTL8201_RMII_REG,
                                            RTL8201_RXDV, true);
            }

          restore = imxrt_switch_phy_write(port, PHY_PAGE, 0);
          if (ret < 0 || restore < 0)
            {
              return ret < 0 ? ret : restore;
            }
        }
#endif

      /* ERR051651: RTL8201FI preamble is incompatible at 10 Mb/s. */

      ret = imxrt_switch_phy_write(port, PHY_ADVERTISE,
                  PHY_ADV_SELECTOR | PHY_ADV_100_FULL | PHY_ADV_100_HALF);
    }

  if (ret == OK)
    {
      ret = imxrt_switch_phy_write(port, PHY_BMCR, PHY_AUTONEG_RESTART);
    }

  return ret;
}

static int imxrt_switch_phy_status(const struct imxrt_netc_port_s *port,
                                   uint8_t *mode)
{
  uint16_t status;
  uint16_t partner;
  uint16_t advertise;
  int ret;

  *mode = 0;
  ret = imxrt_switch_phy_write(port, PHY_PAGE, 0);
  if (ret == OK)
    {
      ret = imxrt_switch_phy_read(port, PHY_BMSR, &status);
    }

  if (ret == OK)
    {
      ret = imxrt_switch_phy_read(port, PHY_BMSR, &status);
    }

  if (ret < 0 || (status & PHY_LINK_COMPLETE) != PHY_LINK_COMPLETE)
    {
      return ret;
    }

  if (port->phy == IMXRT_NETC_PHY_RTL8211)
    {
      ret = imxrt_switch_phy_read(port, RTL8211_STATUS, &status);
      if (ret == OK && (status & RTL8211_LINK) != 0)
        {
          /* mode: bit 7 link, bit 2 full duplex, bits 1:0 speed. */

          *mode = 0x80 | ((status >> 4) & 3) |
                  ((status & RTL8211_FULL) != 0 ? 4 : 0);
          if ((*mode & 3) == 3 ||
              ((*mode & 3) == 2 && port->mode != IMXRT_NETC_RGMII) ||
              (port->mode == IMXRT_NETC_RGMII && (*mode & 4) == 0))
            {
              *mode = 0;
              return -EPROTONOSUPPORT;
            }
        }
    }
  else
    {
      ret = imxrt_switch_phy_read(port, PHY_ADVERTISE, &advertise);
      if (ret == OK)
        {
          ret = imxrt_switch_phy_read(port, PHY_PARTNER, &partner);
        }

      if (ret == OK)
        {
          partner &= advertise;
          if ((partner & PHY_ADV_100_FULL) != 0)
            {
              *mode = 0x85;
            }
          else if ((partner & PHY_ADV_100_HALF) != 0)
            {
              *mode = 0x81;
            }
        }
    }

  return ret;
}

static int imxrt_switch_port_mode(unsigned int port, uint8_t mode)
{
  const struct imxrt_netc_port_s *cfg = &g_switch_config.ports[port];
  uintptr_t eth = NETC_SW_ETH(port);
  unsigned int speed = mode & 3;
  uint32_t reg;

  /* Stop RX at the MAC and TX at the bridge before changing clocks.
   * Check draining once per poll, never spin waiting for link/FIFO state.
   */

  modifyreg32(eth + NETC_SW_MAC_CMD, NETC_SW_MAC_RX, 0);
  modifyreg32(PORTREG(port, NETC_SW_POR), 0, NETC_SW_DISABLE);
  UP_DSB();
  reg = getreg32(eth + NETC_SW_MAC_IEVENT);
  if ((reg & (NETC_SW_MAC_RXEMPTY | NETC_SW_MAC_TXEMPTY)) !=
      (NETC_SW_MAC_RXEMPTY | NETC_SW_MAC_TXEMPTY) ||
      (getreg32(PORTREG(port, NETC_SW_PSR)) & NETC_SW_BUSY) != 0)
    {
      g_switch_mode[port] = SWITCH_MODE_DRAIN;
      return -EAGAIN;
    }

  modifyreg32(eth + NETC_SW_MAC_CMD, NETC_SW_MAC_TX, 0);
  g_switch_mode[port] = 0;
  if (mode == 0)
    {
      return OK;
    }

  reg = getreg32(eth + NETC_SW_MAC_IFMODE) & ~NETC_SW_IF_MASK;
  reg |= cfg->mode;
  if (cfg->mode == IMXRT_NETC_RGMII)
    {
      reg |= NETC_SW_IF_SSP(speed == 2 ? 2 : speed == 1 ? 0 : 1);
    }
  else if (speed == 0)
    {
      reg |= NETC_SW_IF_10M;
    }

  if ((mode & 4) == 0)
    {
      reg |= NETC_SW_IF_HALF;
    }

  putreg32(reg, eth + NETC_SW_MAC_IFMODE);
  putreg32(reg, eth + NETC_SW_MAC1_OFFSET + NETC_SW_MAC_IFMODE);

  /* PCR speed is 10 Mb/s * (PSPEED + 1), not an Mb/s literal. */

  modifyreg32(PORTREG(port, NETC_SW_PCR), NETC_SW_PSPEED_MASK,
               NETC_SW_PSPEED(speed == 2 ? 99 : speed == 1 ? 9 : 0));
  putreg32(NETC_SW_MAC_TX | NETC_SW_MAC_RX | NETC_SW_MAC_PAD,
           eth + NETC_SW_MAC_CMD);
  UP_DSB();
  modifyreg32(PORTREG(port, NETC_SW_POR), NETC_SW_DISABLE, 0);
  g_switch_mode[port] = mode;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_switch_initialize(const struct imxrt_netc_config_s *config)
{
  unsigned int port;
  unsigned int tc;
  int ret;

  if (config == NULL || config->portmask == 0 ||
      (config->portmask & ~15) != 0 || (config->mac[0] & 1) != 0)
    {
      return -EINVAL;
    }

  for (port = 0; port < 4; port++)
    {
      const struct imxrt_netc_port_s *p = &config->ports[port];

      if ((config->portmask & (1u << port)) != 0 &&
          (p->mdio == 0 || p->phyaddr > 31 ||
           (p->mode != IMXRT_NETC_MII && p->mode != IMXRT_NETC_RMII &&
            p->mode != IMXRT_NETC_RGMII) ||
           (p->phy != IMXRT_NETC_PHY_RTL8201 &&
            p->phy != IMXRT_NETC_PHY_RTL8211) ||
           (p->phy == IMXRT_NETC_PHY_RTL8201 &&
            p->mode == IMXRT_NETC_RGMII)))
        {
          return -EINVAL;
        }
    }

  ret = nxmutex_lock(&g_switch_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_switch_state != SWITCH_UNUSED)
    {
      nxmutex_unlock(&g_switch_lock);
      return -EBUSY;
    }

  /* Infrastructure owns ECAM/IERB and resets.  Never reset PF2 here. */

  if ((getreg16(IMXRT_NETC_PF2_BASE + NETC_PCI_CMD) &
       NETC_PCI_CMD_ENABLE) != NETC_PCI_CMD_ENABLE ||
      (getreg32(SWREG(NETC_SW_CBDRMR)) & NETC_SW_EN) != 0)
    {
      nxmutex_unlock(&g_switch_lock);
      return -EBUSY;
    }

  g_switch_config = *config;
  for (port = 0; port < 5; port++)
    {
      modifyreg32(PORTREG(port, NETC_SW_POR), 0, NETC_SW_DISABLE);
    }

  ret = imxrt_switch_dma_initialize();
  if (ret < 0)
    {
      nxmutex_unlock(&g_switch_lock);
      return ret;
    }

  g_switch_state = SWITCH_INITIALIZING;
  for (port = 0; port < 4; port++)
    {
      if ((config->portmask & (1u << port)) != 0)
        {
          ret = imxrt_switch_phy_init(&config->ports[port]);
          if (ret < 0)
            {
              nxmutex_unlock(&g_switch_lock);
              return ret;
            }
        }
    }

  /* VLAN-unaware access bridge: all traffic uses default VID 1/FID 0.
   * Preserve on-wire tags, prune ingress port, learn/move in hardware.
   * Flood unknown unicast and all multicast/broadcast to members; ENETC
   * filters CPU unicast.  No IP multicast filter or management redirection.
   * This is NOT spanning tree; the deployment must be loop-free.
   */

  putreg32(config->portmask, SWREG(NETC_SW_VFHTDECR0));
  putreg32(NETC_SW_FIXED_FID, SWREG(NETC_SW_VFHTDECR1));
  putreg32(NETC_SW_LEARN_FLOOD, SWREG(NETC_SW_VFHTDECR2));
  putreg32(0xffff, SWREG(NETC_SW_FDBHTMCR));
  for (port = 0; port < 5; port++)
    {
      if (port != 4 && (config->portmask & (1u << port)) == 0)
        {
          putreg32(0, PORTREG(port, NETC_SW_BPSTGSR));
          continue;
        }

      putreg32(NETC_SW_STG_FORWARD, PORTREG(port, NETC_SW_BPSTGSR));
      putreg32(1 | NETC_SW_VLAN_UNAWARE | NETC_SW_ACCEPT_TAGS,
               PORTREG(port, NETC_SW_BPDVR));
      putreg32(0xffff, PORTREG(port, NETC_SW_BPCR));

      /* ERR051649: VLAN-to-QoS profile 0.  ERR051444: do not alter
       * IPV2ICMPMR0; retain reset single-priority congestion management.
       */

      putreg32(NETC_SW_STANDARD_TPIDS, PORTREG(port, NETC_SW_PTAR));
      putreg32(NETC_SW_NULL_STREAM, PORTREG(port, NETC_SW_PISIDCR));
      putreg32(0, PORTREG(port, NETC_SW_PIPFCR));
      putreg32(0, PORTREG(port, NETC_SW_PQOSMR));
      putreg32(20, PORTREG(port, NETC_SW_PRXSDUOR));
      putreg32(20, PORTREG(port, NETC_SW_PTXSDUOR));
      putreg32(0, PORTREG(port, NETC_SW_PTGSCR));
      putreg32(0xff, PORTREG(port, NETC_SW_PDGSR));
      putreg32(0, PORTREG(port, NETC_SW_PIPV2QMR0));
      modifyreg32(PORTREG(port, NETC_SW_PCR), NETC_SW_PSPEED_MASK,
                   NETC_SW_PSPEED(99));
      for (tc = 0; tc < 8; tc++)
        {
          putreg32(NETC_SW_MAXSDU_DISABLE,
                   PORTREG(port, NETC_SW_PTCTMSDUR(tc)));
        }

      if (port < 4)
        {
          putreg32(NETC_SW_STORE_FORWARD, PORTREG(port, NETC_SW_PCTFCR));
          putreg32(1522, ETHREG(port, NETC_SW_MAC_MAXFRM));
          putreg32(64, ETHREG(port, NETC_SW_MAC_MINFRM));
          putreg32(NETC_SW_MAC_PAD, ETHREG(port, NETC_SW_MAC_CMD));
          putreg32(0, ETHREG(port, NETC_SW_MAC1_OFFSET + NETC_SW_MAC_CMD));

          /* ERR051202: wait for 32 bytes in both transmit MAC FIFOs.
           * NXP SWT_Init uses these documented implementation registers.
           */

          putreg32(0x00010104, ETHREG(port, 0x1020));
          putreg32(0x00010104, ETHREG(port, 0x1420));
        }
    }

  putreg32((uintptr_t)g_switch_dma->bd, SWREG(NETC_SW_CBDRBAR0));
  putreg32(0, SWREG(NETC_SW_CBDRBAR1));
  putreg32(0, SWREG(NETC_SW_CBDRPIR));
  putreg32(0, SWREG(NETC_SW_CBDRCIR));
  putreg32(SWITCH_RING_SIZE, SWREG(NETC_SW_CBDRLENR));
  UP_DSB();
  putreg32(NETC_SW_EN, SWREG(NETC_SW_CBDRMR));
  ret = imxrt_switch_fdb(config->mac);
  g_switch_state = ret == OK ? SWITCH_READY : SWITCH_FAILED;
  nxmutex_unlock(&g_switch_lock);
  return ret;
}

int imxrt_netc_switch_cpu(bool enable, const uint8_t *mac)
{
  int ret = nxmutex_lock(&g_switch_lock);

  if (ret < 0)
    {
      return ret;
    }

  if (g_switch_state == SWITCH_UNUSED)
    {
      nxmutex_unlock(&g_switch_lock);
      return -ENODEV;
    }

  /* ERR051246: this must precede ANY PF4 FLR, even after NTMP failure.
   * Membership removal suppresses flooding; POR gates static FDB traffic.
   * Neither operation changes external members or their learned entries.
   */

  modifyreg32(PORTREG(4, NETC_SW_POR), 0, NETC_SW_DISABLE);
  modifyreg32(SWREG(NETC_SW_VFHTDECR0), NETC_SW_CPU, 0);
  UP_DSB();
  if ((getreg32(PORTREG(4, NETC_SW_POR)) & NETC_SW_DISABLE) !=
      NETC_SW_DISABLE)
    {
      nxmutex_unlock(&g_switch_lock);
      return -EIO;
    }

  if (g_switch_state == SWITCH_CPU_UP)
    {
      g_switch_state = SWITCH_READY;
    }

  if (!enable)
    {
      nxmutex_unlock(&g_switch_lock);
      return OK;
    }

  if (g_switch_state == SWITCH_FAILED ||
      g_switch_state == SWITCH_INITIALIZING ||
      mac == NULL || (mac[0] & 1) != 0)
    {
      ret = mac == NULL || (mac[0] & 1) != 0 ? -EINVAL : -EIO;
    }
  else
    {
      ret = imxrt_switch_fdb(mac);
      if (ret == OK)
        {
          modifyreg32(SWREG(NETC_SW_VFHTDECR0), 0, NETC_SW_CPU);
          UP_DSB();
          modifyreg32(PORTREG(4, NETC_SW_POR), NETC_SW_DISABLE, 0);
          g_switch_state = SWITCH_CPU_UP;
        }
    }

  nxmutex_unlock(&g_switch_lock);
  return ret;
}

int imxrt_netc_switch_link(uint8_t *linkmask)
{
  unsigned int port;
  uint8_t mode;
  int result = OK;
  int ret;

  if (linkmask == NULL)
    {
      return -EINVAL;
    }

  *linkmask = 0;
  ret = nxmutex_lock(&g_switch_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_switch_state == SWITCH_UNUSED ||
      g_switch_state == SWITCH_INITIALIZING || g_switch_dma == NULL)
    {
      nxmutex_unlock(&g_switch_lock);
      return -ENODEV;
    }

  for (port = 0; port < 4; port++)
    {
      if ((g_switch_config.portmask & (1u << port)) == 0)
        {
          continue;
        }

      /* No reset/autoneg wait in the polling path.  A failed MDIO read
       * closes only the affected port, then retries on the next poll.
       */

      ret = imxrt_switch_phy_status(&g_switch_config.ports[port], &mode);
      if (ret < 0)
        {
          result = ret;
          mode = 0;
        }

      if (mode != g_switch_mode[port])
        {
          ret = imxrt_switch_port_mode(port, mode);
          if (ret < 0 && ret != -EAGAIN)
            {
              result = ret;
            }
        }

      if ((g_switch_mode[port] & SWITCH_MODE_LINK) != 0)
        {
          *linkmask |= 1u << port;
        }
    }

  nxmutex_unlock(&g_switch_lock);
  return result;
}
