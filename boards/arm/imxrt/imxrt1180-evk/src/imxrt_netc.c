/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_netc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "imxrt_clockconfig.h"
#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "imxrt_netc.h"
#include "hardware/imxrt_netc.h"
#include "hardware/imxrt_src.h"
#include "hardware/rt118x/imxrt118x_ocotp.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_ARCH_CORTEXM7)
#  error "EVK NETC requires Cortex-M7"
#endif
#if (CONFIG_IMXRT_NETC_PORTMASK & 12) && defined(CONFIG_IMXRT_SEMC)
#  error "ETH2/ETH3 pads conflict with SEMC; disable SEMC or these ports"
#endif

/* Verified against MCUXpresso evkmimxrt1180 netc/switch pin_mux.c,
 * hardware_init.c, board.h and the RTL8211F PHY helper.  The block-control
 * offsets/fields below are from MIMXRT1189_cm7.h, BLK_CTRL_WAKEUPMIX_Type.
 * MII_PROT encodings here differ from the NETC MAC PMIFR encodings.
 */

#define EVK_NETC_MISC       (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + 0x24)
#define EVK_NETC_LINK(n)    (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + 0x100 + 4 * (n))
#define EVK_NETC_MII_MASK   0x0fu
#define EVK_NETC_RMII       1u
#define EVK_NETC_RGMII      2u
#define EVK_NETC_REVMII     (1u << 31)
#define EVK_NETC_RMII_OUT   1u

/* The SDK leaves data-pad electrical settings at reset defaults.  Select
 * high drive and no pull explicitly (PULL=00 is forbidden on these pads).
 */

#define NETC_PIN(p) IOMUX_PIN(IOMUXC_PAD_##p, IOMUXC_PAD_PULL_NONE, 0)
#define NETC_REF(p) IOMUX_PIN(IOMUXC_PAD_##p, IOMUXC_PAD_PULL_NONE, \
                             IOMUXC_MUX_SION_ON)
#define NETC_RESET(p, ctl, port, pin) \
  IOMUX_GPIO(IOMUXC_PAD_##p, ctl, GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
             GPIO_PORT##port | GPIO_PIN##pin)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct evk_netc_pins_s
{
  const imxrt_pinset_t *pins;
  unsigned int count;
};

enum evk_netc_state_e
{
  EVK_NETC_COLD,
  EVK_NETC_READY,
  EVK_NETC_FAILED
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const imxrt_pinset_t g_port0_pins[] =
{
  NETC_PIN(GPIO_EMC_B2_05_NETC_PINMUX_ETH0_TXD00),
  NETC_PIN(GPIO_EMC_B2_06_NETC_PINMUX_ETH0_TXD01),
  NETC_PIN(GPIO_EMC_B2_07_NETC_PINMUX_ETH0_TX_EN),
  NETC_REF(GPIO_EMC_B2_08_NETC_PINMUX_ETH0_TX_CLK),
  NETC_PIN(GPIO_EMC_B2_09_NETC_PINMUX_ETH0_RXD00),
  NETC_PIN(GPIO_EMC_B2_10_NETC_PINMUX_ETH0_RXD01),
  NETC_PIN(GPIO_EMC_B2_11_NETC_PINMUX_ETH0_RX_DV),
  NETC_PIN(GPIO_EMC_B2_12_NETC_PINMUX_ETH0_RX_ER)
};

static const imxrt_pinset_t g_port1_pins[] =
{
  NETC_PIN(GPIO_B1_00_NETC_PINMUX_ETH1_TXD00),
  NETC_PIN(GPIO_B1_01_NETC_PINMUX_ETH1_TXD01),
  NETC_PIN(GPIO_B1_02_NETC_PINMUX_ETH1_TX_EN),
  NETC_PIN(GPIO_B1_03_NETC_PINMUX_ETH1_TX_CLK),
  NETC_PIN(GPIO_B1_04_NETC_PINMUX_ETH1_RXD00),
  NETC_PIN(GPIO_B1_05_NETC_PINMUX_ETH1_RXD01),
  NETC_PIN(GPIO_B1_06_NETC_PINMUX_ETH1_RX_DV),
  NETC_PIN(GPIO_B1_07_NETC_PINMUX_ETH1_TXD02),
  NETC_PIN(GPIO_B1_08_NETC_PINMUX_ETH1_TXD03),
  NETC_PIN(GPIO_B1_09_NETC_PINMUX_ETH1_RXD02),
  NETC_PIN(GPIO_B1_10_NETC_PINMUX_ETH1_RXD03),
  NETC_PIN(GPIO_B1_11_NETC_PINMUX_ETH1_RX_CLK)
};

static const imxrt_pinset_t g_port2_pins[] =
{
  NETC_PIN(GPIO_EMC_B1_13_NETC_PINMUX_ETH2_RX_DV),
  NETC_PIN(GPIO_EMC_B1_16_NETC_PINMUX_ETH2_RXD00),
  NETC_PIN(GPIO_EMC_B1_17_NETC_PINMUX_ETH2_RXD01),
  NETC_PIN(GPIO_EMC_B1_21_NETC_PINMUX_ETH2_RX_CLK),
  NETC_PIN(GPIO_EMC_B1_22_NETC_PINMUX_ETH2_RXD02),
  NETC_PIN(GPIO_EMC_B1_23_NETC_PINMUX_ETH2_RXD03),
  NETC_PIN(GPIO_EMC_B1_24_NETC_PINMUX_ETH2_TXD03),
  NETC_PIN(GPIO_EMC_B1_25_NETC_PINMUX_ETH2_TXD02),
  NETC_PIN(GPIO_EMC_B1_26_NETC_PINMUX_ETH2_TXD01),
  NETC_PIN(GPIO_EMC_B1_27_NETC_PINMUX_ETH2_TXD00),
  NETC_PIN(GPIO_EMC_B1_28_NETC_PINMUX_ETH2_TX_EN),
  NETC_PIN(GPIO_EMC_B1_29_NETC_PINMUX_ETH2_TX_CLK)
};

static const imxrt_pinset_t g_port3_pins[] =
{
  NETC_PIN(GPIO_EMC_B1_00_NETC_PINMUX_ETH3_TXD03),
  NETC_PIN(GPIO_EMC_B1_01_NETC_PINMUX_ETH3_TXD02),
  NETC_PIN(GPIO_EMC_B1_02_NETC_PINMUX_ETH3_RX_CLK),
  NETC_PIN(GPIO_EMC_B1_03_NETC_PINMUX_ETH3_RXD03),
  NETC_PIN(GPIO_EMC_B1_04_NETC_PINMUX_ETH3_RXD02),
  NETC_PIN(GPIO_EMC_B1_05_NETC_PINMUX_ETH3_TXD00),
  NETC_PIN(GPIO_EMC_B1_06_NETC_PINMUX_ETH3_TXD01),
  NETC_PIN(GPIO_EMC_B1_07_NETC_PINMUX_ETH3_TX_EN),
  NETC_PIN(GPIO_EMC_B1_08_NETC_PINMUX_ETH3_TX_CLK),
  NETC_PIN(GPIO_EMC_B1_09_NETC_PINMUX_ETH3_RXD00),
  NETC_PIN(GPIO_EMC_B1_10_NETC_PINMUX_ETH3_RXD01),
  NETC_PIN(GPIO_EMC_B1_11_NETC_PINMUX_ETH3_RX_DV)
};

static const imxrt_pinset_t g_reset_pins[] =
{
  NETC_RESET(GPIO_AD_25_GPIO4_IO25, 0, 4, 25),
  NETC_RESET(GPIO_B1_13_GPIO6_IO13,
             IOMUXC_PAD_ODE | IOMUXC_PAD_PULL_DOWN, 6, 13),
  NETC_RESET(GPIO_AD_28_GPIO4_IO28, 0, 4, 28),
  NETC_RESET(GPIO_B2_01_GPIO6_IO15,
             IOMUXC_PAD_ODE | IOMUXC_PAD_PULL_UP, 6, 15),
  NETC_RESET(GPIO_AD_13_GPIO4_IO13, 0, 4, 13) /* ETH4, not CPU port 4 */
};

static const struct evk_netc_pins_s g_port_pins[] =
{
  {
    g_port0_pins, nitems(g_port0_pins)
  },
  {
    g_port1_pins, nitems(g_port1_pins)
  },
  {
    g_port2_pins, nitems(g_port2_pins)
  },
  {
    g_port3_pins, nitems(g_port3_pins)
  }
};

static const struct imxrt_netc_config_s g_config =
{
  .portmask = CONFIG_IMXRT_NETC_PORTMASK,
  .ports =
  {
    { IMXRT_NETC_PEMDIO(0), 2, IMXRT_NETC_RMII,
      IMXRT_NETC_PHY_RTL8201, false, false },
    { IMXRT_NETC_PEMDIO(1), 5, IMXRT_NETC_RGMII,
      IMXRT_NETC_PHY_RTL8211, true, true },
    { IMXRT_NETC_PEMDIO(2), 4, IMXRT_NETC_RGMII,
      IMXRT_NETC_PHY_RTL8211, true, true },
    { IMXRT_NETC_PEMDIO(3), 7, IMXRT_NETC_RGMII,
      IMXRT_NETC_PHY_RTL8211, true, true }
  }
};

/* Never repeat the board-wide PHY reset, even after driver init fails. */

static mutex_t g_netc_lock = NXMUTEX_INITIALIZER;
static enum evk_netc_state_e g_netc_state;
static int g_netc_result;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imxrt_netc_setup(void)
{
  uint32_t frequency;
  uint32_t status;
  unsigned int port;
  unsigned int pin;
  int ret;

  /* IMXRT1180RM 27.3.4.2: all MIXs power up on POR (unless fused out).
   * M33 owns TRDC setup before releasing M7.  Do not seize its permissions,
   * power-cycle NETCMIX or interfere with a low-power transition here.
   */

  status = getreg32(IMXRT_SRC_SLICE_FUNC_STAT(IMXRT_NETC_MIX_SLICE_BASE));
  if ((status & (SRC_SLICE_FUNC_STAT_PSW_STAT |
                 SRC_SLICE_FUNC_STAT_RST_STAT |
                 SRC_SLICE_FUNC_STAT_ISO_STAT)) !=
      SRC_SLICE_FUNC_STAT_RST_STAT)
    {
      syslog(LOG_ERR, "NETC: M33 must power/release NETCMIX: %#lx\n",
             (unsigned long)status);
      return -EHOSTDOWN;
    }

  ret = imxrt_get_rootclock(CCM_CR_NETC, &frequency);
  if (ret < 0 || frequency != 240000000)
    {
      return ret < 0 ? ret : -EINVAL;
    }

  for (port = 0; port < 4; port++)
    {
      if ((g_config.portmask & (1 << port)) == 0)
        {
          continue;
        }

      ret = imxrt_get_rootclock(CCM_CR_MAC0 + port, &frequency);
      if (ret < 0 || frequency != (port == 0 ? 50000000 : 125000000))
        {
          return ret < 0 ? ret : -EINVAL;
        }
    }

  ret = imxrt_ccm_gate_on(CCM_LPCG_IOMUXC1, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_clockall_gpio4();
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_clockall_gpio6();
  if (ret < 0)
    {
      return ret;
    }

  /* Shared onboard management signals, SDK pad values 0x06/0x16. */

  ret = imxrt_config_gpio(IOMUX_PIN(IOMUXC_PAD_GPIO_AD_30_NETC_EMDC,
                                   IOMUXC_PAD_DSE | IOMUXC_PAD_PUE, 0));
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_config_gpio(IOMUX_PIN(IOMUXC_PAD_GPIO_AD_31_NETC_EMDIO,
                                   IOMUXC_PAD_DSE | IOMUXC_PAD_PUE |
                                   IOMUXC_PAD_ODE, 0));
  if (ret < 0)
    {
      return ret;
    }

  for (port = 0; port < 4; port++)
    {
      if ((g_config.portmask & (1 << port)) == 0)
        {
          continue;
        }

      for (pin = 0; pin < g_port_pins[port].count; pin++)
        {
          ret = imxrt_config_gpio(g_port_pins[port].pins[pin]);
          if (ret < 0)
            {
              return ret;
            }
        }

      modifyreg32(EVK_NETC_LINK(port), EVK_NETC_MII_MASK | EVK_NETC_REVMII,
                  port == 0 ? EVK_NETC_RMII : EVK_NETC_RGMII);
    }

  if ((g_config.portmask & 1) != 0)
    {
      modifyreg32(EVK_NETC_MISC, 0, EVK_NETC_RMII_OUT);
    }

  /* SDK: reset every onboard PHY to avoid unstable unused PHYs.  Preserve
   * power-on settling (150 ms), assert for 10 ms, then wait 150 ms for the
   * slowest RTL8201.  ETH4 is not otherwise enabled by this driver.
   */

  for (pin = 0; pin < nitems(g_reset_pins); pin++)
    {
      ret = imxrt_config_gpio(g_reset_pins[pin]);
      if (ret < 0)
        {
          return ret;
        }
    }

  up_mdelay(150);
  for (pin = 0; pin < nitems(g_reset_pins); pin++)
    {
      imxrt_gpio_write(g_reset_pins[pin], false);
    }

  up_mdelay(10);
  for (pin = 0; pin < nitems(g_reset_pins); pin++)
    {
      imxrt_gpio_write(g_reset_pins[pin], true);
    }

  up_mdelay(150);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_boardinitialize(void)
{
  struct imxrt_netc_config_s config = g_config;
  uint32_t uidl;
  uint32_t uidml;
  int ret;

  ret = nxmutex_lock(&g_netc_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_netc_state == EVK_NETC_COLD)
    {
      ret = imxrt_netc_setup();
      if (ret == OK)
        {
          /* Same locally administered UID convention as imxrt_enet.c. */

          uidl = getreg32(IMXRT_OCOTP_UNIQUE_ID_MSB);
          uidml = getreg32(IMXRT_OCOTP_UNIQUE_ID_LSB);
          config.mac[0] = ((uidml >> 8) & 0xfe) | 0x02;
          config.mac[1] = uidml;
          config.mac[2] = uidl >> 24;
          config.mac[3] = uidl >> 16;
          config.mac[4] = uidl >> 8;
          config.mac[5] = uidl;
          ret = imxrt_netc_initialize(&config);
        }

      g_netc_result = ret;
      g_netc_state = ret == OK ? EVK_NETC_READY : EVK_NETC_FAILED;
    }

  ret = g_netc_result;
  nxmutex_unlock(&g_netc_lock);
  return ret;
}

#ifndef CONFIG_NETDEV_LATEINIT
void arm_netinitialize(void)
{
  int ret = imxrt_netc_boardinitialize();

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: NETC initialization failed: %d\n", ret);
    }
}
#endif
