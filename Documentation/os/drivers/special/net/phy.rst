==================
PHY Device Drivers
==================

These drivers use the :doc:`PHYLIB framework </os/networking/phylib>`.

Realtek RTL8201F and RTL8211F
=============================

``CONFIG_NET_PHY_REALTEK_RTL8201F`` enables the Clause 22 MII/RMII driver;
``CONFIG_NET_PHY_REALTEK_RTL8211F`` enables the RTL8211F-VD and RTL8211F/FDI
RGMII driver. Register them with ``rtl8201f_register()`` or
``rtl8211f_register()`` from ``<nuttx/net/realtek_phy.h>`` before creating
PHY devices. Repeated registration is safe.

Set ``phydev->interface`` before ``phy_connect()``. For RTL8201F, select
``PHY_INTERFACE_RMII`` when the reference clock is supplied to the PHY,
or ``PHY_INTERFACE_RMII_PHY_CLK_OUT`` when the PHY supplies the clock.
The interface mode determines clock direction in both the MAC and PHY
drivers; no separate clock flag or driver-private configuration is needed.
The RTL8201F driver preserves RMII timing and oscillator settings.
RTL8211F programs both delay
bits according to ``PHY_INTERFACE_RGMII``, ``RGMII_ID``, ``RGMII_TXID`` or
``RGMII_RXID`` (no internal delay, both, TX only or RX only). Both drivers
use generic Clause 22 capability discovery, auto-negotiation, negotiated
link-status resolution and power control. Register-page operations are
serialized by the PHYLIB paged-access helpers. These drivers use polling
and auto-negotiation; forced link modes and PHY interrupts are not
implemented.

TI DP83867E
===========

``CONFIG_NET_PHY_TI_DP83867`` enables the DP83867E RGMII driver. Call
``dp83867_register()`` from ``<nuttx/net/ti_phy.h>`` before
``phy_device_create()``, then set ``phydev->interface`` before
``phy_connect()``.

Select ``PHY_INTERFACE_RGMII`` for no internal delay, ``RGMII_ID`` for
both delays, ``RGMII_TXID`` for TX delay only, or ``RGMII_RXID`` for RX
delay only. ``CONFIG_NET_PHY_TI_DP83867_RX_DELAY`` and
``CONFIG_NET_PHY_TI_DP83867_TX_DELAY`` select delay codes from 0 to 15:
each delay is ``(code + 1) * 250 ps``. Both default to code 7 (2 ns).
Choose these values to match the board timing budget, accounting for
delays already provided by the MAC and PCB.

The driver disables SGMII, enables RGMII, and clears the internal test
mode associated with RX_CTRL strap modes 1/2. Other strap settings,
including clock output, impedance and port mirroring, are preserved.
Extended-register transactions use indirect Clause 22 access with the
MDIO bus locked across address selection and read-modify-write; native
Clause 45 support is not required. Generic PHYLIB operations provide
capability discovery, auto-negotiation, link status and power control.
Link monitoring uses polling; SGMII and PHY interrupts are not implemented.
