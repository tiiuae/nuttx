===========
NXP i.MX RT
===========

The i.MX RT series of chips from NXP Semiconductors is based around an ARM Cortex-M7 core running
at 500 MHz, 600 MHz or 1 GHz based on particular MCUs

Supported MCUs
==============

The following list includes MCUs from i.MX RT series and indicates whether they are supported in NuttX

======  =======  ===============  =================
MCU     Support  Core             Frequency
======  =======  ===============  =================
RT500   No       Cortex-M33       200 MHz
RT600   No       Cortex-M33       300 MHz
RT1010  No       Cortex-M7        500 MHz
RT1015  No       Cortex-M7        500 MHz
RT1020  Yes      Cortex-M7        500 MHz
RT1024  No       Cortex-M7        500 MHz
RT1050  Yes      Cortex-M7        600 MHz
RT1060  Yes      Cortex-M7        600 MHz
RT1064  Yes      Cortex-M7        600 MHz
RT1170  No       Cortex-M7 + M4   1 GHz + 400 MHz
RT1180  Yes      Cortex-M7 + M33  800 Mhz + 300 MHz
======  =======  ===============  =================

i.MX RT118x
===========

The i.MX RT118x family is supported on both the Cortex-M33 and Cortex-M7
cores. The Cortex-M33 is the boot core; it can run NuttX directly, or it
can run a small bootloader that releases the Cortex-M7 to run a separate
NuttX image.

The current clock configuration runs the Cortex-M7 at 798 MHz and the
Cortex-M33 at 266 MHz due to ARM_PLL and divider setting limitations.

RT118x uses the generation 3 clock configuration driver for the clock
roots found on this family. It also uses the generation 3 IOMUXC driver
for the RT118x pinmux layout and GPIO mode selection.

RT118x uses the RGPIO driver and the matching RGPIO interrupt driver
instead of the legacy i.MX RT GPIO block. The version 2 eDMA driver is
selected for the newer RT118x eDMA IP.

The EdgeLock Enclave driver provides the messaging path used for RT118x
ELE firmware, and the TRDC driver configures access to RT118x memory and
peripheral resources. The Cortex-M7 release helper is used by Cortex-M33
firmware to set the Cortex-M7 boot vector and start the M7 core.

NETC architecture driver
------------------------

``CONFIG_IMXRT_NETC`` provides a NETDEV lower-half Ethernet driver for the
RT118x NETC. It registers one interface through ENETC1 station interface 0,
connected to switch CPU port 4. Selected external switch ports 0--3 share
one VLAN-unaware bridge domain (VID 1, FID 0). Learned external unicast
traffic remains in hardware; broadcast, unknown unicast and multicast are
flooded to bridge members. ENETC filters CPU unicast using its primary MAC
address. Multicast reception includes IPv6 discovery without optional
multicast group callbacks. Packet sockets see CPU-delivered traffic, not
all hardware-switched transit traffic.

Standalone ENETC0, half-duplex, forced PHY modes, Clause 45 PHYs,
promiscuous transit capture, jumbo frames and checksum/timestamp offloads
are not implemented. Software checksums remain enabled according to the
normal stack configuration. There is no spanning-tree implementation:
external connections must form a loop-free topology.

Board integration is explicit and is not supplied by this driver. Enable
``NETDEV_LATEINIT``, ``NET``, ``NET_ETHERNET``, ``NETDEVICES`` and ``MM_IOB``,
then call::

  imxrt_netc_initialize(&config);

The public types and entry points are in ``imxrt_netc.h``. The driver
copies each enabled controller's configuration and all of its port entries,
including each ``phy_desc_s``; caller-owned storage need not survive
initialization. Declare the board configuration object ``static const``.
Supply all requested interfaces together, since shared IERB setup runs once
before interface initialization. Its fields have the following ownership:

* ``enetc[i]`` describes hardware controller ``i``. Its ``enabled`` flag
  selects the controller; disabled controller settings are ignored.
  Controller count and maximum external port count are chip-defined through
  ``IMXRT_NETC_NENETC`` and ``IMXRT_NETC_SW0_NPORTS``, not board settings.
  The chip hardware table determines direct-PHY versus switch attachment.
  Currently only ``enetc[1]`` is supported; ENETC0's direct-PHY data path
  remains unimplemented.
* ``msgintr`` reserves one complete MSGINTR instance, numbered 1--6,
  exclusively for this driver. Its NVIC interrupt and all three message
  registers must not be used by another owner.
* ``mac`` is a valid unicast MAC from the board's storage/OTP policy.
  Changes through the normal network ioctl take effect on the next
  interface up; the driver updates the static CPU FDB entry then.
* ``enetc[i].ports[p]`` describes external port ``p`` of controller ``i``.
  Each port's ``enabled`` flag selects it; disabled port settings are ignored.
  Unspecified entries in a static initializer are disabled. The CPU port is
  not included. Hardware membership masks are derived internally using
  32-bit values; there is no public port index or mask.
* Each port entry supplies its MII/RMII/GMII/RGMII mode,
  PHY description and supported full-duplex advertisements. The immutable
  chip hardware table supplies each port's MDIO endpoint, not board settings.
  PHY IDs are checked while ignoring the revision nibble. Only the shared
  EMDIO endpoint permits address-range probing; port MDIO endpoints
  require one fixed address. A ``status`` value of ``0xffff`` resolves
  negotiation from standard MII/GMII registers instead of vendor status.
* An optional per-port ``phyinit`` callback runs after identification and
  before advertisement. It receives the port configuration, hardware-selected
  MDIO endpoint and resolved PHY address. Each port selects its PHY-specific
  callback; callbacks must use the supplied endpoint rather than hardcoding
  an ENETC or switch port. It may use the serialized MDIO read/write API
  for PHY-specific delay setup, and must restore the default PHY page.

The switch references the driver-owned controller configuration and the same
immutable hardware definition used by ENETC. Port helpers receive both the
hardware definition and board configuration, without a second port lookup
array or retained membership mask. Link state,
DMA state and the currently installed CPU FDB MAC remain separate runtime
state; the FDB MAC can differ from the initial configured MAC after an ioctl.

Startup and function-reset completion poll ``OSR`` in the switch/ENETC
common register block (offset ``0x1010``), not the separate global block.

The driver queries the NETC root frequency through ``imxrt_get_rootclock()``
at initialization and uses it for IERB timing and MDIO divisors. The board
must enable and stabilize the PLL/root sources first, and keep the frequency
unchanged while NETC is in use. A zero or unsupported frequency is rejected;
the frequency must be in whole MHz.

The board owns pinmux, clock direction, RGMII delay ownership, PHY hardware
reset/timing, NETCMIX reset release and TRDC permissions. Advertisements
must be restricted to modes supported by the board and PHY. In particular,
do not enable 10 Mb/s on the EVK's RTL8201FI PHYs (ERR051651). Do not apply
RGMII delays at both ends of a link.

The complete configuration is validated before IERB setup. Empty selections,
reused MSGINTR channels, invalid enabled-port PHY descriptions, enabled ports
outside a controller's chip-defined capabilities and unsupported controllers
are rejected without locking IERB. Each enabled controller needs at least
one enabled external port.
The driver locks an initially unlocked IERB only when no NETC physical
function has been enabled. It sets the clock-period values and selected
port-MDIO PHY addresses, retaining other resource defaults. It never
unlocks or globally resets shared infrastructure. An already locked IERB
must match the requested frequency and MDIO bindings. The board must
reserve the switch function, ENETC1 and selected MDIO endpoints against
other-core access. Initialization rejects an enabled ENETC1 or switch
function and enabled ENETC1 virtual functions.

DMA uses aligned normal-heap allocations, not dedicated sections or MPU
changes. The complete allocation is translated with ``imxrt_dma_address()``
before enabling DMA. Local M7 DTCM and M33 system TCM use their system-bus
aliases; OCRAM addresses remain unchanged. Ring, packet-buffer and switch
command/response addresses are derived from the translated base, while CPU
access and cache maintenance use the original pointer. Allocations crossing
the helper's supported TCM window are rejected. Boards must retain the TCM
partition supported by that helper (256 KB M7 DTCM or 128 KB M33 system TCM).
System-bus accessibility, TRDC permissions and any external-memory setup
remain board duties.
Packet buffers are 128-byte aligned. Extended 32-byte RX descriptors and
paired 16-byte TX descriptors isolate independently owned entries on
32-byte cache lines. A shared compile-time assertion checks descriptor sizes
and allocation alignment against the selected core's cache-line size.
Explicit clean/invalidate and barriers apply with
both write-through and write-back caches. Chained IOB packets are copied
through staging buffers; no global IOB alignment or section is required.
Packet lengths include the Ethernet header and any on-wire VLAN tag, but
exclude FCS. Hardware supplies TX padding/FCS and removes RX FCS; no VLAN
tag insertion or stripping is requested.

TX/RX ring sizes are selected in Kconfig. TX quota is half the hardware
descriptor count minus one; RX quota is one, since packets are allocated
only when returned to the upper half. SI0 is assigned two TX rings and one
RX ring. Normal switched traffic uses TX ring 1 and RX ring 0; TX ring 0
remains disabled because it is reserved for direct switch enqueue
(IMXRT1180RM section 53.4.2.3.1.2.1.1). Pool limits conservatively account
for chained maximum-size packets and stack headroom. ``MM_IOB`` is an
explicit prerequisite rather than a driver select, avoiding a Kconfig
dependency cycle with these pool constraints.

The common NETDEV worker owns packet polling and protocol dispatch.
Interrupts remain enabled even with a nonzero upper-half polling period.
A separate LPWORK item polls links and detects five-second TX stalls.
Carrier requires an operational CPU path and at least one external link.
Interface down and TX recovery reset only ENETC1, retaining external
switching and link monitoring. DMA references are not released if reset
fails. A failed recovery or loss of the link worker leaves the interface
failed rather than reporting successful operation.

PHY ioctls encode the external switch port in ``phy_id[9:8]`` and its
resolved PHY address in ``phy_id[4:0]``. ``SIOCGMIIPHY`` preserves the
requested port and returns that port's resolved address. MDIO helpers
are task-context operations after the endpoint has been initialized;
runtime multi-register PHY paging must hold the network lock.

No board configuration enables NETC by default. Actual PHY negotiation,
cache-enabled ring wraparound, traffic forwarding, reset recovery and
silicon-specific acceptance require later validation on the final board.

Data and Instruction Cache
==========================

MCUs i.MX RT1010 and higher have separated caches for instructions and data. Data cache is initially
set as write-through but can be changed to write-back via Kconfig. While write-back gives better
performance than write-through, it is not supported for all peripherals in NuttX yet. Write-back data
cache can not be selected while running legacy ENET or serial port over USB.
The NETC architecture driver uses explicit DMA cache maintenance.

Tickless OS
===========

With Tickless OS, the periodic, timer interrupt is eliminated and replaced with a one-shot,
interval timer, that becomes event driven instead of polled. This allows to run the MCU with
higher resolution without using more of the CPU bandwidth processing useless interrupts.

Only tickless via an alarm is currently supported for i.MX RT MCU, which can be selected by
CONFIG_SCHED_TICKLESS_ALARM option. CONFIG_USEC_PER_TICK option determines the resolution
of time reported by :c:func:`clock_systime_ticks()` and the resolution of times that can be set
for certain delays including watchdog timers and delayed work. It is important that value set in
CONFIG_USEC_PER_TICK represents the frequency of GPT timer that runs the tickless mode. Clock
source of the timer is 16.6 MHz, which is then divided by the prescaler value from 1 to 4096.
Possible values for CONFIG_USEC_PER_TICK are 10 or 100 for example.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======
Peripheral  Support
==========  =======
ACMP        No
ADC         Yes
CAN         Yes
CSI         No
DAC         No
eLCDIF      Yes
ENC         Yes
ENET        Yes
FlexIO      No
GPIO        Yes
I2S         Yes
PWM         Yes
SAI         No
SPDIF       No
SPI         Yes
UART        Yes
USB         Yes
==========  =======

ACMP
----

The circout for comparing two analog input voltages designed to operate across the full
range of supply voltage (rail-to-rail operation).

ADC
---

ADC driver with the successive approximation analog/digital converter. The lower-half of
this driver is initialize by calling :c:func:`imxrt_adcinitialize`.

ADC module can use either continuous trigger (next conversion is started as soon as the
previous is finished) or hardware trigger. This option is selected by IMXRT_ADCx_ETC
(x = 1, 2) config option. If IMXRT_ADCx_ETC = -1 then continuous trigger is used. If
corresponding XBAR number is put in IMXRT_ADCx_ETC then that signal is used to trigger
the ADC conversion (for example PWM signal can be used as a source). For PWM XBAR options
please refer to PWM chapter of this documentation.

Hardware triggering is currently limited to maximum of 8 channels. HW trigger is automatically
disabled if there are more than 8 channels.

DMA is currently not supported for ADC modules.

CAN
---

FlexCAN driver is supported in MCUs i.MX RT1020 and higher. i.MX RT106x have both classical
CAN and also one CAN FD while i.MX RT1170 have 3 CAN FD peripherals. FlexCAN driver in imxrt
works beyond SocketCAN driver layout. The lower-half of this driver is initialize by
calling :c:func:`imxrt_cannitialize()`.

There is an booting option that automatically provides initialization of network interface
in the early stages of booting and therefore calling :c:func:`imxrt_cannitialize()` via
board specific logic is not necessary. This however works only when there is only one
interface in the chip. For running more interfaces (like CAN and Ethernet), network late
initialization must be turn on by CONFIG_NETDEV_LATEINIT and board specific logic must call
lower-half part of drivers.

CSI
---

CMOS Sensor interface which enables the chip to connect directly to external CMOS image sensors.

DAC
---

Digital/analog converter for external signal is only supported in i.MX RT1170 MCU. It is 12 bit
lower power, general purpose DAC.

eLCDIF
------

The enhanced Liquid Crystal Display interface (LCDIF) is a general purpose display controller.

ENC
---

The enhanced quadrature encoder/decoder module supported in i.MX RT1015 and higher. The lower-half
of this driver is initialize by calling :c:func:`imxrt_qeinitialize`.

ENET
----

Ethernet driver supported in i.MX RT1020 and higher. The lower-half of this driver is initialize
by calling :c:func:`imxrt_netnitialize`.

There is an booting option that automatically provides initialization of network interface
in the early stages of booting and therefore calling :c:func:`imxrt_cannitialize()` via
board specific logic is not necessary. This however works only when there is only one
interface in the chip. For running more interfaces (like CAN and Ethernet), network late
initialization must be turn on by CONFIG_NETDEV_LATEINIT and board specific logic must call
lower-half part of drivers.

FlexIO
------

A configurable module providing a range of functionality like emulation of a variety of
serial/parallel communication protocols, flexible 16-bit timers or programmable logic blocks.
This module is supported in i.MX RT1010 and higher.

GPIO
----

Pins can be configured using :c:func:`imxrt_config_gpio` function. Writing to pins is
done by :c:func:`imxrt_gpio_write` function and reading is done by :c:func:`imxrt_gpio_read`.

MCUs i.MX RT1060 and higher includes both standard speed GPIOs (1-5) and high speed
GPIOS (6-9). Regular and high speed GPIO share the same pins (GPIO1 is with GPIO6 etc),
therefore IOMUXC_GPR_GPR26-29 registers are used to determine what module is used for the
GPIO pins.

I2C
---

Inter-Integrated Circout module supporting an interface to an I2C bus as master and/or
as a slave. The lower-half of this driver is initialize by calling :c:func:`imxrt_i2cbus_initialize`.

PWM
---

Pulse width modulator supported in i.MX RT1010 and higher. Multiple channels option is available.
Output on pin B is currently supported only as a complementary option to pin A.
The lower-half of this driver is initialize by calling :c:func:`imxrt_pwminitialize`.

PWM module can be synchronized by an external signal. The external signal used for synchronization
is selected by IMXRT_FLEXPWMx_MODx_SYNC_SRC config option. The number in IMXRT_FLEXPWMx_MODx_SYNC_SRC
corresponds with the XBAR number. Following numbers can be used for synchronization of PWMs with other
PWM module when using iMXRT1020, iMXRT1050 or iMXRT1060.

- PWM1 Module 1 = 40
- PWM1 Module 2 = 41
- PWM1 Module 3 = 42
- PWM1 Module 4 = 43
- PWM2 Module 1 = 44
- PWM2 Module 2 = 45
- PWM2 Module 3 = 46
- PWM2 Module 4 = 47
- PWM3 Module 1 = 48
- PWM3 Module 2 = 49
- PWM3 Module 3 = 50
- PWM3 Module 4 = 51
- PWM4 Module 1 = 52
- PWM4 Module 2 = 53
- PWM4 Module 3 = 54
- PWM4 Module 4 = 55

iMXRT1170 has different XBAR connections:

- PWM1 Module 1 = 74
- PWM1 Module 2 = 75
- PWM1 Module 3 = 76
- PWM1 Module 4 = 77
- PWM2 Module 1 = 78
- PWM2 Module 2 = 79
- PWM2 Module 3 = 80
- PWM2 Module 4 = 81
- PWM3 Module 1 = 82
- PWM3 Module 2 = 83
- PWM3 Module 3 = 84
- PWM3 Module 4 = 85
- PWM4 Module 1 = 86
- PWM4 Module 2 = 87
- PWM4 Module 3 = 88
- PWM4 Module 4 = 89

Option IMXRT_FLEXPWMx_MODx_TRIG allows the module to generate a trigger signal. The trigger is generated on
timer capture of either period or duty cycle value based on the configuration.

SAI
---

Synchronous audio interface provided by I2C module. Supported in i.MX RT1015 and higher.

SPDIF
-----

Sony/Philips digital interface audio block. It is a stereo transceiver that allows the
processor to receive and transmit digital audio. Supported in i.MX RT1010 and higher.

SPI
---

Serial Peripheral interface module that supports an interface to an SPI bus as a master
and/or a slave. The lower-half of this driver is initialize by calling :c:func:`imxrt_lpsibus_initialize`.

UART
----

Universal Asynchronous Receiver/Transmitter module. UART is initialized automatically during
MCU boot.

USB
---

Console communication over USB is supported via CDC-ACM. Only USB Device is currently supported
for i.MX RT in NuttX

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
