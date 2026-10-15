================
i.MX RT1180 EVK
================

.. tags:: arch:arm, arch:armv7m, arch:armv8m, arch:cm7, arch:cm33, chip:imxrt, chip:imxrt1180, vendor:nxp, ethernet, usb

The `i.MX RT1180 EVK
<https://www.nxp.com/design/design-center/development-boards-and-designs/MIMXRT1180-EVK>`_
is an evaluation kit from NXP. It is based on the MIMXRT1189 crossover
microcontroller, which combines Arm Cortex-M33 and Cortex-M7 cores.
See the `MIMXRT1180-EVK Quick Start Guide
<https://www.nxp.com/docs/en/quick-reference-guide/IMXRT1180QSG.pdf>`_ for
an overview of the board and its connectors.

.. figure:: https://www.nxp.com/assets/images/en/dev-board-image/MIMXRT1180-EVK-TOP-IMG.jpg
   :align: center
   :alt: Top view of the i.MX RT1180 EVK
   :width: 80%

   i.MX RT1180 EVK top view. Image source: NXP.

Features
========

- MIMXRT1189CVM8C microcontroller
    - Cortex-M33 running at 266 MHz with NuttX
    - Cortex-M7 running at 798 MHz with NuttX
- Memory
    - On-chip TCM and OCRAM
    - 512-Mbit external SDRAM
    - 128-Mbit FlexSPI1 QSPI NOR flash
    - 512-Mbit HyperRAM
    - MicroSD card socket
- Connectivity and expansion
    - MCU-Link debug interface with a virtual COM port
    - 20-pin external JTAG/SWD connector
    - Ethernet connectors
    - USB host and device connectors
    - CAN connector
    - Arduino-compatible expansion headers
- Audio codec, microphone, headphone jack, and speaker connections
- Two software-controlled user LEDs

The NuttX configurations provide the serial console, USB device support,
image generation, and multicore boot flow.  The ``netc`` configuration adds
switched Ethernet as described below; the remaining interfaces depend on
the selected configuration.

NETC switched Ethernet
======================

The ``netc`` configuration is based on the Cortex-M7 ``nsh`` configuration
and enables the new NETC driver.  It exposes one network interface through
ENETC1 and the internal switch CPU port 4, not four independent interfaces.
External ETH4/J32 belongs to ENETC0 and is not enabled.

.. list-table:: External switch ports
   :header-rows: 1

   * - Port / connector
     - PHY / address
     - Interface / supported speeds
     - Active-low reset
   * - ETH0 / J28
     - RTL8201FI / 2
     - RMII / 100 Mb/s only
     - GPIO_AD_25 (RGPIO4.25)
   * - ETH1 / J29
     - RTL8211FD / 5
     - RGMII / 10, 100, 1000 Mb/s
     - GPIO_B1_13 (RGPIO6.13)
   * - ETH2 / J30
     - RTL8211FD / 4
     - RGMII / 10, 100, 1000 Mb/s
     - GPIO_AD_28 (RGPIO4.28)
   * - ETH3 / J31
     - RTL8211FD / 7
     - RGMII / 10, 100, 1000 Mb/s
     - GPIO_B2_01 (RGPIO6.15)

``CONFIG_IMXRT_NETC_PORTMASK`` is a decimal bitmap, range 1..15, with bit n
selecting ETHn.  The board default is 15; the supplied bring-up configuration
uses 2 (ETH1 only).  Connect J29 first and validate link, ARP and ping before
adding other ports.  RTL8201 10 Mb/s operation is unsupported by RT1180 NETC
(ERR051651); do not force ETH0 to 10 Mb/s.  These are supported driver modes,
not a claim of hardware validation on every board/silicon revision.

Before selecting ETH2, set J65 and J72 to 2-3; before selecting ETH3, set J63
and J73 to 2-3 (UM12021 Rev. 3, Table 4).  ETH2/ETH3 share routing with SDRAM
and EtherCAT.  Do not enable those functions on the shared pins, including
in M33 firmware or a debugger initialization script.  The board glue rejects
SEMC with ETH2/3 selected.  RMII on ETH0 assumes the standard resistor
population, without the manual's MII rework.

The pad routes and reset sequence follow the MCUXpresso
``mcuxsdk-examples/_boards/evkmimxrt1180/driver_examples/netc/switch``
``pin_mux.c`` and ``hardware_init.c``.  Management pads are GPIO_AD_30
(EMDC) and GPIO_AD_31 (EMDIO); the configuration uses the individual port
external MDIO register groups, not PCS MDIO.  The SDK's ``board.h`` specifies
PHY addresses 2, 5, 4, 7.  Its RTL8211F helper enables both RX and TX clock
delays, which the board configuration requests from the driver; do not add
a second MAC-side delay.  All five onboard PHYs are reset as in the SDK,
including unused ETH4 (GPIO_AD_13 / RGPIO4.13).  Thus M33 firmware must not
concurrently use ETH4.  Setup allows 150 ms power settling, 10 ms reset and
150 ms recovery.

With ``CONFIG_IMXRT_NETC``, M7 startup configures the SDK clock roots:
NETC = SYS_PLL3_PFD3 / 2 = 240 MHz, MAC0 = SYS_PLL1_DIV2 / 10 = 50 MHz,
and MAC1-3 = SYS_PLL1_DIV2 / 4 = 125 MHz.  M33 NuttX startup initializes
SYS_PLL1 from the board's ``sys_pll1`` configuration when ``enable`` is set
and enables its DIV2 and DIV5 outputs.  An already-locked PLL is preserved.
Rebuild and flash the M33 bootloader when updating this clock setup;
updating only the M7 application does not initialize SYS_PLL1.
SYS_PLL2/3 and their PFDs still retain their earlier boot configuration.
ETH0's RMII reference clock is an MCU output.  The PHYs' separate 25 MHz
crystals must not be confused
with this 50 MHz reference.  Clock frequencies are checked before PHY reset.
The MAC is locally administered unicast, derived from the existing RT118x
OCOTP UID shadow definitions using the conventional i.MX RT ENET mapping.
It is not a factory-assigned globally unique MAC.

Boot, memory and initialization requirements
-------------------------------------------

M33 boot firmware must establish TRDC permissions before releasing M7.
The supplied board policy already grants domain 10 (NETC) access to OCRAM1
and OCRAM2 and grants M7 access to the NETC register window.  Keep that policy
when replacing the boot firmware.  NETCMIX is on after initial power-up
(IMXRT1180RM 27.3.4.2); the board hook checks that power, reset release and
isolation status are ready.  If M33 has powered it down, initialization fails
with ``-EHOSTDOWN`` rather than attempting to seize SRC/TRDC ownership or
reset another core's resources.  Only one core may own NETC.  IERB, PRB,
PCI functions and MSI-X are initialized by the driver, not duplicated here.

The ``netc`` configuration uses ``flash.ld`` with data, BSS, idle stack and
primary heap in DTCM, like ``nsh``, and a secondary heap in OCRAM1
(``CONFIG_IMXRT_TCM_PRIMARY=y``, ``CONFIG_IMXRT_OCRAM_HEAP=y``,
``CONFIG_MM_REGIONS=2``).  NETC DMA storage is static, not allocated with
``kmm_memalign``.  The linker places ``.netc_dma.packet`` and
``.netc_dma.command`` in a dedicated ``.netc_dma`` NOLOAD section in OCRAM1,
after USB's separate ``.dmamemory`` range and before the OCRAM heap.  Input
section alignment is preserved, and all alignment padding is excluded from
the heap.  ROM patch memory and the M33 reserve remain excluded; linker
assertions check OCRAM bounds and separation from USB and the heap.

Packet/ring storage has size and alignment equal to the next power of two
covering the packet structure.  With 16 RX / 16 TX descriptors, the supplied
590-byte packet setting reserves 32 KiB; a 1518-byte setting reserves 64 KiB.
Switch command storage has size and alignment of 512 bytes.  The driver
explicitly initializes both blocks because NOLOAD storage is not cleared by
the BSS startup code.  DMA placement has no heap-location or fragmentation
prerequisite.  ``flash-netc.ld`` remains available when both NETC and
``CONFIG_IMXRT_OCRAM_PRIMARY=y`` are selected; it keeps the same static DMA
placement with data, BSS, idle stack and primary heap in OCRAM1.

With data cache enabled, ``CONFIG_ARM_MPU=y`` and **two free MPU regions**
are required: one for packet/ring memory and one for command memory.
``netc`` sets ``CONFIG_ARM_MPU_NREGIONS=16``; account for startup mappings,
USB DMA and any other MPU users when extending it.  Each MPU mapping covers
exactly its static storage block, not the aggregate linker section or its
padding.  Both blocks must remain non-cacheable and reachable under the M33
TRDC policy.

``CONFIG_NETDEV_LATEINIT=y`` is recommended and selected by ``netc``.
``imxrt_bringup()`` calls ``imxrt_netc_boardinitialize()`` after normal board
initialization.  If late network initialization is disabled, the board's
``arm_netinitialize()`` calls the same hook during architecture network
initialization, after heap setup; do not call the network setup hook from
``imxrt_boardinitialize()``.  Do not supply a second architecture-level
``arm_netinitialize()`` for NETC.  A mutex and a single board lifecycle state
prevent repeated PHY resets or duplicate registration, including after a
failed first attempt.  Recovering from an initialization failure requires a
reboot.

Buttons and LEDs
================

The board port configures the following user LEDs during early
initialization:

=== ===== ========== =========
LED Color Signal     GPIO
=== ===== ========== =========
D6  Green GPIO_AD_27 RGPIO4.27
D7  Red   GPIO_AD_26 RGPIO4.26
=== ===== ========== =========

D7 is turned on during board initialization. The supplied configurations do
not enable the NuttX automatic LED or user-button interfaces.

Serial Console
==============

The default serial console is LPUART1 through the MCU-Link virtual COM port:

============ ============ =============
Function     Signal       Pad
============ ============ =============
Console TX   LPUART1_TX   GPIO_AON_08
Console RX   LPUART1_RX   GPIO_AON_09
============ ============ =============

Connect the host to the MCU-Link USB connector J53. On Linux, the console is
normally available as ``/dev/ttyACM0``.

Pin Mapping
===========

The board port currently defines these on-board connections:

============ ============= ==============================
Pad          Function      Board connection
============ ============= ==============================
GPIO_AON_04  LPSPI1_SCK    On-board LPSPI NOR flash
GPIO_AON_05  LPSPI1_PCS0   On-board LPSPI NOR flash
GPIO_AON_06  LPSPI1_SDO    On-board LPSPI NOR flash
GPIO_AON_07  LPSPI1_SDI    On-board LPSPI NOR flash
GPIO_AON_08  LPUART1_TX    MCU-Link virtual COM port
GPIO_AON_09  LPUART1_RX    MCU-Link virtual COM port
GPIO_AON_15  LPI2C2_SDA    On-board peripherals
GPIO_AON_16  LPI2C2_SCL    On-board peripherals
GPIO_AD_26   GPIO4_IO26    Red user LED D7
GPIO_AD_27   GPIO4_IO27    Green user LED D6
============ ============= ==============================

Consult the MIMXRT1180-EVK user manual (UM12021) and schematic, available from
the NXP product page linked above, for the complete expansion-header pinout and
any signal-sharing restrictions.

Power Supply
============

The board requires a 5 V supply. It can be powered from the external 5 V input
J2, or from a supported USB connector when the power-selection jumpers are
configured accordingly. Use SW1 to turn the board on. Refer to the board user
manual before changing the factory jumper settings or powering the board from
more than one source.

Installation
============

All testing has been conducted using the following Arm GNU Toolchain for the
``arm-none-eabi`` target, available from the `Arm GNU Toolchain downloads page
<https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>`_:

.. code-block:: console

   $ arm-none-eabi-gcc --version
   arm-none-eabi-gcc (Arm GNU Toolchain 13.2.rel1 (Build arm-13.7)) 13.2.1 20231009

Building a Cortex-M33 image also requires ``python3`` with the ``venv``
module, ``pip``, and either ``curl`` or ``wget``. The image-building helper
creates a private Python environment and installs the pinned NXP SPSDK
version automatically.

Flashing requires the `SEGGER J-Link Software and Documentation Pack
<https://www.segger.com/downloads/jlink/>`_ version 7.98a or later, one of:

- a SEGGER J-Link probe connected to the 20-pin JTAG/SWD connector J37; or
- the on-board MCU-Link probe, reflashed with SEGGER MCU-Link J-Link firmware
  and connected through J53,

and the boot-mode DIP switch SW5 set to ``0100`` (FlexSPI Quad SPI NOR boot).

Building NuttX
==============

Select a configuration and build it from the NuttX repository root:

.. code:: console

   $ ./tools/configure.sh imxrt1180-evk:<config>
   $ make -j$(nproc)

The first Cortex-M33 build downloads the pinned NXP EdgeLock Enclave (ELE)
firmware after displaying its license and installs SPSDK into
``tools/imxrt1180/.cache``. Running the build implies acceptance of the NXP
firmware license described in `License Exceptions`_.

Flash Layout
============

The Cortex-M33 image is the bootable image. It contains the FlexSPI
Configuration Block (FCB) at offset ``0x400`` and the AHAB container at offset
``0x1000``. The Cortex-M7 image is programmed separately as a raw XIP payload.

========== ======== ====== ==============================
Address    Offset   Size   Contents
========== ======== ====== ==============================
0x28000000 0x0      512 KB Cortex-M33 image, ``flash.bin``
0x28080000 0x80000  Rest   Cortex-M7 image, ``nuttx.bin``
========== ======== ====== ==============================

Flashing
========

Set the boot-mode DIP switch SW5 to ``0100`` for FlexSPI Quad SPI NOR boot.
The ``flash-jlink.sh`` helper takes an image name and either a flash offset or
an absolute address.

The helper programs through the Cortex-M33 device profile because the RT1180
ROM starts the Cortex-M33 first and initializes FlexSPI1 before the SEGGER
flash loader runs. Always use the supplied helper for programming. The
Cortex-M7 J-Link device profile is intended for debugging the running
Cortex-M7 image, not for programming.

To flash a standalone Cortex-M33 NSH image:

.. code:: console

   $ ./tools/configure.sh imxrt1180-evk:nsh-m33
   $ make -j$(nproc)
   $ boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh flash.bin 0x0

To flash the Cortex-M33 bootloader and Cortex-M7 NSH images:

.. code:: console

   $ ./tools/configure.sh imxrt1180-evk:bl
   $ make -j$(nproc)
   $ cp flash.bin flash-bl.bin
   $ make distclean
   $ ./tools/configure.sh imxrt1180-evk:nsh
   $ make -j$(nproc)
   $ cp nuttx.bin nuttx-m7.bin
   $ boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh flash-bl.bin 0x0
   $ boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh nuttx-m7.bin 0x80000

J-Link Debugging
================

The board port also supplies a J-Link GDB server helper for Cortex-M7:

.. code:: console

   $ boards/arm/imxrt/imxrt1180-evk/tools/gdbserver-jlink.sh

Connect from another terminal:

.. code:: console

   $ arm-none-eabi-gdb nuttx
   (gdb) target extended-remote localhost:2331
   (gdb) monitor reset
   (gdb) continue

Configurations
==============

The board identifier for use with ``tools/configure.sh`` is
``imxrt1180-evk``.

nsh-m33
-------

Runs NuttShell (NSH) directly on the Cortex-M33. The RT1180 ROM boots
``flash.bin`` from FlexSPI1 QSPI NOR. NSH is available on LPUART1 at
the MCU-Link virtual COM port. This configuration also enables USB CDC/ACM
and the ``ostest`` application.

bl
--

Builds a minimal Cortex-M33 bootloader as ``flash.bin``. It sets the
Cortex-M7 boot address to ``0x28080000``, releases that core, and then remains
in an indefinite loop that periodically calls ``usleep``. Use this
configuration together with ``nsh``; it does not provide an interactive shell
itself.

nsh
---

Runs NSH on the Cortex-M7. The output ``nuttx.bin`` is a raw XIP payload that
must be programmed at offset ``0x80000`` and started by the ``bl``
configuration. NSH is available on LPUART1 at the MCU-Link virtual COM port.
This configuration also enables USB CDC/ACM and the ``ostest`` application.

License Exceptions
==================

The Cortex-M33 image build downloads
``mxrt1180b0-ahab-container.img`` from NXP's MCUXpresso SDK at a pinned commit.
This proprietary EdgeLock Enclave firmware is covered by the LA_OPT NXP
Software License, version 56 (April 2024). The download helper displays the
source URL, license URL, and expected SHA-256 digest before downloading it.

The NXP SPSDK Python package used to assemble the boot image is distributed
under the BSD-3-Clause license.
