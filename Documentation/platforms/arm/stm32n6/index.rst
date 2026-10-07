==========
ST STM32N6
==========

This is a port of NuttX to the STM32N6 family.
The STM32N6 is a chip based on the Arm Cortex-M55.

Development is performed on the Nucleo-N657X0-Q.  At this time only the
STM32N657X0 is supported.  Kconfig will need updates to support other
MCUs in the family.

Supported MCUs
==============

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32N645     No
STM32N647     No
STM32N655     No
STM32N657     Yes    STM32N657X0 only
===========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  ====================================================================
Peripheral  Support  Notes
==========  =======  ====================================================================
GPIO        Yes      GPIO-backed EXTI lines 0-15 via ``stm32_gpiosetevent()``
PWR         Partial  Board power and I/O voltage setup.
RCC         Partial  PLL1 system clock setup and selected peripheral clock gates.
SPI         Partial  SPI1-SPI6 polling master; board-owned chip select
USART       Partial  USART1-3/6/10 and UART4/5/7/8/9 serial drivers.

ADC         No
DCACHE      Partial  Cache maintenance through ARMv8-M primitives
DCMIPP      No
DMA         Partial  GPDMA1/HPDMA1 core; initial USART1 TX
ETH         No
I2C         Partial  I2C1-I2C4 master; board timing/wiring qualification pending
ICACHE      No
IWDG        No
LPTIM       No
LTDC        No
MPU         No
NPU         No
RNG         No
RTC         No
SAI         No
SDMMC       No
TIM         Partial  TIM1-TIM18 driver; no board PWM/DShot client yet
USB         No
XSPI        No
==========  =======  ====================================================================

RCC Support
===========

The STM32N6 RCC support configures the board's PLL1-based CPU and system
clock tree and enables selected clocks needed by NuttX. For the
Nucleo-N657X0-Q, PLL1 is fed by the 64 MHz HSI: M=4 and N=50 produce an
800 MHz VCO, and IC1 divides it to a 200 MHz CPU clock. IC2, IC6, and IC11
provide the system clock inputs; the configured bus prescalers result in
50 MHz HCLK, PCLK1, and PCLK2. The 600/800 MHz CPU operating points are not
currently configured.

If an earlier boot stage has already switched the CPU and system clocks to
the expected PLL1 outputs, startup checks the bus prescalers and leaves the
locked clock configuration in place. Otherwise it configures PLL1, its
output dividers, the system clock switch, and the bus prescalers. Startup
also enables the SRAM and GPIO banks and PWR clocks, plus selected DMA and
USART1 clocks. Other peripheral drivers manage their own instance clocks
when initialized; RCC support does not enable every peripheral.

The initialization does not program the shared HSI divider (HSIDIV). Serial,
SPI, and I2C kernel-clock users that select ``hsi_div_ck`` therefore inherit
its current value from reset or an earlier boot stage. They must account for
the active divider and keep the shared clock configuration stable while in
use. This is partial RCC support, not a general runtime clock-management API.

USART Support
=============

The serial driver in ``arch/arm/src/stm32n6/stm32_serial.c`` provides
conditional instances for USART1, USART2, USART3, USART6, USART10, UART4,
UART5, UART7, UART8, and UART9. Each enabled instance requires board TX/RX
pin definitions; optional RTS/CTS flow control also requires matching board
pins. Instances use the HSI-derived kernel clock selected through their RCC
kernel-clock mux and account for the inherited HSIDIV value.

The driver supports interrupt-driven serial I/O, an early polled console,
7- or 8-bit payloads, no/even/odd parity, and one or two stop bits. With
``CONFIG_SERIAL_TERMIOS``, supported format and baud settings can be changed
at runtime. TX DMA is optional through GPDMA1; RX DMA is not implemented.
The DMA, termios, and flow-control options do not imply that a board has
electrically routed or validated the corresponding pins and signals.

On the Nucleo-N657X0-Q, USART1 on PE5/PE6 is the ST-LINK VCOM console.
USART3 on PD8/PD9 is an optional Arduino D1/D0 test-port route enabled in
the ``nsh-test`` configuration; its target-level serial qualification is
pending. The other serial instances need separate board pin assignments.
See the :doc:`Nucleo-N657X0-Q board description
<boards/nucleo-n657x0-q/index>` for the board routes and configuration
details.

I2C Support
===========

The STM32N6 driver in ``arch/arm/src/stm32n6/stm32_i2c.c`` implements
synchronous I2C master transfers for I2C1-I2C4. It supports unshifted
7-bit addresses, 100 kHz Standard-mode and 400 kHz Fast-mode frequency
ceilings, reads, writes, clock stretching, and transfers longer than
255 bytes using NBYTES reloads. A zero-length write is an address-only
probe; zero-length reads are rejected. Ten-bit addressing, Fast-mode Plus,
DMA, target/slave operation, and SMBus/PEC are not implemented.

Enable ``CONFIG_STM32_I2C`` and the required ``CONFIG_STM32_I2Cn`` instance.
Interrupt-driven byte service uses both event and error IRQs;
``CONFIG_I2C_POLLED`` selects the polling path through the same transfer
engine. Transfers are serialized by a per-bus mutex and return zero on
success or a negative errno on failure. Both paths require thread context
with interrupts enabled; ISR and interrupt-masked callers are rejected.

Each enabled bus requires board-owned pins, kernel/APB clock frequencies,
oscillator tolerance, rise/fall times, and filter settings. The driver
currently requires ``hsi_div_ck``, verifies its source and HSI readiness,
and reads HSIDIV to check the actual kernel frequency against the board
contract. It does not change the shared oscillator divider. TIMINGR is
calculated from those inputs and programmed only while idle with PE
disabled. Invalid or unachievable timing is rejected rather than silently
clamping the requested frequency. Clock configuration must remain stable.

Message boundaries follow the NuttX ``i2c_msg_s`` flag contract:

* Ordinary messages are separated by STOP, a bus-free interval, and START.
* ``I2C_M_NOSTOP`` followed by a message without ``I2C_M_NOSTART`` requests
  a repeated START.
* ``I2C_M_NOSTART`` continues the preceding payload without another address
  phase; address, direction, and frequency must match.

A vector must use one frequency. An initial NOSTART, a final NOSTOP, and
zero-length continuations are rejected. Internal 255-byte reload boundaries
do not insert START or STOP.

Timeouts and recovery
--------------------

The active-transfer timeout is ``CONFIG_STM32_I2CTIMEOTICKS`` system ticks.
With ``CONFIG_STM32_I2C_DYNTIMEO``, the driver instead computes a budget
from payload/address bytes and START/STOP phases:

.. code:: text

   (payload bytes + address phases) * USECPERBYTE microseconds
   + (address phases + stop phases) * STARTSTOP milliseconds

Here ``USECPERBYTE`` and ``STARTSTOP`` are
``CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE`` and
``CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP`` respectively. Clock stretching
consumes the remaining aggregate transfer budget.

Bus-idle waits have a separate 25 ms limit. After a failed transfer, the
driver disables byte service and attempts STOP cleanup with a 20 ms
deadline. It checks status immediately, then requests 100 microsecond
``nxsig_usleep()`` intervals instead of busy-waiting. The bus mutex remains
held: other threads can execute, but other clients of that bus must wait.
Interrupted sleeps retry within the same deadline; other sleep errors
propagate to cleanup failure handling. Tick granularity and scheduling
latency can extend the elapsed time beyond the nominal deadline.

Abort cleanup checks pending START/STOP requests as well as BUSY. A pending
START, or a pending control request on an idle bus, triggers a controller
reset and timing restore so an abandoned request cannot execute later.
Quiesced cleanup flushes nonempty TXDR, drains RXDR, and clears applicable
flags. Arbitration loss returns ``-EAGAIN`` without requesting STOP or
actively recovering the bus; the local TXDR flush does not generate bus
conditions. NACK returns ``-ENXIO``, deadline expiry ``-ETIMEDOUT``, and
bus/state errors ``-EIO``. The original transfer error is preserved if
cleanup also fails.

Failed STOP cleanup marks the bus faulted and attempts a local controller
reset; the bus remains unavailable until successful explicit recovery or
clean uninitialization/reinitialization. A controller reset is not proof
that the physical lines are free. ``CONFIG_I2C_RESET`` exposes GPIO recovery
for single-master wiring only: release both open-drain lines, wait for SCL,
generate up to nine clock pulses if SDA is held low, attempt STOP, and
restore alternate functions and controller timing. Recovery uses a shared
100 ms deadline and reports failure if the lines cannot be released.
The lower half never automatically replays a failed transfer.

Qualification
-------------

Host tests cover timing constraints, message validation, and reload
decisions. They do not qualify the complete IRQ state machine or physical
bus behavior. The Nucleo board currently provides only the I2C2 PB10/PB11
route, with intentionally unqualified timing inputs. Initialization fails
until valid board timing inputs are supplied. I2C1, I2C3, and I2C4 require
their own board definitions and target validation.

SPI Support
===========

The STM32N6 SPI driver in ``arch/arm/src/stm32n6/stm32_spi.c`` provides
polling, full-duplex master transfers for SPI1 through SPI6. It supports
modes 0-3, MSB-first 8- and 16-bit frames, finite transfers up to the
controller's transfer-size limit, and GPIO chip select through board
callbacks. The bus must be locked around a transaction when shared.

All instances select ``hsi_div_ck`` through ``RCC_CCIPR9``. The driver
verifies that selection and HSI readiness, and reads ``RCC_HSICFGR.HSIDIV``
to derive the kernel frequency from the board's ``STM32_HSI_FREQUENCY``.
Both the initial /256 SCK and subsequent prescaler selections use this
divided frequency; transfer deadlines use the resulting actual SCK.
The driver does not change HSIDIV or other shared boot clocks. Clock
configuration must remain stable during a transfer. If HSIDIV changes
between transfers, the next nonempty transfer fails until
``SPI_SETFREQUENCY`` successfully refreshes the prescaler and cached clock.

FIFO service and completion waits use a DWT cycle-counter deadline based
on the chunk's wire time plus 100 ms. Recovery uses a separate 100 ms
deadline shared by suspension and RX FIFO draining. These deadlines do not
depend on scheduler ticks and remain usable with interrupts masked. A
recovery timeout is logged and leaves the bus faulted.

After a mode fault, the driver clears the fault and restores the cached
master/mode configuration without replaying the interrupted transfer. If
restoration fails, the bus is marked faulted and rejects subsequent transfers.

The NuttX exchange and block-transfer methods return no status. STM32N6
callers can use ``stm32_spi_getlasterror(dev)`` from ``stm32_spi.h`` to read
the last configuration/transfer result: zero on success or a negative errno
on failure. Read it after the operation and chip-select cleanup, while
still owning the bus and before another configuration or transfer. Reading
does not clear the result; a subsequent successful operation, including a
zero-length transfer, clears it. ``SPI_STATUS`` remains a device-presence
status, not a transfer-error channel.

SPI DMA and interrupt-driven transfers are not implemented. Enabling an
SPI DMA or interrupt option for STM32N6 causes a build-time error. Configure
``CONFIG_STM32_SPI`` and the required ``CONFIG_STM32_SPIn`` option to enable
a bus; buses not selected in the configuration remain disabled.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

References
==========

[RM0486] STMicroelectronics, STM32N647/657xx Arm®-based 32-bit MCUs
