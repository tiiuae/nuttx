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

==========  =======  ============================================================
Peripheral  Support  Notes
==========  =======  ============================================================
GPIO        Yes      GPIO-backed EXTI lines 0-15 via ``stm32_gpiosetevent()``
PWR         Yes      Partial.
RCC         Yes      PLL1 clock tree.
SPI         Partial  SPI1-SPI6 polling master; board-owned chip select
USART       Yes      USART1 only.

ADC         No
DCACHE      Partial  Cache maintenance through ARMv8-M primitives
DCMIPP      No
DMA         Partial  GPDMA1/HPDMA1 core; initial USART1 TX
ETH         No
I2C         No
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
==========  =======  ============================================================

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
