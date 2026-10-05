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

After a mode fault, the driver clears the fault and restores the cached
master/mode configuration without replaying the interrupted transfer. If
restoration fails, the bus is marked faulted and rejects subsequent transfers.

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
