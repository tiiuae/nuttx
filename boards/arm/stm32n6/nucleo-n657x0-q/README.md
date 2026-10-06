# NUCLEO-N657X0-Q

Build and flash instructions for getting the board to boot NuttX standalone
from the external Octo-SPI NOR flash, using the `bl` (SRAM2 bootloader) and
`nsh-xspi` (XIP application) configurations.

## Why two images

The STM32N657X0 has no internal flash, so the boot ROM has to fetch the first
image from the MX25UM51245G Octo-SPI NOR on XSPI2. The ROM reaches that flash
through the XSPI1 controller in indirect mode and leaves XSPI2 clock-gated, so
it cannot start an XIP image directly. Booting therefore takes two stages:

| Stage | Config    | Linker script | Flash address | Runs from                 |
|-------|-----------|---------------|---------------|---------------------------|
| 1     | `bl`      | `bl_flash.ld` | `0x70000000`  | AXI SRAM2 `0x34180400`    |
| 2     | `nsh-xspi`| `flash.ld`    | `0x70100000`  | XIP from `0x70100400`     |

The boot ROM copies the stage 1 payload into SRAM2 and enters it (LRUN). The
bootloader configures XSPI2 for memory-mapped reads at `0x70000000`, then
branches to the vector table of the stage 2 image at `0x70100400`. Stage 2
executes code and read-only data in place from flash; only writable data is
copied to AXI SRAM.

Both images are wrapped in an STM32 v2.3 boot header (`tools/mkimage.sh`)
because the ROM only accepts headered images, and the bootloader locates
stage 2 at a fixed offset just past its own 0x400-byte header.

There is also a `nsh` / `ostest` / `leds` set of configurations that build for
DEV boot mode (`sram.ld`, loaded by the debugger with `tools/sramload.sh`).
Those are for quick edit/debug cycles and are not covered here.

## Prerequisites

* `arm-none-eabi-` toolchain (Arm GNU Toolchain, AArch32 bare-metal).
* NuttX host tools (`kconfig-frontends`, `make`, `genromfs`).
* STM32CubeProgrammer 2.17 or newer, which provides both
  `STM32_Programmer_CLI` and `STM32_SigningTool_CLI`, plus the
  `MX25UM51245G_STM32N6570-NUCLEO.stldr` external loader.

Point `STM32_PRG_PATH` at the STM32CubeProgrammer `bin` directory. The build
uses it to sign images, and the flashing scripts use it to find the programmer
and the external loader:

```sh
export STM32_PRG_PATH=$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin
```

If `STM32_PRG_PATH` is unset the build still succeeds, but the POSTBUILD step
is skipped with a warning and only the unbootable raw `nuttx.bin` is produced.

All commands below are run from the `nuttx/` directory, with `apps/` as a
sibling directory.

## Boot switches

The board selects its boot mode with the two boot switches (BOOT0/BOOT1, see
the board user manual for the silkscreen markings):

* **DEV boot mode** — required for *all* programming. The ROM parks the CPU
  with the debug port open, so the ST-LINK can drive the external loader.
* **Boot-from-flash mode** — required to *run* what you programmed.

Programming with the switches in boot-from-flash mode fails or hangs, because
the running image owns the XSPI pins.

## 1. Build and flash the bootloader

```sh
make distclean
./tools/configure.sh nucleo-n657x0-q:bl
make -j$(nproc)
```

The POSTBUILD step signs `nuttx.bin` with a load address of `0x34180000`
(LRUN) and produces **`bl-flash.bin`**.

Put the board in **DEV boot mode**, then:

```sh
./boards/arm/stm32n6/nucleo-n657x0-q/tools/xspiflash.sh \
    -i bl-flash.bin -a 0x70000000
```

The script checks the `STM2` header magic, programs through the
`MX25UM51245G_STM32N6570-NUCLEO.stldr` external loader, and verifies the
read-back.

## 2. Build and flash the application

Keep a copy of `bl-flash.bin` first if you want it — `make distclean` removes
it.

```sh
cp bl-flash.bin /tmp/            # optional
make distclean
./tools/configure.sh nucleo-n657x0-q:nsh-xspi
make -j$(nproc)
```

The POSTBUILD step signs `nuttx.bin` with `--flash-base 0x70100000` (XIP, load
address `0xffffffff`) and produces **`nuttx-flash.bin`**. `mkimage.sh` also
cross-checks the ELF against `flash.ld`: it fails if `.text` does not land at
`0x70100400` or if the entry point is outside the flash window.

Still in **DEV boot mode**:

```sh
./boards/arm/stm32n6/nucleo-n657x0-q/tools/xspiflash.sh \
    -i nuttx-flash.bin -a 0x70100000
```

## 3. Run it

1. Set the boot switches to **boot-from-flash**.
2. Open the console: USART1 on the ST-LINK VCP (`/dev/ttyACM0`), 115200 8N1.
3. Press **NRST** or power-cycle the board.

Expected output: the NuttX banner and the `nsh>` prompt. The bootloader is
silent: its `_alert()` traces compile out unless the `bl` configuration is
built with `CONFIG_DEBUG_ALERT` (`make menuconfig` -> Build Setup -> Debug
Options).

```
nsh> uname -a
nsh> free
nsh> ?
```

Only stage 2 needs to be re-flashed for application changes; the bootloader
stays in place until its own behaviour changes.

## Quick reference

```sh
# bootloader
./tools/configure.sh nucleo-n657x0-q:bl        && make -j$(nproc)
./boards/arm/stm32n6/nucleo-n657x0-q/tools/xspiflash.sh -i bl-flash.bin    -a 0x70000000

# application
./tools/configure.sh nucleo-n657x0-q:nsh-xspi  && make -j$(nproc)
./boards/arm/stm32n6/nucleo-n657x0-q/tools/xspiflash.sh -i nuttx-flash.bin -a 0x70100000
```

Useful script options (both scripts accept `-h`):

* `--mode UR` — connect under reset if `HOTPLUG` cannot take the target.
* `--no-verify` — skip read-back verification.
* `-d` — dump the STM32 header before programming.
* `--el <file>` — use a different external loader.

## Memory map

```
0x70000000  XSPI2 flash, stage 1 image (STM32 header + SRAM2 payload)
0x70100000  XSPI2 flash, stage 2 image (STM32 header)
0x70100400    stage 2 .text / .rodata, executed in place
0x34000400  AXI SRAM, base for DEV-boot (sram.ld) images
0x34180000  AXI SRAM2 bank base, ROM context area (1 KiB)
0x34180400  bootloader image; also stage 2 .data/.bss (511 KiB region)
```

## Serial board and boot contract

This contract is for **MB1940-N657X0Q-C02**, STM32N657X0H3Q (VFBGA264),
with factory wiring and no attached shields or custom connections. It selects
the second-port route for later driver work; it does **not** enable USART3,
assign it a `/dev/ttyS*` minor, or claim hardware qualification.

### Console and second-port wiring

| Port / signal | MCU pin / AF | Connector |
|---------------|--------------|-----------|
| USART1 TX (console) | PE5 / AF7 | Onboard STLINK-V3EC VCP, USB CN10 |
| USART1 RX (console) | PE6 / AF7 | Onboard STLINK-V3EC VCP, USB CN10 |
| USART3 TX (test port) | PD8 / AF7 | Arduino CN13 pin 2 (D1), also Morpho CN15 pin 35 |
| USART3 RX (test port) | PD9 / AF7 | Arduino CN13 pin 1 (D0), also Morpho CN15 pin 37 |
| Test-port common ground | GND | CN15 pin 20, or Arduino power CN5 pin 6/7 |
| USART3 RTS / CTS | Not assigned | No hardware flow control in the initial wiring contract |

Preserve the USART1 PE5/PE6 configuration and its factory ST-Link connection.
SB43 (PE5) and SB34 (PE6) expose the console nets to Morpho CN15 pins 4 and 2;
they are not second-port routing switches. Do not connect another transmitter
to the console RX net or change its bridges for a USART3 test.

The C02 schematic routes PD8/PD9 directly to CN13 and CN15 without a
solder-bridge selection, level shifter, inverter, or serial transceiver.
No bridge changes are required for the selected USART3 route. Both connector
appearances of each signal are the same net, not independent ports.

PD8/PD9 and PE5/PE6 use the main VDD I/O domain, supplied by the board's
3.3 V VDDIO rail, not the 1.8 V VDDIO3 domain used by the XSPI2 port-N pins.
Keep `PWR_SVMCR3.VDDIOVRSEL` clear and the VDD high-speed/low-voltage option
consistent with the 3.3 V supply. Use a 3.3 V logic-level peer and a common
ground. Do not connect RS-232 voltage levels directly. An inverted receiver
protocol or RS-232/RS-485 connection needs separately qualified inversion or
external interface circuitry; none is present on these test-port nets.

For later loopback, connect CN13 pin 2 to pin 1 (or CN15 pin 35 to pin 37).
Do not add this jumper until USART3 driver support and its opt-in configuration
exist. For a peer, cross board TX to peer RX and board RX to peer TX.

PD8/PD9 do not overlap the existing XSPI2 boot pins, SPI5 PE15/PG1/PG2
(or its PA3 chip select), I2C2 PB10/PB11, LEDs PG10/PG0/PG8, or the PC13
button EXTI test. The current TIM1/TIM5 counter test does not configure these
pads. Arduino shields using D0/D1, or future DCMIPP/DCMI/PSSI, FMC, LCD,
SPDIF or tamper use of PD8/PD9, conflict with this route and must not run
concurrently.

### Kernel clock and boot handoff

USART1 uses `RCC_CCIPR13.USART1SEL=6` (**hsi_div_ck**, not undivided HSI).
Startup changes only that selector field and preserves the other fields.
USART3 will use the same source through its own selector when implemented;
step 1 does not write its selector or enable/reset its peripheral.

Early console setup and the full serial driver both call
`stm32_usart_clock()` to read `RCC_HSICFGR.HSIDIV[8:7]` and derive the nominal
post-divider clock from the board's 64 MHz HSI definition:

| HSIDIV encoding | Divider | Nominal USART kernel clock with PRESC=/1 |
|-----------------|---------|------------------------------------------|
| 0 | /1 | 64 MHz |
| 1 | /2 | 32 MHz |
| 2 | /4 | 16 MHz |
| 3 | /8 | 8 MHz |

The RCC reset encoding is 0. The local clock initialization does not program
HSIDIV; an FSBL may leave a different value. Both serial initialization paths
explicitly select PRESC while UE is clear, rather than depending on FSBL
leftovers. Normal console rates use /1; very low rates may require a documented
prescaler up to /256. `CONFIG_SUPPRESS_UART_CONFIG` retains its usual
meaning: the handoff must already provide the correct clock and USART format.

Neither serial path changes the global oscillator divider. HSIDIV and the
USART kernel selector must remain stable while serial is active; changing
them requires a separate, coordinated reconfiguration. The derived frequency
is nominal, not an oscillator-tolerance or measured-baud qualification.
An FSBL must leave HSI enabled and ready, and quiesce serial/DMA transfers
before jumping to NuttX.

### CPU, peripheral and DMA access requirements

| Surface | DEV/SRAM boot | FSBL/XSPI boot |
|---------|---------------|----------------|
| CPU / register access | Secure privileged NuttX execution; secure RCC/GPIO/USART aliases | Same execution contract; FSBL must not hand off to nonsecure execution |
| USART / GPIO / RCC | CPU must be permitted to access USART1, GPIOE and RCC; later USART3 also requires GPIOD and USART3 | Inherited RIFSC/GPIO permissions and locks must allow the same accesses |
| DMA controllers | Configured GPDMA1/HPDMA1 channel pools are set secure and privileged by `stm32_dma_access_initialize()` | Same local initialization must be permitted by inherited isolation settings |
| DMA transfers | Native DMA driver sets secure source/destination attributes; HPDMA channels use CID 1 with filtering | Same attributes; FSBL RISAF/RIFSC policy must admit the selected DMA master/channel |
| Writable serial buffers | Static buffers in AXI SRAM, within the `sram.ld` region starting at `0x34000400` | Static buffers in AXI SRAM, within the `flash.ld` writable region `0x34180400..0x341fffff`, not XSPI code/rodata |
| Memory isolation / cache | RISAF must permit CPU and the selected DMA master to access buffers and descriptors; use the native DMA cache/ownership API | Inherited RISAF policy must provide the same access; XIP does not make SRAM buffers noncacheable |

The local policy does not weaken USART RIFSC permissions: RM0486 describes
non-RIF-aware peripheral reset access as nonsecure/unprivileged, which admits
the secure privileged CPU/DMA accesses used here. That reset policy is not
proof of an arbitrary FSBL's configuration. Do not silently relax isolation
or substitute nonsecure aliases when access is denied.

Ordinary WFI uses Sleep, not Stop: startup retains AXI SRAM and USART1 clocks
through the existing LPEN set aliases. USART3 will need its own APB1L enable
and LPEN handling later. This contract does not qualify Stop-mode reception.

**Target qualification still required:** on each boot path record
`RCC_HSICFGR`, `RCC_CCIPR13`, USART1 `PRESC/BRR/CR1`, and relevant LPEN
registers; confirm the resulting clock and console operation. Record the
USART/GPIO RIFSC permissions, DMA `SECCFGR/PRIVCFGR`, applicable CID settings
and RISAF buffer-region permissions before DMA qualification. Verify 3.3 V,
ground and pin continuity on the target before attaching a peer. USART3
loopback and flow-control qualification belong to the later enabled-driver
steps. These observations have not been collected by this implementation.

Sources: local RM0486 Rev 4 (chapters 3, 6, 7, 13, 14, 18/19 and 65);
[DS14791 Rev 1](https://my.avnet.com/wcm/connect/07670e5b-bab1-4163-bb60-e04c35e8bdcf/STM32N657x0-Datasheet_ebv25044.pdf?MOD=AJPERES)
(Tables 16/17, VFBGA264 and AF7);
[UM3417 Rev 3](https://www.st.com/resource/en/user_manual/um3417-stm32n6-nucleo144-board-mb1940-stmicroelectronics.pdf)
(section 7.9, Tables 12/13);
[MB1940-N657X0Q-C02 schematic](https://www.st.com/resource/en/schematic_pack/mb1940-n657x0q-c02-schematic.pdf)
(MCU, power, Morpho, Arduino and ST-Link sheets).
The ST manual and schematic were read from
[mirrored ST PDFs](https://github.com/gotree94/mcu_ml/tree/main/Day3-6N)
because direct ST downloads were unavailable.

### USART1 line configuration and power policy

Early and full setup share validated baud/format calculation and register
programming. Arithmetic uses 64-bit intermediates; oversampling by 16 is
preferred, with oversampling by 8 only when needed and representable.
FIFO mode, word length, parity, PRESC and BRR are established with UE clear.
Transmitter/receiver acknowledgement waits are bounded to 100 microseconds
per attempt. Failed reconfiguration restores the prior registers; failure
to acknowledge the restoration returns `-EIO`, not success.

With `CONFIG_SERIAL_TERMIOS`, USART1 accepts CS7/CS8 payloads, no/even/odd
parity, and one/two stop bits. RX and interrupt TX mask CS7 payloads to seven
bits; the hardware word length performs the same masking for DMA TX.
TCGETS reports the configured format and nominal requested input/output
speed, not a measured or quantization-adjusted rate. Unsupported payload
widths, zero baud (B0/hangup is not implemented), and unavailable flow pins
return `-EINVAL`; unrepresentable rates return `-ERANGE`. Configuration
suppression leaves the bootloader's format intact and rejects TCSETS with
`-ENOTSUP`.

Runtime changes exclude IRQ/debug output and reject queued TX, active DMA,
DMA requests, incomplete wire TX, pending hardware RX or an active receiver
with `-EBUSY`, without aborting transfers. The peer must be idle during the
change. NuttX's upper half implements drain/flush; the lower half preserves
the software RX ring. Frames arriving at the UE-disable boundary cannot be
guaranteed. Each FIFO character obtains fresh error status, with PE/FE/NE/ORE
cleared before RDR advances the FIFO. Errors without payload are also cleared.
The existing 256-pass ISR bound, TC-based wire completion and debug
CR-before-LF behavior are retained.

Close disables interrupts, stops/releases TX DMA, disables the USART, and
only then gates its APB clock and releases configured pins. A nonblocking
close or upper-half drain timeout can discard wire TX and logs a warning.
A failed DMA stop or USART disable leaves the clock enabled, logs the error
and makes subsequent setup return that error until successful shutdown or
reboot; it must not silently reopen an uncertain channel/peripheral.

PM prepare leaves service untouched and permits NORMAL/IDLE. Initialized
ports veto STANDBY/SLEEP with `-EBUSY` for queued/activity cases and
`-ENOTSUP` otherwise. There is no unbounded CTS/TC wait or unsafe void-notify
suspend. This gate does not alter the board's ordinary WFI/Sleep behavior
or claim Stop-mode clock retention, DMA retention or RX wakeup support.

Host checks are available with
`make -C arch/arm/src/stm32n6/tests/host check-serial`. They exercise baud
boundaries and real driver routines against mocked registers, including
termios, FIFO errors, acknowledgement failures, PM and close/reopen.
**Hardware qualification remains pending:** measure baud and receive
clock-deviation tolerance on both boot paths, inject parity/framing/noise/
overrun errors, exercise FIFO bursts and close/reopen, and verify busy/
CTS-blocked PM rejection using separately qualified flow-control wiring.
No second port or RTS/CTS board route is enabled by these changes.

## Boot-time tests

`stm32_bringup()` calls `stm32_bringup_test()` in `src/stm32_bringup_test.c`
after registering the user-LED driver. The runner executes enabled GPIO EXTI,
timer clock, DMA policy, SPI5 loopback, and SPI5 BMP280 tests in that order.
Each test retains its own `CONFIG_NUCLEO_N657X0_Q_*` switch. Failures are
logged without preventing later tests or normal board bring-up; with no
tests enabled, the runner does nothing.

## I2C2 NSH configuration

The dedicated `i2c` configuration enables STM32N6 I2C2, the board's opt-in
PB10/PB11 bring-up, `/dev/i2c2` registration through `CONFIG_I2C_DRIVER`, and
NuttX's `i2c` NSH tool:

```sh
./tools/configure.sh nucleo-n657x0-q:i2c
make -j$(nproc)
./boards/arm/stm32n6/nucleo-n657x0-q/tools/sramload.sh
```

**This configuration is not yet runtime-qualified.** The I2C2 rise/fall timing
inputs in `boards/arm/stm32n6/nucleo-n657x0-q/include/board.h` remain
unmeasured and zero, so the driver deliberately rejects timing setup.
Bring-up logs the initialization failure and does not register `/dev/i2c2`
until qualified board timing values are supplied. Do not substitute guessed
values.

PB10/PB11 are the configured MCU pin route only; connector mapping, I/O
voltage, external pull-ups, and bus capacitance have not been qualified here.
Verify those electrical details against the board documentation and hardware
before connecting or probing a target.

After timing is qualified and a compatible device is connected, use `i2c bus`
to check the registered bus. For a device whose datasheet documents a
register-read transaction, use `i2c get -b 2 -f 100000 -n -a ADDRESS -r REGISTER`, replacing
`ADDRESS` and `REGISTER` with that device's documented 7-bit address and
register. The combined register/read operation uses a repeated START.
Do not use `-s`: the current tool submits the register write alone with
NOSTOP, which the N6 driver rejects as a final NOSTOP. Clients needing
separate STOP/START transactions must submit ordinary messages without
NOSTOP. Use the device's documented transaction requirements.

Avoid broad `i2c dev` scans: the tool's default address probe performs a
one-byte read, which can have device-specific side effects. Do not issue
`i2c set` unless the target and register are known and the write is safe.
Physical SCL/SDA waveforms, one real sensor's identification and repeated
reads, and 400 kHz operation remain to be verified on hardware.

## GPIO external interrupts

EXTI lines 0-15 are shared by GPIO port: for example, PA3 and PB3 both use
EXTI3, so only one port can own a given line. The first GPIO port configured
for an EXTI line retains that line until reboot; a request for the same line
from another port fails with `-EBUSY`.

### Blue user button EXTI hardware test

Build the `nsh-test` configuration and load it in DEV boot mode:

```sh
./tools/configure.sh nucleo-n657x0-q:nsh-test
make -j$(nproc)
./boards/arm/stm32n6/nucleo-n657x0-q/tools/sramload.sh
```

With `CONFIG_NUCLEO_N657X0_Q_GPIO_EXTI_TEST`, the test configures the blue
user button on PC13 (EXTI13) as an active-high input with an internal
pull-down. During board bring-up, press the button within three seconds of
the console prompt. A detected press is logged; otherwise the test returns
`-ETIMEDOUT` and bring-up logs a warning and continues. Brief presses are
latched until the polling task observes them. The blue user LED (LD7) turns
on while the button is pressed and off when it is released, including after
the wait completes. This configuration uses the user-LED lower half instead
of `CONFIG_ARCH_LEDS`; no external jumper is needed.

## Troubleshooting

**`STM32_PRG_PATH is not set`** — export it as shown above; without it the
build never produces `bl-flash.bin` / `nuttx-flash.bin`.

**`'nuttx-signed.bin' not found`** — `xspiflash.sh` defaults to that name;
pass `-i bl-flash.bin` or `-i nuttx-flash.bin` explicitly.

**`does not start with the STM32 header magic ("STM2")`** — you are pointing at
the raw `nuttx.bin`. Use the signed output from POSTBUILD, or run
`tools/mkimage.sh` manually.

**`Error: No STM32 target found`, or programming hangs** — the board is not in
DEV boot mode, or the debug session is stale. Re-check the switches, then try
`--mode UR`.

**Nothing on the console after reset** — verify the boot switches are back in
boot-from-flash mode, and that both images were programmed at the right
addresses; the bootloader jumps unconditionally to `0x70100400`.

**Hard fault in the idle task / lockup after the banner** — a symptom of the
XSPI2 clocks being gated in CSLEEP. The bootloader sets the XSPI2/XSPIM
sleep-mode clock enables via `RCC_AHB5LPENSR`; check that the bootloader
actually ran and was not bypassed.

**Images larger than 16 MiB** — the bootloader replays the boot ROM's 24-bit
address read transaction, which only reaches the first 16 MiB of the 64 MiB
device. Larger images need 4-byte addressing and a matching read opcode in
`src/stm32_bootloader.c`.
```
