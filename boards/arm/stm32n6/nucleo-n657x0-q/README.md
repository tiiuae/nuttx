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
register-read transaction, use `i2c get -b 2 -a ADDRESS -r REGISTER`, replacing
`ADDRESS` and `REGISTER` with that device's documented 7-bit address and
register. The default combined register/read operation uses a repeated START;
the `-s` option instead splits it into separate transfers with a STOP between
them. Use the device's documented transaction requirements.

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
