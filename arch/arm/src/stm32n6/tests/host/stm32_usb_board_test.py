#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Exercise the actual CN8 board policy and shared I2C initialization."""

import os
import pathlib
import re
import shlex
import subprocess
import tempfile


def without_includes(path):
    return re.sub(r"^#include[^\n]*\n", "", path.read_text(), flags=re.M)


def main():
    directory = pathlib.Path(__file__).resolve().parent
    chip = directory.parents[1]
    root = chip.parents[3]
    board = root / "boards/arm/stm32n6/nucleo-n657x0-q/src"
    headers = [
        chip / "hardware/stm32n6xxx_memorymap.h",
        chip / "hardware/stm32n6xxx_rcc.h",
        chip / "hardware/stm32n6xxx_ucpd.h",
    ]
    source = (directory / "stm32_usb_board_test.c").read_text()
    source = source.replace(
        "/* BOARD_HEADERS */", "\n".join(without_includes(path) for path in headers)
    )
    source = source.replace(
        "/* BOARD_I2C */", without_includes(board / "stm32_i2c_board.c")
    )
    source = source.replace(
        "/* BOARD_POLICY */", without_includes(board / "stm32_usb.c")
    )
    compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
    defines = [
        "CONFIG_STM32_STM32N6XXXX",
        "CONFIG_NUCLEO_N657X0_Q_USBDEV",
        "CONFIG_USBDEV_SELFPOWERED",
        "CONFIG_USBMONITOR",
    ]
    with tempfile.TemporaryDirectory(prefix="usb-board-", dir=directory) as work:
        command = compiler + [
            "-x",
            "c",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-o",
            str(pathlib.Path(work) / "board"),
            "-",
        ]
        if os.environ.get("HOST_USB_SANITIZE"):
            command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        for qualified in (False, True):
            for raw_bus in (False, True):
                flags = defines.copy()
                if qualified:
                    flags += ["CONFIG_NUCLEO_N657X0_Q_USBDEV_QUALIFIED"]
                if raw_bus:
                    flags += ["CONFIG_I2C_DRIVER"]
                subprocess.run(
                    command + ["-D" + flag for flag in flags],
                    input=source,
                    text=True,
                    check=True,
                )
                subprocess.run([command[command.index("-o") + 1]], check=True)
                print(f"qualified={qualified}, raw_bus={raw_bus}: PASS")
        for owner in (
            "CDCACM_CONSOLE",
            "CDCACM_COMPOSITE",
            "SYSTEM_CDCACM",
            "EXAMPLES_USBSERIAL",
            "USBDEV_REMOTEWAKEUP",
        ):
            result = subprocess.run(
                command + ["-D" + flag for flag in defines + ["CONFIG_" + owner]],
                input=source,
                text=True,
                capture_output=True,
                check=False,
            )
            assert result.returncode != 0, owner
            assert "#error" in result.stderr, result.stderr
        print("Duplicate CDC owners and remote wakeup rejected: PASS")

        headers = [
            root / "include/nuttx/usb/usb.h",
            root / "include/nuttx/usb/usbdev.h",
            root / "include/nuttx/usb/cdc.h",
            root / "include/nuttx/usb/cdcacm.h",
            root / "drivers/usbdev/cdcacm.h",
        ]
        # Match NuttX's system-header treatment of the existing CDC header,
        # which has unrelated duplicate macro definitions.
        (pathlib.Path(work) / "cdc.h").write_text(
            without_includes(root / "include/nuttx/usb/cdc.h")
        )
        source = (directory / "stm32_usb_cdc_desc_test.c").read_text()
        source = source.replace(
            "/* CDC_HEADERS */",
            "\n".join(
                (
                    "#include <cdc.h>"
                    if path == root / "include/nuttx/usb/cdc.h"
                    else without_includes(path)
                )
                for path in headers
            ),
        )
        source = source.replace(
            "/* CDC_DESCRIPTORS */",
            without_includes(root / "drivers/usbdev/cdcacm_desc.c"),
        )
        flags = [
            "CONFIG_CDCACM_HAVE_EPINTIN",
            "CONFIG_USBDEV_SELFPOWERED",
            "CONFIG_CDCACM_VENDORID=0x16c0",
            "CONFIG_CDCACM_PRODUCTID=0x05e1",
            'CONFIG_CDCACM_PRODUCTSTR="Nucleo N657 CDC ACM"',
            "CONFIG_CDCACM_EPINTIN_FSSIZE=16",
            "CONFIG_CDCACM_EPINTIN_HSSIZE=16",
        ]
        for dual in (False, True):
            speed_flags = flags + (["CONFIG_USBDEV_DUALSPEED"] if dual else [])
            subprocess.run(
                command + ["-isystem", work] + ["-D" + flag for flag in speed_flags],
                input=source,
                text=True,
                check=True,
            )
            subprocess.run([command[command.index("-o") + 1]], check=True)
            print(f"CDC descriptors, dualspeed={dual}: PASS")


if __name__ == "__main__":
    main()
