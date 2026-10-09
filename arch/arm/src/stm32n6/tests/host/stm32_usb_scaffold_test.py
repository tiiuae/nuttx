#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Exercise N6 FIFO USB device APIs, MMIO faults, and configuration guards."""

import os
import pathlib
import re
import shlex
import shutil
import subprocess


def without_includes(path):
    return re.sub(r"^#include[^\n]*\n", "", path.read_text(), flags=re.M)


def main():
    directory = pathlib.Path(__file__).resolve().parent
    chip = directory.parents[1]
    root = chip.parents[3]
    headers = [
        chip / "hardware/stm32n6xxx_memorymap.h",
        chip / "hardware/stm32n6xxx_rcc.h",
        chip / "hardware/stm32n6xxx_pwr.h",
        root / "arch/arm/include/stm32n6/stm32n6xx_irq.h",
        chip / "stm32_otg.h",
        chip / "hardware/stm32n6xxx_otg.h",
        chip / "hardware/stm32n6xxx_usbphyc.h",
        root / "include/nuttx/usb/usb.h",
        root / "include/nuttx/usb/usbdev.h",
        root / "include/nuttx/usb/usbdev_trace.h",
        root / "boards/arm/stm32n6/nucleo-n657x0-q/include/board.h",
    ]
    source = (directory / "stm32_usb_scaffold_test.c").read_text()
    source = source.replace(
        "/* USB_HEADERS */", "\n".join(without_includes(path) for path in headers)
    )
    source = source.replace(
        "/* USB_DRIVER */", without_includes(chip / "stm32_otgdev.c")
    )
    compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
    common = ["CONFIG_STM32_STM32N6XXXX", "CONFIG_STM32_N6_OTGDEV", "CONFIG_USBDEV"]
    work = directory / f".usb-host-{os.getpid()}"
    work.mkdir()
    try:
        executable = work / "usb-scaffold"
        command = compiler + [
            "-x",
            "c",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wsign-compare",
            "-o",
            str(executable),
            "-",
        ]
        if os.environ.get("HOST_USB_SANITIZE"):
            command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        for port in (1, 2):
            for mode in ("fs", "hs", "custom", "nonsecure"):
                defines = common + [f"CONFIG_STM32_N6_OTG{port}"]
                if mode != "hs":
                    defines += ["CONFIG_STM32_N6_OTGDEV_FS"]
                else:
                    defines += ["CONFIG_USBDEV_DUALSPEED", "TEST_USB_BOARDHOOK"]
                if mode != "fs":
                    defines += ["CONFIG_USBDEV_TRACE", "CONFIG_USBDEV_TRACE_STRINGS"]
                if mode == "custom":
                    defines += [
                        "TEST_USB_CUSTOM_FIFO",
                        "CONFIG_USBDEV_EP0_TXFIFO_SIZE=64",
                        "CONFIG_USBDEV_EP1_TXFIFO_SIZE=65",
                        "CONFIG_USBDEV_EP2_TXFIFO_SIZE=512",
                        "CONFIG_USBDEV_EP8_TXFIFO_SIZE=128",
                    ]
                if mode == "nonsecure":
                    defines += ["CONFIG_ARCH_TRUSTZONE_NONSECURE"]
                subprocess.run(
                    command + ["-D" + name for name in defines],
                    input=source,
                    text=True,
                    check=True,
                )
                subprocess.run([str(executable)], check=True)
                print(f"USB{port}, mode={mode}: PASS")

        invalid = [
            ([], "Select exactly one"),
            (["CONFIG_STM32_N6_OTG1", "CONFIG_STM32_N6_OTG2"], "Select exactly one"),
        ]
        for feature in (
            "USBHOST",
            "USBDEV_DMA",
            "USBDEV_ISOCHRONOUS",
            "USBDEV_SUPERSPEED",
            "USBDEV_COMPOSITE",
        ):
            invalid.append(
                (["CONFIG_STM32_N6_OTG1", "CONFIG_" + feature], "single FIFO-mode")
            )
        for setting, message in (
            ("USBDEV_EP8_TXFIFO_SIZE=4096", "exceeds 4 KiB"),
            ("USBDEV_EP0_TXFIFO_SIZE=0", "Invalid STM32N6"),
            ("USBDEV_EP1_TXFIFO_SIZE=-1", "Invalid STM32N6"),
            ("USBDEV_EP2_TXFIFO_SIZE=32", "Invalid STM32N6"),
        ):
            invalid.append((["CONFIG_STM32_N6_OTG1", "CONFIG_" + setting], message))
        invalid.append(
            (
                [
                    "CONFIG_STM32_N6_OTG1",
                    "CONFIG_STM32_N6_OTGDEV_FS",
                    "CONFIG_USBDEV_DUALSPEED",
                ],
                "Forced full-speed",
            )
        )
        for defines, message in invalid:
            result = subprocess.run(
                command + ["-D" + name for name in common + defines],
                input=source,
                text=True,
                capture_output=True,
                check=False,
            )
            assert result.returncode != 0, defines
            assert message in result.stderr, result.stderr
        print(f"Rejected {len(invalid)} unsupported configurations: PASS")
    finally:
        shutil.rmtree(work)


if __name__ == "__main__":
    main()
