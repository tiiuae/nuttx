#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Compile the real N6 serial routines against host register/upper-half mocks."""

import os
import pathlib
import re
import shlex
import subprocess
import tempfile


def extract(text, declaration):
    match = re.search(declaration + r"\s*\{", text)
    if match is None:
        raise ValueError(f"Declaration not found: {declaration}")

    # Ignore braces in comments and strings without changing source offsets.
    tokens = re.sub(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda item: " " * len(item.group()),
        text,
        flags=re.DOTALL,
    )
    end = match.end() - 1
    depth = 1
    while depth:
        end += 1
        depth += (tokens[end] == "{") - (tokens[end] == "}")
    return text[match.start() : end + 1]


def function(text, name):
    return extract(
        text,
        r"(?:static\s+(?:inline\s+)?)?"
        r"(?:void|int|bool|uint32_t)\s+" + name + r"\([^;{]*\)",
    )


def check_instances(directory, chip, nuttx, compiler, temporary):
    ports = ["USART1", "USART2", "USART3", "UART4", "UART5",
             "USART6", "UART7", "UART8", "UART9", "USART10"]
    low = (chip / "stm32_lowputc.c").read_text()
    serial = (chip / "stm32_serial.c").read_text()
    uart = (chip / "stm32_uart.h").read_text()
    source = (directory / "stm32_serial_instances_test.c").read_text()
    start = uart.index("/* Sanity checks */")
    end = uart.index("#define USART_CR1_USED_INTS", start)
    source = source.replace("/* CONSOLE_SELECTION */", uart[start:end])
    source = source.replace("/* DRIVER_TYPES */",
                            extract(uart, r"struct stm32_usart_s") + ";\n" +
                            extract(serial, r"struct stm32_serial_s"))
    source = source.replace("/* HARDWARE_INSTANCES */",
                            extract(low, r"const struct stm32_usart_s\s+"
                                    r"g_usart_config\[[^]]+\]\s*=") + ";")
    start = serial.index("/* Each enabled hardware instance")
    end = serial.index("#ifdef CONFIG_PM\nstatic struct pm_callback_s", start)
    source = source.replace("/* DRIVER_INSTANCES */", serial[start:end])
    low_names = ["stm32_usart_waitack", "stm32_usart_clock",
                 "stm32_usart_setclock", "stm32_usart_initialize",
                 "stm32_usart_disable", "stm32_usart_configure"]
    serial_names = ["stm32serial_getreg", "stm32serial_putreg",
                    "stm32serial_setusartint", "stm32serial_disableusartint",
                    "stm32serial_setapbclock", "stm32serial_setup",
                    "stm32serial_shutdown", "stm32serial_send",
                    "stm32serial_receive", "stm32serial_register",
                    "arm_serialinit"]
    source = source.replace("/* DRIVER_ROUTINES */",
                            "\n".join(function(low, name) for name in low_names) +
                            "\n" + "\n".join(function(serial, name)
                                            for name in serial_names))
    source = source.replace("/* ACK_TIMEOUT */",
                            re.search(r"^#define USART_ACK_TIMEOUT_US .*",
                                      low, re.MULTILINE).group())
    variants = [
        ("all-console10", ports, "USART10", []),
        ("all-console10-fixed-order", ports, "USART10",
         ["CONFIG_STM32_SERIAL_DISABLE_REORDERING"]),
        ("all-no-console", ports, None, []),
        ("sparse-console3-dma1", ["USART1", "USART3"], "USART3",
         ["STM32_USART1_TXDMA"]),
        ("sparse-console10", ["USART1", "USART10"], "USART10", []),
    ]
    for name, selected, console, options in variants:
        executable = pathlib.Path(temporary) / name
        definitions = ["CONFIG_STM32_STM32N6XXXX", "CONFIG_SERIAL_TERMIOS",
                       "STM32_IRQ_FIRST=16"] + options
        for index, port in enumerate(selected, 1):
            definitions += [f"CONFIG_STM32_{port}",
                            f"CONFIG_STM32_{port}_SERIALDRIVER",
                            f"CONFIG_{port}_BAUD=115200", f"CONFIG_{port}_BITS=8",
                            f"CONFIG_{port}_PARITY=0", f"CONFIG_{port}_2STOP=0",
                            f"CONFIG_{port}_RXBUFSIZE=32",
                            f"CONFIG_{port}_TXBUFSIZE=32",
                            f"GPIO_{port}_TX={100 + 2 * index}",
                            f"GPIO_{port}_RX={101 + 2 * index}"]
        if console:
            definitions += [f"CONFIG_{console}_SERIAL_CONSOLE"]
        subprocess.run(compiler + ["-x", "c", "-std=c11", "-Wall", "-Wextra",
                                   "-Werror", "-I" + str(directory / "include"),
                                   "-I" + str(chip),
                                   "-idirafter",
                                   str(nuttx / "arch/arm/include"),
                                   "-o", str(executable), "-"] +
                       ["-D" + item for item in definitions],
                       input=source, text=True, check=True)
        subprocess.run([str(executable)], check=True)
        print(f"STM32N6 serial instance tests passed: {name}", flush=True)


def main():
    directory = pathlib.Path(__file__).resolve().parent
    chip = directory.parent.parent
    nuttx = chip.parents[3]
    lowputc = (chip / "stm32_lowputc.c").read_text()
    serial = (chip / "stm32_serial.c").read_text()
    uart = (chip / "stm32_uart.h").read_text()
    routines = [
        function(lowputc, name)
        for name in (
            "stm32_usart_waitack",
            "stm32_usart_clock",
            "stm32_usart_setclock",
            "stm32_usart_initialize",
            "stm32_usart_disable",
            "stm32_usart_configure",
        )
    ]
    routines += [
        function(serial, name)
        for name in (
            "stm32serial_getreg",
            "stm32serial_putreg",
            "stm32serial_setusartint",
            "stm32serial_restoreusartint",
            "stm32serial_disableusartint",
            "stm32serial_busy",
            "stm32serial_setapbclock",
            "stm32serial_setup",
            "stm32serial_shutdown",
            "stm32serial_interrupt",
            "stm32serial_ioctl",
            "stm32serial_receive",
            "stm32serial_rxint",
            "stm32serial_rxavailable",
            "stm32serial_send",
            "stm32serial_txready",
            "stm32serial_txempty",
            "stm32serial_pmprepare",
            "up_putc",
        )
    ]
    source = (directory / "stm32_serial_driver_test.c").read_text()
    source = source.replace(
        "/* DRIVER_TYPES */",
        extract(uart, r"struct stm32_usart_s") + ";\n" +
        extract(serial, r"struct stm32_serial_s"),
    )
    source = source.replace("/* DRIVER_ROUTINES */", "\n".join(routines))
    source = source.replace(
        "/* ACK_TIMEOUT */",
        re.search(r"^#define USART_ACK_TIMEOUT_US .*", lowputc, re.MULTILINE).group(),
    )
    compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
    variants = (
        ("dma-flow", ["STM32_USART1_TXDMA", "CONFIG_SERIAL_IFLOWCONTROL",
                      "CONFIG_SERIAL_OFLOWCONTROL"]),
        ("irq-flow", ["CONFIG_SERIAL_IFLOWCONTROL", "CONFIG_SERIAL_OFLOWCONTROL"]),
        ("dma-no-flow", ["STM32_USART1_TXDMA"]),
        ("software-rts", ["CONFIG_SERIAL_IFLOWCONTROL",
                          "CONFIG_SERIAL_OFLOWCONTROL",
                          "CONFIG_STM32_FLOWCONTROL_BROKEN"]),
        ("suppressed", ["STM32_USART1_TXDMA", "CONFIG_SUPPRESS_UART_CONFIG"]),
    )
    with tempfile.TemporaryDirectory(prefix="stm32n6-serial-") as temporary:
        for name, options in variants:
            executable = pathlib.Path(temporary) / name
            subprocess.run(
                compiler + [
                    "-x", "c", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter",
                    "-I" + str(directory / "include"), "-I" + str(chip),
                    '-DNUTTX_TERMIOS_HEADER="' + str(nuttx / "include/termios.h") + '"',
                    "-o", str(executable), "-",
                ] + ["-D" + option for option in options],
                input=source,
                text=True,
                check=True,
            )
            subprocess.run([str(executable)], check=True)
            print(f"STM32N6 serial register tests passed: {name}", flush=True)
        check_instances(directory, chip, nuttx, compiler, temporary)


if __name__ == "__main__":
    main()
