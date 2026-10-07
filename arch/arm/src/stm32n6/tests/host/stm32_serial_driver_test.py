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
        r"(?:void|int|bool|uint32_t|DMA_HANDLE)\s+" + name + r"\([^;{]*\)",
    )


def check_dma_abort(directory, chip, compiler, temporary):
    dma = (chip / "stm32_dma.c").read_text()
    source = (directory / "stm32_dma_abort_test.c").read_text()
    source = source.replace("/* DMA_TYPES */",
                            extract(dma, r"enum stm32_dma_transfer_state_e") +
                            ";\n" +
                            extract(dma, r"struct stm32_dma_channel_s"))
    start = dma.index("#define STM32_DMA_INTERRUPT_MASK")
    end = dma.index("\n\n", start)
    source = source.replace("/* DMA_INTERRUPTS */", dma[start:end])
    source = source.replace("/* DMA_ROUTINES */",
                            "\n".join(function(dma, name) for name in (
                                "stm32_dma_allocated", "stm32_dma_recovering",
                                "stm32_dma_in_flight", "stm32_dma_busy",
                                "stm32_dma_controller_enabled",
                                "stm32_dma_request_valid",
                                "stm32_dma_check_config", "stm32_dmafree",
                                "stm32_dmasetup", "stm32_dmallibuild",
                                "stm32_dmacallback", "stm32_dma_interrupt",
                                "stm32_dma_initialize_controller",
                                "stm32_dmachannel",
                                "stm32_dmastart", "stm32_dma_stop",
                                "stm32_dmastop", "stm32_dmaabort",
                                "stm32_dmastatus")))
    executable = pathlib.Path(temporary) / "dma-abort"
    # Static descriptors must fit the driver's 32-bit DMA address space.
    # Request validation compares an enum with a signed direction sentinel.
    subprocess.run(compiler + ["-x", "c", "-std=c11", "-Wall", "-Wextra",
                               "-Werror", "-Wno-sign-compare",
                               "-fno-pie", "-no-pie",
                               "-I" + str(directory / "include"),
                               "-I" + str(chip), "-o", str(executable), "-"],
                   input=source, text=True, check=True)
    subprocess.run([str(executable)], check=True)


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
                            "#ifdef STM32_SERIAL_TXDMA\n" +
                            extract(serial, r"enum stm32_serial_txdma_state_e") +
                            ";\n#endif\n" +
                            extract(serial, r"struct stm32_serial_s"))
    source = source.replace("/* HARDWARE_INSTANCES */",
                            extract(low, r"const struct stm32_usart_s\s+"
                                    r"g_usart_config\[[^]]+\]\s*=") + ";")
    start = serial.index("/* Each enabled hardware instance")
    end = serial.index("#ifdef CONFIG_PM\nstatic struct pm_callback_s", start)
    source = source.replace("/* DRIVER_INSTANCES */", serial[start:end])
    low_names = ["stm32_usart_waitack", "stm32_usart_clock",
                 "stm32_usart_setclock", "stm32_usart_initialize",
                 "stm32_usart_disable", "stm32_usart_flowcontrol",
                 "stm32_usart_apply",
                 "stm32_usart_configure"]
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
    source = source.replace("/* DMA_ABORT */",
                            function(serial, "stm32serial_dmaabort"))
    source = source.replace("/* ACK_TIMEOUT */",
                            re.search(r"^#define USART_ACK_TIMEOUT_US .*",
                                      low, re.MULTILINE).group())
    variants = [
        ("all-console10", ports, "USART10", []),
        ("all-console10-fixed-order", ports, "USART10",
         ["CONFIG_STM32_SERIAL_DISABLE_REORDERING"]),
        ("all-no-console", ports, None, []),
        ("sparse-console3-dma1", ["USART1", "USART3"], "USART3",
         ["STM32_SERIAL_TXDMA", "CONFIG_USART1_TXDMA", "TEST_DMA_PORT_MASK=1"]),
        ("all-dma", ports, "USART1",
         ["STM32_SERIAL_TXDMA", "TEST_DMA_PORT_MASK=1023"] +
         [f"CONFIG_{port}_TXDMA" for port in ports]),
        ("sparse-dma9-dma10", ["USART1", "USART3", "UART9", "USART10"], "USART1",
         ["STM32_SERIAL_TXDMA", "CONFIG_UART9_TXDMA", "CONFIG_USART10_TXDMA",
          "TEST_DMA_PORT_MASK=768"]),
        ("sparse-console10", ["USART1", "USART10"], "USART10", []),
    ]
    for name, selected, console, options in variants:
        executable = pathlib.Path(temporary) / name
        definitions = ["CONFIG_STM32_STM32N6XXXX", "CONFIG_SERIAL_TERMIOS",
                       "STM32_IRQ_FIRST=16"] + options
        if "STM32_SERIAL_TXDMA" in options:
            definitions.append("CONFIG_STM32_GPDMA1")
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
            "stm32_usart_flowcontrol",
            "stm32_usart_apply",
            "stm32_usart_configure",
            "stm32_usart_setmode",
        )
    ]
    for name in (
        "stm32serial_getreg",
        "stm32serial_putreg",
        "stm32serial_setusartint",
        "stm32serial_restoreusartint",
        "stm32serial_disableusartint",
        "stm32serial_busy",
        "stm32serial_setapbclock",
        "stm32serial_setup",
        "stm32serial_setflow",
        "stm32serial_shutdown",
        "stm32serial_detach",
        "stm32serial_interrupt",
        "stm32serial_setmode",
        "stm32serial_ioctl",
        "stm32serial_receive",
        "stm32serial_rxint",
        "stm32serial_rxavailable",
        "stm32serial_rxflowcontrol",
        "stm32serial_send",
        "stm32serial_txready",
        "stm32serial_txint",
        "stm32serial_txempty",
        "stm32serial_pmprepare",
        "up_putc",
    ):
        routine = function(serial, name)
        if name == "stm32serial_rxflowcontrol":
            routine = "#ifdef CONFIG_SERIAL_IFLOWCONTROL\n" + routine + "\n#endif"
        elif name == "stm32serial_setflow":
            routine = ("#if defined(CONFIG_SERIAL_IFLOWCONTROL) && "
                       "!defined(CONFIG_SUPPRESS_UART_CONFIG)\n" +
                       routine + "\n#endif")
        elif name == "stm32serial_setmode":
            routine = ("#if defined(CONFIG_STM32_USART_INVERT) || "
                       "defined(CONFIG_STM32_USART_SINGLEWIRE)\n" +
                       routine + "\n#endif")
        routines.append(routine)
    dma_names = (
        "stm32serial_dmainitialize", "stm32serial_dmasend",
        "stm32serial_dmatxavail", "stm32serial_dmaabort",
        "stm32serial_dmafallback", "stm32serial_dmatxcallback",
        "stm32serial_debugsend",
    )
    routines.append("#ifdef STM32_SERIAL_TXDMA\n" +
                    "\n".join(function(serial, name) for name in dma_names) +
                    "\n#endif")
    dma = (nuttx / "drivers/serial/serial_dma.c").read_text()
    routines.append("#ifdef STM32_SERIAL_TXDMA\n" +
                    function(dma, "uart_xmitchars_dma") + "\n" +
                    function(dma, "uart_xmitchars_done") + "\n#endif")
    source = (directory / "stm32_serial_driver_test.c").read_text()
    tioctl = (nuttx / "include/nuttx/serial/tioctl.h").read_text()
    source = source.replace(
        "/* IOCTL_FLAGS */",
        "\n".join(re.findall(
            r"^#\s*define SER_(?:SINGLEWIRE|INVERT)_.*", tioctl, re.MULTILINE)),
    )
    gpio = (chip / "stm32_gpio.h").read_text()
    source = source.replace("/* GPIO_DEFINITIONS */",
                            gpio[gpio.index("/* Mode:"):
                                 gpio.index("/* External interrupt")])
    source = source.replace(
        "/* DRIVER_TYPES */",
        extract(uart, r"struct stm32_usart_s") + ";\n" +
        "#ifdef STM32_SERIAL_TXDMA\n" +
        extract(serial, r"enum stm32_serial_txdma_state_e") + ";\n#endif\n" +
        extract(serial, r"struct stm32_serial_s"),
    )
    source = source.replace("/* DRIVER_ROUTINES */", "\n".join(routines))
    source = source.replace(
        "/* ACK_TIMEOUT */",
        re.search(r"^#define USART_ACK_TIMEOUT_US .*", lowputc, re.MULTILINE).group(),
    )
    compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
    variants = (
        ("dma-flow", ["STM32_SERIAL_TXDMA", "CONFIG_SERIAL_IFLOWCONTROL",
                      "CONFIG_SERIAL_OFLOWCONTROL"]),
        ("irq-flow", ["CONFIG_SERIAL_IFLOWCONTROL", "CONFIG_SERIAL_OFLOWCONTROL"]),
        ("input-flow", ["CONFIG_SERIAL_IFLOWCONTROL"]),
        ("output-flow", ["CONFIG_SERIAL_OFLOWCONTROL"]),
        ("dma-no-flow", ["STM32_SERIAL_TXDMA"]),
        ("software-rts", ["CONFIG_SERIAL_IFLOWCONTROL",
                          "CONFIG_SERIAL_OFLOWCONTROL",
                          "CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS",
                          "CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK=90",
                          "CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK=10",
                          "CONFIG_STM32_FLOWCONTROL_BROKEN"]),
        ("rc-modes", ["CONFIG_STM32_USART_INVERT",
                      "CONFIG_STM32_USART_SINGLEWIRE"]),
        ("invert-only", ["CONFIG_STM32_USART_INVERT"]),
        ("singlewire-only", ["CONFIG_STM32_USART_SINGLEWIRE"]),
        ("rc-flow", ["STM32_SERIAL_TXDMA", "CONFIG_STM32_USART_INVERT",
                     "CONFIG_STM32_USART_SINGLEWIRE",
                     "CONFIG_SERIAL_IFLOWCONTROL",
                     "CONFIG_SERIAL_OFLOWCONTROL"]),
        ("rc-software-rts", ["CONFIG_STM32_USART_INVERT",
                             "CONFIG_STM32_USART_SINGLEWIRE",
                             "CONFIG_SERIAL_IFLOWCONTROL",
                             "CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS",
                             "CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK=90",
                             "CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK=10",
                             "CONFIG_STM32_FLOWCONTROL_BROKEN"]),
        ("rc-suppressed", ["CONFIG_STM32_USART_INVERT",
                           "CONFIG_STM32_USART_SINGLEWIRE",
                           "CONFIG_SUPPRESS_UART_CONFIG"]),
        ("suppressed", ["STM32_SERIAL_TXDMA", "CONFIG_SUPPRESS_UART_CONFIG"]),
    )
    with tempfile.TemporaryDirectory(prefix="stm32n6-serial-") as temporary:
        check_dma_abort(directory, chip, compiler, temporary)
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
