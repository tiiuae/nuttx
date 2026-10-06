#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Generate real Kconfig selections and compile N6 serial without reconfiguring."""

import os
import pathlib
import shlex
import subprocess
import tempfile


def main():
    chip = pathlib.Path(__file__).resolve().parents[2]
    root = chip.parents[3]
    arch = root / "arch/arm/src"
    if (arch / "chip").resolve() != chip:
        raise RuntimeError("Configure an STM32N6 build before running this matrix")
    ports = ["USART1", "USART2", "USART3", "UART4", "UART5",
             "USART6", "UART7", "UART8", "UART9", "USART10"]
    baseline = (root / "boards/arm/stm32n6/nucleo-n657x0-q/configs/"
                "nsh-xspi/defconfig").read_text()
    baseline = "\n".join(line for line in baseline.splitlines()
                         if not any(line.startswith("CONFIG_" + prefix)
                                    for port in ports
                                    for prefix in ("STM32_" + port, port + "_")))
    output = subprocess.check_output(
        ["make", "-s", "-C", str(arch), "TOPDIR=" + str(root), "-n",
         "stm32_lowputc.o", "stm32_serial.o", "stm32_start.o"], text=True)
    commands = [shlex.split(line) for line in output.splitlines()
                if line.startswith("arm-none-eabi-gcc ")]
    if len(commands) != 3:
        raise RuntimeError("Expected three ARM compile commands")
    variants = [(port.lower(), [port], port, []) for port in ports]
    variants += [
        ("all-console10", ports, "USART10", []),
        ("all-fixed-order", ports, "USART10",
         ["STM32_SERIAL_DISABLE_REORDERING"]),
        ("board-usart1-usart3", ["USART1", "USART3"], "USART1", []),
        ("all-pm", ports, "USART1", ["PM"]),
        ("all-suppressed", ports, "USART1", ["SUPPRESS_UART_CONFIG"]),
        ("all-no-console", ports, None, []),
        ("no-uart", [], None, []),
        ("all-rc", ports, "USART1",
         ["STM32_USART_INVERT", "STM32_USART_SINGLEWIRE"]),
        ("invert-only", ["USART1", "USART3"], "USART1",
         ["STM32_USART_INVERT"]),
        ("singlewire-only", ["USART1", "USART3"], "USART1",
         ["STM32_USART_SINGLEWIRE"]),
        ("rc-no-termios", ["USART1", "USART3"], "USART1",
         ["STM32_USART_INVERT", "STM32_USART_SINGLEWIRE", "SERIAL_TERMIOS=n"]),
        ("rc-suppressed", ["USART1", "USART3"], "USART1",
         ["STM32_USART_INVERT", "STM32_USART_SINGLEWIRE",
          "SUPPRESS_UART_CONFIG"]),
        ("flow-in", ["USART1", "USART3"], "USART1", ["USART3_IFLOWCONTROL"]),
        ("flow-out", ["USART1", "USART3"], "USART1", ["USART3_OFLOWCONTROL"]),
        ("flow-both", ports, "USART1",
         ["USART3_IFLOWCONTROL", "USART3_OFLOWCONTROL"]),
        ("flow-no-pins", ["USART1", "USART3"], "USART1",
         ["USART3_IFLOWCONTROL", "USART3_OFLOWCONTROL"]),
        ("console-flow", ["USART1", "USART3"], "USART3",
         ["USART3_IFLOWCONTROL", "USART3_OFLOWCONTROL"]),
        ("console-software-rts", ["USART1", "USART3"], "USART3",
         ["USART3_IFLOWCONTROL", "SERIAL_IFLOWCONTROL_WATERMARKS",
          "STM32_FLOWCONTROL_BROKEN"]),
        ("flow-software-rts", ["USART1", "USART3"], "USART1",
         ["USART3_IFLOWCONTROL", "SERIAL_IFLOWCONTROL_WATERMARKS",
          "STM32_FLOWCONTROL_BROKEN"]),
        ("unsupported-options", ports, "USART1",
         ["STM32_USART_BREAKS", "STM32_USART_SWAP"] +
         [port + "_RS485" for port in ports]),
    ]
    with tempfile.TemporaryDirectory(prefix="n6-serial-config-") as temporary:
        directory = pathlib.Path(temporary)
        for source in root.iterdir():
            if source.is_dir() and not source.name.startswith("."):
                (directory / source.name).symlink_to(source, target_is_directory=True)
        env = os.environ.copy()
        env.update(srctree=str(root), BINDIR=temporary,
                   APPSDIR=str(root.parent / "apps"),
                   APPSBINDIR=str(root.parent / "apps"),
                   EXTERNALDIR=str(root / "dummy"), KCONFIG_CONFIG=".config")
        for name, selected, console, options in variants:
            request = baseline + "\n"
            if "SERIAL_TERMIOS=n" not in options:
                request += "CONFIG_SERIAL_TERMIOS=y\n"
            request += "".join(f"CONFIG_STM32_{port}=y\n" for port in selected)
            request += (f"CONFIG_{console}_SERIAL_CONSOLE=y\n" if console else
                        "CONFIG_NO_SERIAL_CONSOLE=y\n")
            request += "".join("CONFIG_" + (option if "=" in option else
                                           option + "=y") + "\n"
                               for option in options)
            seed = directory / "seed"
            seed.write_text(request)
            subprocess.run(["kconfig-conf", "--defconfig=" + str(seed),
                            str(root / "Kconfig")], cwd=directory, env=env,
                           stdout=subprocess.DEVNULL, check=True)
            values = dict(line[7:].split("=", 1)
                          for line in (directory / ".config").read_text().splitlines()
                          if line.startswith("CONFIG_"))
            for port in selected:
                for symbol in ("STM32_" + port, "STM32_" + port + "_SERIALDRIVER",
                               port + "_SERIALDRIVER"):
                    assert values.get(symbol) == "y", (name, symbol)
            if console:
                assert values.get(console + "_SERIAL_CONSOLE") == "y"
            if selected and "SERIAL_TERMIOS=n" not in options:
                assert values.get("SERIAL_TERMIOS") == "y"
            if "SERIAL_TERMIOS=n" in options:
                assert values.get("SERIAL_TERMIOS") != "y"
            for option in options:
                if option in ("STM32_USART_INVERT", "STM32_USART_SINGLEWIRE",
                              "STM32_FLOWCONTROL_BROKEN") or \
                        option.endswith(("IFLOWCONTROL", "OFLOWCONTROL")):
                    assert values.get(option) == "y", (name, option)
            for option in ["STM32_USART_BREAKS", "STM32_USART_SWAP"] + [
                    port + "_RS485" for port in ports]:
                assert values.get(option) != "y", (name, option)
            assert not any(value == "y" for key, value in values.items()
                           if key.startswith("STM32_HAVE_IP_USART") or
                           key.startswith("STM32_HAVE_LPUART"))
            symbols = {"SERIAL_TERMIOS", "SERIAL_TXDMA", "SERIAL_RXDMA",
                       "SERIAL_IFLOWCONTROL", "SERIAL_OFLOWCONTROL", "PM",
                       "SUPPRESS_UART_CONFIG", "STM32_SERIAL_DISABLE_REORDERING",
                       "STM32_USART_INVERT", "STM32_USART_SINGLEWIRE",
                       "STM32_FLOWCONTROL_BROKEN",
                       "SERIAL_IFLOWCONTROL_WATERMARKS",
                       "SERIAL_IFLOWCONTROL_UPPER_WATERMARK",
                       "SERIAL_IFLOWCONTROL_LOWER_WATERMARK"}
            for port in ports:
                symbols.update(("STM32_" + port, "STM32_" + port + "_SERIALDRIVER"))
                symbols.update(port + "_" + suffix for suffix in (
                    "SERIALDRIVER", "SERIAL_CONSOLE", "BAUD", "BITS", "PARITY",
                    "2STOP", "RXBUFSIZE", "TXBUFSIZE", "RXDMA", "TXDMA",
                    "IFLOWCONTROL", "OFLOWCONTROL", "UNCONFIG_RX_ON_CLOSE",
                    "UNCONFIG_TX_ON_CLOSE"))
            header = "#include <nuttx/config.h>\n"
            for symbol in sorted(symbols):
                header += "#undef CONFIG_" + symbol + "\n"
                value = values.get(symbol)
                if value is not None:
                    header += "#define CONFIG_" + symbol + " " + (
                        "1" if value == "y" else value) + "\n"
            for port in selected:
                if port not in ("USART1", "USART3"):
                    # Compile-only bindings; not physical alternate-function routes.
                    header += f"#define GPIO_{port}_TX GPIO_USART1_TX\n"
                    header += f"#define GPIO_{port}_RX GPIO_USART1_RX\n"
                if name != "flow-no-pins":
                    if values.get("SERIAL_IFLOWCONTROL") == "y":
                        header += f"#define GPIO_{port}_RTS GPIO_USART1_TX\n"
                    if values.get("SERIAL_OFLOWCONTROL") == "y":
                        header += f"#define GPIO_{port}_CTS GPIO_USART1_RX\n"
            shim = directory / "variant.h"
            shim.write_text(header)
            for original in commands:
                command = original.copy()
                index = command.index("-o") + 1
                command[index] = str(directory / pathlib.Path(command[index]).name)
                subprocess.run(command + ["-Werror", "-include", str(shim)],
                               cwd=arch, check=True)
            print("STM32N6 generated-config ARM build passed:", name, flush=True)


if __name__ == "__main__":
    main()
