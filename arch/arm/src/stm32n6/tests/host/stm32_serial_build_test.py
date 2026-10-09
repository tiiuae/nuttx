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
    for filename in ("Kconfig-uart", "Kconfig-usart"):
        common = (root / "drivers/serial" / filename).read_text()
        assert "ARCH_CHIP_" not in common, filename
        assert "STM32_GPDMA1" not in common, filename
    ports = ["USART1", "USART2", "USART3", "UART4", "UART5",
             "USART6", "UART7", "UART8", "UART9", "USART10"]
    baseline = (root / "boards/arm/stm32n6/nucleo-n657x0-q/configs/"
                "nsh-xspi/defconfig").read_text()
    baseline = "\n".join(line for line in baseline.splitlines()
                         if line not in ("CONFIG_STM32_GPDMA1=y",
                                         "CONFIG_STM32_HPDMA1=y")
                         and not any(line.startswith("CONFIG_" + prefix)
                                    for port in ports
                                    for prefix in ("STM32_" + port, port + "_")))
    output = subprocess.check_output(
        ["make", "-s", "-C", str(arch), "TOPDIR=" + str(root), "-B", "-n",
         "stm32_lowputc.o", "stm32_serial.o", "stm32_start.o",
         "stm32_dma.o"], text=True)
    commands = [shlex.split(line) for line in output.splitlines()
                if line.startswith("arm-none-eabi-gcc ")]
    if len(commands) != 4:
        raise RuntimeError("Expected four ARM compile commands")
    board_command = next(command.copy() for command in commands
                         if "chip/stm32_serial.c" in command)
    board_command[board_command.index("chip/stm32_serial.c")] = str(
        root / "boards/arm/stm32n6/nucleo-n657x0-q/src/stm32_dma_test.c")
    board_command[board_command.index("-o") + 1] = "stm32_dma_test.o"
    commands.append(board_command)
    variants = [(port.lower(), [port], port, []) for port in ports]
    variants += [(port.lower() + "-dma", [port], port,
                  ["STM32_GPDMA1", port + "_TXDMA"]) for port in ports]
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
        ("all-dma", ports, "USART1",
         ["STM32_GPDMA1"] + [port + "_TXDMA" for port in ports]),
        ("mixed-dma", ports, "USART1",
         ["STM32_GPDMA1", "USART3_TXDMA", "UART9_TXDMA", "USART10_TXDMA"]),
        ("dma-no-termios", ports, "USART1",
         ["STM32_GPDMA1", "SERIAL_TERMIOS=n", "USART3_TXDMA"]),
        ("dma-pm-flow", ports, "USART1",
         ["STM32_GPDMA1", "PM", "USART1_TXDMA", "USART3_TXDMA",
          "USART3_OFLOWCONTROL", "USART3_IFLOWCONTROL"]),
        ("dma-no-console", ports, None,
         ["STM32_GPDMA1", "USART3_TXDMA"]),
        ("dma-suppressed", ports, "USART1",
         ["STM32_GPDMA1", "USART1_TXDMA", "SUPPRESS_UART_CONFIG"]),
        ("dma-cache-off", ports, "USART1",
         ["STM32_GPDMA1", "USART1_TXDMA", "ARMV8M_DCACHE=n"]),
        ("unsupported-rxdma", ports, "USART1",
         ["STM32_GPDMA1"] + [port + "_RXDMA" for port in ports[:8]]),
        ("dma-without-controller", ports, "USART1",
         ["STM32_GPDMA1=n"] + [port + "_TXDMA" for port in ports]),
        ("dma-hpdma-only", ports, "USART1",
         ["STM32_GPDMA1=n", "STM32_HPDMA1"] +
         [port + "_TXDMA" for port in ports]),
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
            if "ARMV8M_DCACHE=n" in options:
                request = request.replace("CONFIG_ARMV8M_DCACHE=y\n", "")
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
                if option.endswith("TXDMA"):
                    assert values.get(option) == "y", (name, option)
            if any(values.get(port + "_TXDMA") == "y" for port in ports):
                assert values.get("SERIAL_TXDMA") == "y", name
            if name == "unsupported-rxdma":
                assert all(values.get(port + "_RXDMA") == "y"
                           for port in ports[:8]), name
            else:
                assert not any(values.get(port + "_RXDMA") == "y"
                               for port in ports), name
            assert values.get("STM32_DMA1") != "y"
            for option in ["STM32_USART_BREAKS", "STM32_USART_SWAP"] + [
                    port + "_RS485" for port in ports]:
                assert values.get(option) != "y", (name, option)
            assert not any(value == "y" for key, value in values.items()
                           if key.startswith("STM32_HAVE_IP_USART") or
                           key.startswith("STM32_HAVE_LPUART"))
            symbols = {"STM32_GPDMA1", "ARMV8M_DCACHE", "SERIAL_TERMIOS",
                       "SERIAL_TXDMA", "SERIAL_RXDMA",
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
            expected_error = None
            if name == "unsupported-rxdma":
                expected_error = "STM32N6 serial RX DMA is not implemented"
            elif name in ("dma-without-controller", "dma-hpdma-only"):
                assert values.get("STM32_GPDMA1") != "y", name
                expected_error = (
                    "STM32N6 serial TX DMA requires CONFIG_STM32_GPDMA1")
            for original in commands:
                command = original.copy()
                index = command.index("-o") + 1
                command[index] = str(directory / pathlib.Path(command[index]).name)
                if expected_error:
                    result = subprocess.run(
                        command + ["-Werror", "-include", str(shim)],
                        cwd=arch, capture_output=True, text=True)
                    assert result.returncode != 0, name
                    assert expected_error in result.stderr, (name, result.stderr)
                    break
                else:
                    subprocess.run(command + ["-Werror", "-include", str(shim)],
                                   cwd=arch, check=True)
            outcome = "rejected" if expected_error else "passed"
            print(f"STM32N6 generated-config ARM build {outcome}: {name}",
                  flush=True)


if __name__ == "__main__":
    main()
