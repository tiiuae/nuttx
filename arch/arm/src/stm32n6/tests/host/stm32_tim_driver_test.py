#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Exercise the complete timer driver with host MMIO and IRQ mocks."""

import os
import pathlib
import re
import shlex
import subprocess
import tempfile


CHANNELS = [6, 4, 4, 4, 4, 0, 0, 6, 2, 1, 1, 2, 1, 1, 2, 1, 1, 0]


def without_includes(source):
    return re.sub(r"^#include[^\n]*\n", "", source, flags=re.M)


def variants():
    timers = list(range(1, 19))
    yield "all-no-gpio", timers, [], False, False
    yield "all-gpio", timers, [], True, True
    yield "sparse-gpio", [1, 2, 8, 18], [], False, True
    for timer in timers:
        yield f"tim{timer}-only", [timer], [], False, False
    for owner in ("PWM", "ADC", "DAC", "QE", "CAP"):
        reserved = [(timer, owner) for timer in timers
                    if owner != "CAP" or CHANNELS[timer - 1] != 0]
        yield f"reserved-{owner.lower()}", timers, reserved, False, False
        if owner != "CAP":
            yield f"tim18-reserved-{owner.lower()}", [2, 18], [
                (18, owner)
            ], False, False
    yield "mixed-ownership", timers, [
        (1, "PWM"), (2, "ADC"), (3, "DAC"),
        (4, "QE"), (8, "CAP"), (18, "ADC")
    ], False, False
    yield "none", [], [], False, False


def main():
    directory = pathlib.Path(__file__).resolve().parent
    chip = directory.parents[1]
    root = chip.parents[3]
    header = without_includes(
        (chip / "hardware/stm32n6xxx_tim.h").read_text()
        + (chip / "stm32_tim.h").read_text()
    )
    driver = without_includes((chip / "stm32_tim.c").read_text())
    functions = driver[driver.index(" * Private Functions"):]
    assert not re.search(r"#\s*if.*(?:CONFIG_STM32_TIM|GPIO_TIM)", functions)
    source = (directory / "stm32_tim_driver_test.c").read_text()
    source = source.replace("/* DRIVER_HEADER */", header)
    source = source.replace("/* DRIVER_SOURCE */", driver)
    compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
    with tempfile.TemporaryDirectory(prefix="stm32n6-tim-") as temporary:
        executable = pathlib.Path(temporary) / "tim-driver"
        for name, timers, reserved, all_gpio, priority in variants():
            excluded = {timer for timer, _ in reserved}
            enabled = sum(1 << timer for timer in timers
                          if timer not in excluded)
            defines = ["CONFIG_STM32_STM32N6XXXX"]
            defines += [f"CONFIG_STM32_TIM{timer}" for timer in timers]
            defines += [f"CONFIG_STM32_TIM{timer}_{owner}"
                        for timer, owner in reserved]
            defines += [f"TEST_ENABLED_MASK={enabled}u"]
            if priority:
                defines += ["CONFIG_ARCH_IRQPRIO"]
            defines += [f"STM32_TIM{timer}_CLKIN={timer * 1000000}u"
                        for timer in timers if timer not in excluded]
            if all_gpio:
                defines += ["TEST_ALL_GPIO"]
                pins = [(timer, channel) for timer in timers
                        for channel in range(1, CHANNELS[timer - 1] + 1)]
            elif name == "sparse-gpio":
                defines += ["TEST_SPARSE_GPIO"]
                pins = [(1, 6), (2, 3), (8, 5)]
            else:
                pins = []
            defines += [
                f"GPIO_TIM{timer}_CH{channel}OUT="
                f"{0 if (timer, channel) == (1, 6) else timer * 16 + channel}u"
                for timer, channel in pins
            ]
            subprocess.run(
                compiler + [
                    "-x", "c", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-implicit-fallthrough",
                    "-I" + str(directory / "include"), "-I" + str(chip),
                    "-I" + str(root / "arch/arm/include"),
                    *["-D" + define for define in defines],
                    "-o", str(executable), "-",
                ],
                input=source, text=True, check=True,
            )
            subprocess.run([str(executable)], check=True)
            print(f"PASS: {name}")


if __name__ == "__main__":
    main()
