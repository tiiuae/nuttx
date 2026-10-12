#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Exercise the adapted H5 I2C routines against N6 register mocks."""

import os
import pathlib
import shlex
import subprocess
import tempfile

from stm32_serial_driver_test import extract


def main():
    directory = pathlib.Path(__file__).resolve().parent
    chip = directory.parent.parent
    driver = (chip / "stm32_i2c.c").read_text()
    source = (directory / "stm32_i2c_driver_test.c").read_text()
    source = source.replace(
        "/* DRIVER_TYPES */",
        "\n".join(extract(driver, declaration) + ";" for declaration in (
            r"enum stm32_intstate_e", r"struct stm32_i2c_config_s",
            r"struct stm32_i2c_priv_s", r"struct stm32_i2c_inst_s")),
    )
    start = driver.index("#ifdef CONFIG_STM32N6_I2C1\nstatic const")
    end = driver.index("/* Device Structures, Instantiation */", start)
    source = source.replace("/* DRIVER_CONFIGS */", driver[start:end])
    names = (
        "getreg", "getreg32", "putreg", "putreg32", "modifyreg32",
        "set_7bit_address", "set_bytes_to_transfer", "set_write_transfer_dir",
        "set_read_transfer_dir", "enable_reload", "disable_reload",
        "sem_waitstop", "setclock", "sendstart", "sendstop", "getstatus",
        "clearinterrupts", "isr_process", "init", "deinit", "process",
        "transfer",
    )
    routines = "\n".join(extract(
        driver, r"static\s+(?:inline\s+)?(?:void|int|uint\d+_t)\s+"
        + "stm32_i2c_" + name + r"\([^;{]*\)") for name in names)
    polled = driver.index("#else\nstatic inline int stm32_i2c_sem_waitdone")
    routines += "\n" + extract(
        driver[polled:], r"static inline int stm32_i2c_sem_waitdone\([^;{]*\)")
    source = source.replace("/* DRIVER_ROUTINES */", routines)
    with tempfile.TemporaryDirectory(prefix="stm32n6-i2c-") as temporary:
        executable = pathlib.Path(temporary) / "i2c-driver"
        compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
        subprocess.run(compiler + [
            "-x", "c", "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I" + str(directory / "include"), "-I" + str(chip),
            "-o", str(executable), "-",
        ], input=source, text=True, check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
