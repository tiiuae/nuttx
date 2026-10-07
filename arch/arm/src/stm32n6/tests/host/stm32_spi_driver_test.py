#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Run production SPI routines against host register mocks."""

import os
import pathlib
import shlex
import subprocess
import tempfile

from stm32_serial_driver_test import extract


def main():
    directory = pathlib.Path(__file__).resolve().parent
    chip = directory.parent.parent
    driver = (chip / "stm32_spi.c").read_text()
    source = (directory / "stm32_spi_driver_test.c").read_text()
    start = driver.index("#define SPI_TIMEOUT_MARGIN_US")
    end = driver.index("enum stm32_spi_state_e")
    source = source.replace("/* DRIVER_DEFINITIONS */", driver[start:end])
    source = source.replace(
        "/* DRIVER_TYPES */",
        "\n".join(
            extract(driver, declaration) + ";"
            for declaration in (
                r"enum stm32_spi_state_e",
                r"struct stm32_spi_priv_s",
                r"struct spi_deadline_s",
            )
        ),
    )
    names = (
        "spi_getreg", "spi_putreg", "spi_get_kernel_frequency",
        "spi_deadline_start", "spi_deadline_expired", "spi_apply_config",
        "spi_apply_mode", "spi_setmode", "spi_setfrequency", "spi_setbits",
        "spi_get_txframe", "spi_put_rxframe", "spi_rx_pending",
        "spi_read_rxframe", "spi_write_txframe", "spi_transfer_timeout_us",
        "spi_drain_rx", "spi_receive_frames", "spi_abort_transfer",
        "spi_transfer_chunk", "spi_transfer",
    )
    source = source.replace(
        "/* DRIVER_ROUTINES */",
        "\n".join(
            extract(
                driver,
                r"static\s+(?:inline\s+)?(?:void|int|bool|uint\d+_t)\s+"
                + name + r"\([^;{]*\)",
            )
            for name in names
        ),
    )
    with tempfile.TemporaryDirectory(prefix="stm32n6-spi-") as temporary:
        executable = pathlib.Path(temporary) / "spi-driver"
        compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
        subprocess.run(
            compiler + [
                "-x", "c", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I" + str(directory / "include"), "-I" + str(chip),
                "-o", str(executable), "-",
            ],
            input=source, text=True, check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
