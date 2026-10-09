HOSTCC ?= cc
STM32N6_DIR := $(abspath ../..)
TEST_SOURCES := stm32_dma_helpers_test.c stm32_i2c_timing_test.c \
	stm32_i2c_transfer_test.c stm32_serial_format_test.c
TEST_INCLUDES := -Iinclude -I$(STM32N6_DIR)
TEST_CFLAGS := -std=c11 -Wall -Wextra -Werror -pthread

.PHONY: check check-common check-spi check-tim check-serial check-serial-build \
	check-usb

check: check-common check-spi check-tim check-usb

check-common:
	@set -e; \
	for source in $(TEST_SOURCES); do \
	  tmp=$$(mktemp); \
	  trap 'rm -f "$$tmp"' EXIT; \
	  $(HOSTCC) $(TEST_CFLAGS) $(TEST_INCLUDES) "$$source" -o "$$tmp"; \
	  "$$tmp"; \
	  rm -f "$$tmp"; \
	  trap - EXIT; \
	done
	@HOSTCC="$(HOSTCC)" python3 stm32_serial_driver_test.py

check-spi:
	@HOSTCC="$(HOSTCC)" python3 -B stm32_spi_driver_test.py

check-tim:
	@HOSTCC="$(HOSTCC)" python3 -B stm32_tim_driver_test.py

check-serial: TEST_SOURCES = stm32_serial_format_test.c
check-serial: check-common

check-serial-build:
	@python3 stm32_serial_build_test.py

check-usb:
	@HOSTCC="$(HOSTCC)" python3 -B stm32_usb_scaffold_test.py
	@HOSTCC="$(HOSTCC)" python3 -B stm32_usb_board_test.py
