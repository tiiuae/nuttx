HOSTCC ?= cc
STM32N6_DIR := $(abspath ../..)
TEST_SOURCES := stm32_dma_helpers_test.c stm32_i2c_timing_test.c
TEST_INCLUDES := -Iinclude -I$(STM32N6_DIR)
TEST_CFLAGS := -std=c11 -Wall -Wextra -Werror -pthread

.PHONY: check

check:
	@set -e; \
	for source in $(TEST_SOURCES); do \
	  tmp=$$(mktemp); \
	  trap 'rm -f "$$tmp"' EXIT; \
	  $(HOSTCC) $(TEST_CFLAGS) $(TEST_INCLUDES) "$$source" -o "$$tmp"; \
	  "$$tmp"; \
	  rm -f "$$tmp"; \
	  trap - EXIT; \
	done
