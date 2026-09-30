HOSTCC ?= cc
STM32N6_DIR := $(abspath ../..)
TEST_SOURCE := stm32_dma_helpers_test.c
TEST_INCLUDES := -Iinclude -I$(STM32N6_DIR)
TEST_CFLAGS := -std=c11 -Wall -Wextra -Werror -pthread

.PHONY: check

check:
	@tmp=$$(mktemp); \
	trap 'rm -f "$$tmp"' EXIT; \
	$(HOSTCC) $(TEST_CFLAGS) $(TEST_INCLUDES) $(TEST_SOURCE) -o "$$tmp" && \
	"$$tmp"
