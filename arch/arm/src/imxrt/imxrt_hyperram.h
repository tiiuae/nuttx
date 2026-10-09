/****************************************************************************
 * arch/arm/src/imxrt/imxrt_hyperram.h
 *
 * Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_HYPERRAM_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_HYPERRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>

#include "imxrt_flexspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_IMXRT_HYPERRAM
#  ifdef CONFIG_IMXRT_HYPERRAM_FLEXSPI2
#    define IMXRT_HYPERRAM_CTRL_BASE IMXRT_FLEXSPI2C_BASE
#    if defined(CONFIG_ARCH_FAMILY_IMXRT118x)
#      define IMXRT_HYPERRAM_BASE IMXRT_FLEXSPI2_XIP_BASE
#    elif defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#      define IMXRT_HYPERRAM_BASE IMXRT_FLEXSPI2_CIPHER_BASE
#    elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#      define IMXRT_HYPERRAM_BASE IMXRT_FLEX2CIPHER_BASE
#    else
#      error Unsupported HyperRAM FlexSPI2 memory map
#    endif
#  elif defined(CONFIG_IMXRT_HYPERRAM_FLEXSPI1)
#    define IMXRT_HYPERRAM_CTRL_BASE IMXRT_FLEXSPIC_BASE
#    if defined(CONFIG_ARCH_FAMILY_IMXRT118x)
#      define IMXRT_HYPERRAM_BASE IMXRT_FLEXSPI1_XIP_BASE
#    elif defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#      define IMXRT_HYPERRAM_BASE IMXRT_FLEXSPI1_CIPHER_BASE
#    else
#      define IMXRT_HYPERRAM_BASE IMXRT_FLEXCIPHER_BASE
#    endif
#  else
#    error Select a HyperRAM FlexSPI instance
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct imxrt_hyperram_config_s
{
  uint32_t size;                 /* Power-of-two bytes, base-aligned */
  uint32_t mcr0;
  uint32_t mcr1;
  uint32_t mcr2;
  uint32_t ahbcr;
  uint32_t flshcr1;
  uint32_t flshcr2;
  uint32_t flshcr4;
  uint32_t dllcr[2];
  const uint32_t *lut;
  size_t lut_words;
  uint32_t cr0_address;
  uint32_t powerup_us;
  uint16_t cr0_value;
  uint16_t device_id;
  uint8_t read_seq;              /* Two-byte register read LUT sequence */
  uint8_t write_seq;             /* Two-byte register write LUT sequence */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize a dedicated FlexSPI controller with HyperRAM on port A1.
 * The board configures pins and clocks first. Code, stack and configuration
 * must reside outside this controller's memory window. Other users must be
 * quiescent. Timings and LUT must match the device's reset and CR0 latency.
 * Returns zero on success or a negative errno on failure.
 */

int imxrt_hyperram_initialize(const struct imxrt_hyperram_config_s *config);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_HYPERRAM_H */
