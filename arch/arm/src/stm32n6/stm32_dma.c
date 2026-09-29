/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dma.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32_dma.h"
#include "stm32_dma_access.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma_initialize
 *
 * Description:
 *   Apply the initial secure DEV-mode access policy after the selected DMA
 *   controllers have been enabled and reset.
 *
 ****************************************************************************/

void stm32_dma_initialize(void)
{
  stm32_dma_access_initialize();
}
