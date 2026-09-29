/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dma_access.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "hardware/stm32n6xxx_gpdma.h"
#include "hardware/stm32n6xxx_hpdma.h"
#include "stm32_dma_access.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma_access_initialize
 *
 * Description:
 *   Assign the configured DMA channel pools to the secure privileged
 *   execution environment used by the STM32N6 DEV boot configuration.
 *
 *   RM0486 defines RIF-aware DMA security locally in each controller. The
 *   non-RIF-aware peripheral clients reset nonsecure and unprivileged, which
 *   permits secure DMA masters to access them; their RIFSC permissions do
 *   not need to be weakened or changed for this initial policy. HPDMA CID
 *   filtering and semaphores also remain disabled at their reset values.
 *
 ****************************************************************************/

void stm32_dma_access_initialize(void)
{
#ifdef CONFIG_STM32_GPDMA1
  uint32_t gpdma1_pool =
    (1u << CONFIG_STM32_GPDMA1_NCHANNELS) - 1u;

  putreg32(gpdma1_pool, STM32_GPDMA1_SECCFGR);
  putreg32(gpdma1_pool, STM32_GPDMA1_PRIVCFGR);
#endif

#ifdef CONFIG_STM32_HPDMA1
  uint32_t hpdma1_pool =
    (1u << CONFIG_STM32_HPDMA1_NCHANNELS) - 1u;

  putreg32(hpdma1_pool, STM32_HPDMA1_SECCFGR);
  putreg32(hpdma1_pool, STM32_HPDMA1_PRIVCFGR);
#endif
}
