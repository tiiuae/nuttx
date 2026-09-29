/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dma.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32N6_STM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Callback and status() flags.  DMA_STATUS_FATAL contains errors that stop
 * the active transfer; the other flags report transfer progress or events.
 */

#define DMA_STATUS_TCF        (1u << 0) /* Transfer complete */
#define DMA_STATUS_HTF        (1u << 1) /* Half transfer */
#define DMA_STATUS_DTEF       (1u << 2) /* Data transfer error */
#define DMA_STATUS_ULEF       (1u << 3) /* Link transfer error */
#define DMA_STATUS_USEF       (1u << 4) /* User setting error */
#define DMA_STATUS_SUSPF      (1u << 5) /* Channel suspended */
#define DMA_STATUS_TOF        (1u << 6) /* Trigger overrun */

#define DMA_STATUS_FATAL      (DMA_STATUS_DTEF | DMA_STATUS_ULEF | \
                               DMA_STATUS_USEF)

#define STM32_DMA_REQUEST_NONE 0xffff

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum stm32_dma_controller_e
{
  STM32_DMA_CONTROLLER_GPDMA1 = 0,
  STM32_DMA_CONTROLLER_HPDMA1
};

enum stm32_dma_direction_e
{
  STM32_DMA_PERIPHERAL_TO_MEMORY = 0,
  STM32_DMA_MEMORY_TO_PERIPHERAL,
  STM32_DMA_MEMORY_TO_MEMORY
};

/* The controller and request are deliberately part of the allocation
 * descriptor.  A request number alone is not sufficient to choose a DMA
 * controller on STM32N6.
 */

struct stm32_dma_request_s
{
  enum stm32_dma_controller_e controller;
  enum stm32_dma_direction_e direction;
  uint16_t request;
  uintptr_t peripheral_address;  /* Fixed register address for P2M/M2P */
};

struct stm32_dma_config_s
{
  uintptr_t source_address;
  uintptr_t destination_address;
  size_t nbytes;                 /* Transfer size in bytes, maximum 65535 */
  uint8_t width;                 /* Bytes per transfer: 1, 2, or 4 (8 M2M HPDMA) */
  uint8_t priority;              /* 0..3 */
  bool source_increment;
  bool destination_increment;
};

struct stm32_dma_status_s
{
  uint32_t flags;                /* Accumulated status flags since last start */
  size_t remaining;              /* Remaining bytes */
  int error;                     /* Last driver/transfer error */
  bool in_flight;
};

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize configured controllers and attach their channel interrupts.
 * Returns 0 or a negative errno value.
 */

int stm32_dma_initialize(void);

/* Allocate from the explicitly selected controller's non-reserved channel
 * pool.  Returns NULL when the request is invalid or no channel is available.
 */

DMA_HANDLE stm32_dmachannel(const struct stm32_dma_request_s *request);
int stm32_dmafree(DMA_HANDLE handle);
int stm32_dmasetup(DMA_HANDLE handle,
                   const struct stm32_dma_config_s *config);
int stm32_dmacallback(DMA_HANDLE handle, dma_callback_t callback, void *arg);
int stm32_dmastart(DMA_HANDLE handle);

/* Abort a transfer using the RM0486 suspend/wait/reset sequence.  A timeout
 * is returned if the hardware does not reach the requested state.
 */

int stm32_dmastop(DMA_HANDLE handle);
int stm32_dmastatus(DMA_HANDLE handle, struct stm32_dma_status_s *status);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_DMA_H */
