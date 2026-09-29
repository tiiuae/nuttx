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

#define STM32_DMA_LLI_ALIGNMENT 32

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

enum stm32_dma_list_mode_e
{
  STM32_DMA_LIST_TERMINAL = 0, /* Stop after the last descriptor */
  STM32_DMA_LIST_CIRCULAR,     /* Repeat the complete descriptor ring */
  STM32_DMA_LIST_PINGPONG      /* Two alternating descriptors */
};

/* A fixed-size, 32-byte aligned descriptor supports both hardware layouts.
 * Channels 0-11 consume the first six words; channels 12-15 consume the
 * extended TR3/BR2/LLR layout in the union.
 */

struct stm32_dma_lli_s
{
  uint32_t tr1;
  uint32_t tr2;
  uint32_t br1;
  uint32_t sar;
  uint32_t dar;
  union
  {
    uint32_t llr;
    struct
    {
      uint32_t tr3;
      uint32_t br2;
      uint32_t llr;
    } extended;
  } tail;
} __attribute__((aligned(STM32_DMA_LLI_ALIGNMENT)));

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
/* Build and install a static linked-list.  Descriptor storage and all DMA
 * buffers must remain DMA-accessible until completion or explicit abort;
 * descriptors and memory-to-DMA buffers must not be modified while active.
 * The list array must be 32-byte aligned and fit wholly in one 64-Kbyte
 * address window.
 *
 * The DMA core cleans descriptors and memory sources before starting,
 * cleans RX destinations before DMA writes, and invalidates RX destinations
 * after each completed LLI (or after a successful abort).  With D-cache
 * enabled, RX ranges must cover complete cache lines and must not share
 * cache lines with unrelated writable data.  Do not access a DMA buffer
 * while its transfer is in flight; an HTF callback alone does not make RX
 * data cache-coherent.  TCF is reported for each LLI, and the final TCF in a
 * terminal list marks the end of descriptor ownership by the DMA core.
 */

int stm32_dmallibuild(DMA_HANDLE handle,
                      const struct stm32_dma_config_s *configs,
                      size_t count, struct stm32_dma_lli_s *descriptors,
                      size_t capacity, enum stm32_dma_list_mode_e mode);
int stm32_dmacallback(DMA_HANDLE handle, dma_callback_t callback, void *arg);
int stm32_dmastart(DMA_HANDLE handle);

/* Abort a transfer using the RM0486 suspend/wait/reset sequence.  A timeout
 * is returned if the hardware does not reach the requested state.
 */

int stm32_dmastop(DMA_HANDLE handle);
int stm32_dmastatus(DMA_HANDLE handle, struct stm32_dma_status_s *status);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_DMA_H */
