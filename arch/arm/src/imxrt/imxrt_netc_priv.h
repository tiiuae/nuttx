/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc_priv.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_PRIV_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include "imxrt_netc.h"

/* Standard 16-byte descriptors, little endian.  Avoid implementation-defined
 * C bitfields.  RX writeback overwrites the buffer address; buffer addresses
 * are derived from the private buffer pool, never from a completed RX BD.
 */

struct imxrt_netc_bd_s
{
  uint32_t word[4];
};

static_assert(sizeof(struct imxrt_netc_bd_s) == 16,
               "NETC standard BD must be 16 bytes");
static_assert(offsetof(struct imxrt_netc_bd_s, word[3]) == 12,
               "NETC flags must be at offset 12");

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETC_DMA_ALIGN           128
#define NETC_TX_FINAL            (1u << 31)
#define NETC_TX_INTERRUPT        (1u << 29)
#define NETC_TX_WRITTEN          (1u << 26)
#define NETC_TX_STATUS_MASK      (0x1ffu << 16)
#define NETC_RX_READY            (1u << 30)
#define NETC_RX_FINAL            (1u << 31)
#define NETC_RX_ERROR_MASK       (0xffu << 16)
#define NETC_RX_LENGTH_MASK      0xffffu

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int imxrt_netc_mdio_read(uintptr_t base, uint8_t phy, uint8_t reg,
                        uint16_t *value);
int imxrt_netc_mdio_write(uintptr_t base, uint8_t phy, uint8_t reg,
                         uint16_t value);
int imxrt_netc_switch_initialize(const struct imxrt_netc_config_s *config);

/* Disabling CPU port 4 establishes the ERR051246 PF4 FLR precondition. */

int imxrt_netc_switch_cpu(bool enable, const uint8_t *mac);
int imxrt_netc_switch_link(uint8_t *linkmask);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_PRIV_H */
