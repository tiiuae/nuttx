/****************************************************************************
 * include/nuttx/net/ti_phy.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_TI_PHY_H
#define __INCLUDE_NUTTX_NET_TI_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: dp83867_register
 *
 * Description:
 *   Register the DP83867E driver before calling phy_device_create().
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PHY_TI_DP83867
int dp83867_register(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_TI_PHY_H */
