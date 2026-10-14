/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_netc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_FAMILY_IMXRT118x
#  include "rt118x/imxrt118x_netc.h"
#else
#  error "NETC is only supported on i.MX RT118x"
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_H */
