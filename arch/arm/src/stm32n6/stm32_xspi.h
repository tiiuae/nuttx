/****************************************************************************
 * arch/arm/src/stm32n6/stm32_xspi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_XSPI_H
#define __ARCH_ARM_SRC_STM32N6_STM32_XSPI_H

#include <nuttx/config.h>
#include <nuttx/spi/qspi.h>

#ifdef CONFIG_STM32_XSPI

#ifdef __cplusplus
extern "C"
{
#endif

FAR struct qspi_dev_s *stm32_xspi_initialize(int intf);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STM32_XSPI */
#endif /* __ARCH_ARM_SRC_STM32N6_STM32_XSPI_H */
