/****************************************************************************
 * arch/arm/src/stm32n6/stm32_otg.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_OTG_H
#define __ARCH_ARM_SRC_STM32N6_STM32_OTG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32_N6_OTGDEV

#include <arch/irq.h>
#include <stdbool.h>
#include "hardware/stm32n6xxx_memorymap.h"
#include "hardware/stm32n6xxx_otg.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "hardware/stm32n6xxx_usbphyc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32_N6_OTG1) == defined(CONFIG_STM32_N6_OTG2)
#  error "Select exactly one STM32N6 USB device controller"
#endif

#if !defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST) || \
    defined(CONFIG_USBDEV_DMA) || defined(CONFIG_USBDEV_ISOCHRONOUS) || \
    defined(CONFIG_USBDEV_SUPERSPEED) || defined(CONFIG_USBDEV_COMPOSITE)
#  error "STM32N6 supports only a single FIFO-mode USB device configuration"
#endif

#if defined(CONFIG_STM32_N6_OTGDEV_FS) && defined(CONFIG_USBDEV_DUALSPEED)
#  error "Forced full-speed USB cannot advertise dual-speed operation"
#endif

#ifdef CONFIG_STM32_N6_OTG1
#  define STM32_OTG_PORT                1
#  define STM32_OTG_BASE                STM32_USB1_OTG_HS_BASE
#  define STM32_USBPHYC_BASE            STM32_USB1_HS_PHYC_BASE
#  define STM32_IRQ_OTG                 STM32_IRQ_USB1_OTG_HS
#  define STM32_OTG_RCC_EN              RCC_AHB5ENR_OTG1EN
#  define STM32_OTG_RCC_PHY_EN          RCC_AHB5ENR_OTGPHY1EN
#  define STM32_OTG_RCC_RST             RCC_AHB5RSTR_OTG1RST
#  define STM32_OTG_RCC_PHY_RST         RCC_AHB5RSTR_OTGPHY1RST
#  define STM32_OTG_RCC_PHYCTL_RST      RCC_AHB5RSTR_OTG1PHYCTLRST
#  define STM32_OTG_RIFSC_INDEX         56
#  define STM32_OTG_RCC_CLKSEL_MASK     (RCC_CCIPR6_OTGPHY1SEL_MASK | \
                                        RCC_CCIPR6_OTGPHY1CKREFSEL)
#  define STM32_OTG_RCC_CLKSEL          RCC_CCIPR6_OTGPHY1CKREFSEL
#  define STM32_OTG_RCC_OTHER_EN        (RCC_AHB5ENR_OTG2EN | \
                                        RCC_AHB5ENR_OTGPHY2EN)
#else
#  define STM32_OTG_PORT                2
#  define STM32_OTG_BASE                STM32_USB2_OTG_HS_BASE
#  define STM32_USBPHYC_BASE            STM32_USB2_HS_PHYC_BASE
#  define STM32_IRQ_OTG                 STM32_IRQ_USB2_OTG_HS
#  define STM32_OTG_RCC_EN              RCC_AHB5ENR_OTG2EN
#  define STM32_OTG_RCC_PHY_EN          RCC_AHB5ENR_OTGPHY2EN
#  define STM32_OTG_RCC_RST             RCC_AHB5RSTR_OTG2RST
#  define STM32_OTG_RCC_PHY_RST         RCC_AHB5RSTR_OTGPHY2RST
#  define STM32_OTG_RCC_PHYCTL_RST      RCC_AHB5RSTR_OTG2PHYCTLRST
#  define STM32_OTG_RIFSC_INDEX         57
#  define STM32_OTG_RCC_CLKSEL_MASK     (RCC_CCIPR6_OTGPHY2SEL_MASK | \
                                        RCC_CCIPR6_OTGPHY2CKREFSEL)
#  define STM32_OTG_RCC_CLKSEL          RCC_CCIPR6_OTGPHY2CKREFSEL
#  define STM32_OTG_RCC_OTHER_EN        (RCC_AHB5ENR_OTG1EN | \
                                        RCC_AHB5ENR_OTGPHY1EN)
#endif

/* The board must qualify Type-C sink/protection policy and actual VBUS
 * before reporting presence. This notification never controls VBUS sourcing.
 */

int stm32_usbdev_vbus(bool present);

#endif /* CONFIG_STM32_N6_OTGDEV */
#endif /* __ARCH_ARM_SRC_STM32N6_STM32_OTG_H */
