/****************************************************************************
 * arch/arm/src/stm32n6/stm32_serial_format.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_SERIAL_FORMAT_H
#define __ARCH_ARM_SRC_STM32N6_STM32_SERIAL_FORMAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/stm32n6xxx_uart.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct stm32_usart_format_s
{
  uint32_t cr1;
  uint32_t cr2;
  uint32_t brr;
  uint32_t presc;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* RM0486 sections 65.5.8 and 65.8.15.  Keep the result untouched on error.
 * OVER8 has no divisor bit 0: round to an even USARTDIV before encoding it.
 */

static inline int stm32_usart_format(uint32_t clock, uint32_t baud,
                                     uint8_t bits, uint8_t parity,
                                     bool stopbits2,
                                     struct stm32_usart_format_s *format)
{
  static const uint16_t divisors[] =
  {
    1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256
  };

  struct stm32_usart_format_s result;
  uint64_t denominator;
  uint64_t divider;
  unsigned int i;

  if (format == NULL || clock == 0 || baud == 0 ||
      (bits != 7 && bits != 8) || parity > 2)
    {
      return -EINVAL;
    }

  result.cr1 = parity != 0 ? USART_CR1_PCE : 0;
  if (parity == 1)
    {
      result.cr1 |= USART_CR1_PS;
    }

  if (bits == 8 && parity != 0)
    {
      result.cr1 |= USART_CR1_M0;
    }
  else if (bits == 7 && parity == 0)
    {
      result.cr1 |= USART_CR1_M1;
    }

  result.cr2 = stopbits2 ? USART_CR2_STOP2 : USART_CR2_STOP1;

  for (i = 0; i < sizeof(divisors) / sizeof(divisors[0]); i++)
    {
      denominator = (uint64_t)baud * divisors[i];
      divider = ((uint64_t)clock + denominator / 2) / denominator;

      if ((uint64_t)clock >= denominator * 16 &&
          divider >= 16 && divider <= UINT16_MAX)
        {
          result.brr = divider;
        }
      else if ((uint64_t)clock >= denominator * 8 &&
               divider >= 8 && divider <= UINT16_MAX / 2)
        {
          divider *= 2;
          result.brr = (divider & 0xfff0) | ((divider & 0xf) >> 1);
          result.cr1 |= USART_CR1_OVER8;
        }
      else
        {
          continue;
        }

      result.presc = i;
      *format = result;
      return 0;
    }

  return -ERANGE;
}

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_SERIAL_FORMAT_H */
