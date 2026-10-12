/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_serial_format_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../../stm32_serial_format.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void test_baud(void)
{
  static const uint32_t rates[] =
  {
    9600, 57600, 100000, 115200, 460800, 921600, 1500000
  };

  static const uint32_t expected[] =
  {
    6667, 1111, 640, 556, 139, 69, 43
  };

  struct stm32_usart_format_s format;
  unsigned int i;

  for (i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
    {
      assert(stm32_usart_format(64000000, rates[i], 8, 0, false,
                                &format) == 0);
      assert(format.brr == expected[i]);
      assert(format.presc == USART_PRESC_DIV1);
      assert((format.cr1 & USART_CR1_OVER8) == 0);
    }

  assert(stm32_usart_format(64000000, 4000000, 8, 0, false, &format) == 0);
  assert(format.brr == 16 && (format.cr1 & USART_CR1_OVER8) == 0);
  assert(stm32_usart_format(64000000, 4000001, 8, 0, false, &format) == 0);
  assert(format.brr == 32 && (format.cr1 & USART_CR1_OVER8) != 0);
  assert(stm32_usart_format(64000000, 8000000, 8, 0, false, &format) == 0);
  assert(format.brr == 16 && (format.cr1 & USART_CR1_OVER8) != 0);
  assert(stm32_usart_format(64000000, 8000001, 8, 0, false,
                            &format) == -ERANGE);

  assert(stm32_usart_format(65535, 1, 8, 0, false, &format) == 0);
  assert(format.brr == 65535 && format.presc == 0);
  assert(stm32_usart_format(65536, 1, 8, 0, false, &format) == 0);
  assert(format.brr == 32768 && format.presc == 1);
  assert(stm32_usart_format(64000000, 4, 8, 0, false, &format) == 0);
  assert(format.brr == 62500 && format.presc == 11);
  assert(stm32_usart_format(64000000, 3, 8, 0, false, &format) == -ERANGE);

  /* The odd OVER8 divisor's bit 0 is not representable.  Round to the
   * nearest even divisor, rather than truncate a rounded odd divisor.
   */

  assert(stm32_usart_format(172, 20, 8, 0, false, &format) == 0);
  assert(format.brr == 0x11 && (format.brr & 8) == 0);
  assert((format.cr1 & USART_CR1_OVER8) != 0);
}

static void test_formats(void)
{
  struct stm32_usart_format_s format;
  uint8_t bits;
  uint8_t parity;
  unsigned int stop;
  unsigned int hsidiv;

  for (hsidiv = 0; hsidiv < 4; hsidiv++)
    {
      for (bits = 7; bits <= 8; bits++)
        {
          for (parity = 0; parity < 3; parity++)
            {
              for (stop = 0; stop < 2; stop++)
                {
                  assert(stm32_usart_format(64000000 >> hsidiv, 115200,
                                            bits, parity, stop != 0,
                                            &format) == 0);
                  assert(!!(format.cr1 & USART_CR1_PCE) == (parity != 0));
                  assert(!!(format.cr1 & USART_CR1_PS) == (parity == 1));
                  assert(!!(format.cr1 & USART_CR1_M0) ==
                         (bits == 8 && parity != 0));
                  assert(!!(format.cr1 & USART_CR1_M1) ==
                         (bits == 7 && parity == 0));
                  assert(format.cr2 ==
                         (stop != 0 ? USART_CR2_STOP2 : USART_CR2_STOP1));
                }
            }
        }
    }
}

static void test_invalid(void)
{
  struct stm32_usart_format_s original =
  {
    .cr1 = 1, .cr2 = 2, .brr = 3, .presc = 4
  };

  struct stm32_usart_format_s format = original;
  uint8_t bits;

  assert(stm32_usart_format(64000000, 0, 8, 0, false, &format) == -EINVAL);
  assert(memcmp(&format, &original, sizeof(format)) == 0);
  assert(stm32_usart_format(0, 115200, 8, 0, false, &format) == -EINVAL);
  assert(stm32_usart_format(64000000, UINT32_MAX, 8, 0, false,
                            &format) == -ERANGE);
  assert(memcmp(&format, &original, sizeof(format)) == 0);
  assert(stm32_usart_format(64000000, 115200, 8, 3, false,
                            &format) == -EINVAL);
  assert(stm32_usart_format(64000000, 115200, 8, 0, false,
                            NULL) == -EINVAL);
  for (bits = 0; bits < 16; bits++)
    {
      if (bits != 7 && bits != 8)
        {
          assert(stm32_usart_format(64000000, 115200, bits, 0, false,
                                    &format) == -EINVAL);
          assert(memcmp(&format, &original, sizeof(format)) == 0);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  test_baud();
  test_formats();
  test_invalid();
  puts("STM32N6 serial baud/format tests passed");
  return 0;
}
