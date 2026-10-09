/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_usb_cdc_desc_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAR
#define CODE
#define EXTERN extern
#define UNUSED(n) (void)(n)
#define begin_packed_struct
#define end_packed_struct __attribute__((packed))

/* CDC_HEADERS */

/* CDC_DESCRIPTORS */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned int test_word(const uint8_t *data)
{
  return data[0] | (unsigned int)data[1] << 8;
}

static void test_config(uint8_t speed, uint8_t type)
{
  struct usbdev_devinfo_s info =
  {
    .ifnobase = 0,
    .ninterfaces = 2,
    .epno =
    {
      1, 2, 3
    }
  };

  uint8_t data[128];
  const struct usb_cfgdesc_s *cfg;
  const struct usb_epdesc_s *ep;
  unsigned int endpoints = 0;
  unsigned int offset;
  unsigned int effective = speed;
  int length;

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      effective = speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH;
    }

  memset(data, 0xa5, sizeof(data));
  length = cdcacm_mkcfgdesc(data, &info, speed, type);
  assert(length > 0 && length < (int)sizeof(data));
  assert(data[length] == 0xa5);
  assert(length == cdcacm_mkcfgdesc(NULL, &info, speed, type));
  cfg = (const struct usb_cfgdesc_s *)data;
  assert(cfg->len == USB_SIZEOF_CFGDESC && cfg->type == type);
  assert(test_word(cfg->totallen) == (unsigned int)length);
  assert(cfg->attr == (USB_CONFIG_ATTR_ONE | USB_CONFIG_ATTR_SELFPOWER));
  assert(cfg->mxpower == 50 && cfg->ninterfaces == 2);

  for (offset = 0; offset < (unsigned int)length; offset += data[offset])
    {
      assert(data[offset] != 0 &&
             offset + data[offset] <= (unsigned int)length);
      if (data[offset + 1] != USB_DESC_TYPE_ENDPOINT)
        {
          continue;
        }

      ep = (const struct usb_epdesc_s *)(data + offset);
      endpoints++;
      if (ep->attr == USB_EP_ATTR_XFER_INT)
        {
          assert(ep->addr == 0x81 && test_word(ep->mxpacketsize) == 16);
          assert(ep->interval == 10);
        }
      else
        {
          assert(ep->addr == 0x82 || ep->addr == 0x03);
          assert(ep->attr == USB_EP_ATTR_XFER_BULK);
          assert(test_word(ep->mxpacketsize) ==
                 (effective == USB_SPEED_HIGH ? 512 : 64));
        }
    }

  assert(endpoints == 3 && offset == (unsigned int)length);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  const struct usb_devdesc_s *dev = cdcacm_getdevdesc();
  uint8_t strdata[128];
  struct usb_strdesc_s *str = (struct usb_strdesc_s *)strdata;
  const char *product = "Nucleo N657 CDC ACM";
  unsigned int n;

  assert(test_word(dev->vendor) == 0x16c0);
  assert(test_word(dev->product) == 0x05e1);
  assert(dev->classid == USB_CLASS_CDC && dev->mxpacketsize == 64);
  assert(cdcacm_mkstrdesc(CDCACM_PRODUCTSTRID, str) ==
         (int)(2 + 2 * strlen(product)));
  for (n = 0; n < strlen(product); n++)
    {
      assert(strdata[2 + 2 * n] == (uint8_t)product[n]);
      assert(strdata[3 + 2 * n] == 0);
    }

  test_config(USB_SPEED_FULL, USB_DESC_TYPE_CONFIG);
#ifdef CONFIG_USBDEV_DUALSPEED
  {
    const struct usb_qualdesc_s *qual = cdcacm_getqualdesc();

    assert(qual->classid == dev->classid && qual->subclass == dev->subclass);
    assert(qual->protocol == dev->protocol && qual->mxpacketsize == 64);
    assert(qual->nconfigs == 1);
  }

  test_config(USB_SPEED_HIGH, USB_DESC_TYPE_CONFIG);
  test_config(USB_SPEED_FULL, USB_DESC_TYPE_OTHERSPEEDCONFIG);
  test_config(USB_SPEED_HIGH, USB_DESC_TYPE_OTHERSPEEDCONFIG);
#endif

  puts("CN8 CDC descriptors: PASS");
  return 0;
}
