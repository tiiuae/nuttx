/****************************************************************************
 * arch/arm64/src/imx9/imx9_ele.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"
#include "imx9_gpio.h"
#include "imx9_iomuxc.h"
#include "imx9_ele.h"

#include "hardware/imx9_mua.h"
#include "hardware/imx9_pinmux.h"


//#ifdef CONFIG_IMX9_S3MUA

#define s3info         _err
//#define s3info        _none

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct header_t
{
  union
   {
    uint32_t data;
    struct
     {
      uint32_t ver : 8;
      uint32_t size : 8;
      uint32_t command : 8;
      uint32_t tag : 8;
    };
  };
};

struct keyb_option_t
{
  union {
    uint32_t data;
    struct
     {
      uint32_t flags : 8;
      uint32_t size : 8;
      uint32_t algorithm : 8;
      uint32_t reserved : 8;
    };
  };
};

struct keyb_header_t
{
  union
   {
    uint32_t data;
    struct
     {
      uint32_t version : 8;
      uint32_t size : 16;
      uint32_t tag : 8;
    };
  };
};


struct generate_key_blob_hdr
{
  uint8_t version;
  uint8_t length_lsb;
  uint8_t length_msb;
  uint8_t tag;
  uint8_t flags;
  uint8_t size;
  uint8_t algorithm;
  uint8_t mode;
} __packed;


struct ele_msg
{
  struct header_t header;
  uint32_t data[S3MUA_MSG_SIZE];
};

struct ele_cmd
{
  struct header_t header;
};

struct keyblob_msg
{
  struct header_t header;
  uint32_t key_ident;
  uint32_t load_addr_h;
  uint32_t load_addr_l;
  uint32_t export_addr_h;
  uint32_t export_addr_l;
  uint32_t max_export_size;
  uint32_t crc;
};

struct get_info_msg
{
  struct header_t header;
  uint32_t export_addr_h;
  uint32_t export_addr_l;
  uint32_t buffer_size;
};

struct __attribute__((__packed__)) keyblob_in
{
  struct keyb_header_t header;
  struct keyb_option_t option;
  uint8_t key[32]; /*256bit max*/
};

struct imx9_s3muadev_s
{
  uint32_t base;
  uint32_t ocram_base;
  mutex_t lock;
  bool initialized;
  uint32_t rng_status;
  uint32_t num_rx_ch;
  uint32_t num_tx_ch;
  void *tx_m;
  struct ele_msg rx_m;
};



/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imx9_s3muadev_s g_s3muadev_s =
{
  .base   = IMX9_S3MUA_BASE,
  .ocram_base = 0x20480000,
  .lock         = NXMUTEX_INITIALIZER,
  .initialized = false,
  .num_tx_ch = 8,
  .num_rx_ch = 4,
};
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void set_msg_header(uint32_t *hdr, uint8_t cmd, uint8_t size, uint8_t ver)
{
  *hdr = (TX_TAG << 24 | cmd << 16 | size << 8 | ver);
}

static uint32_t calculate_xor_checksum(uint32_t *data, uint32_t length)
{
  uint32_t checksum = 0;
  for(int i = 0; i < length; i++)
    {
      checksum ^= data[i];
    }
  return checksum;
}

/****************************************************************************
 * Name: imx9_s3mua_putreg
 *
 * Description:
 *   Write a 16-bit value to the S3MUA register at offset
 *
 * Input Parameters:
 *   priv   - private S3MUA device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline void imx9_s3mua_putreg32(struct imx9_s3muadev_s *priv,
                                       uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}
/****************************************************************************
 * Name: imx9_s3mua_getreg
 *
 * Description:
 *   Get the contents of the S3MUA register at offset
 *
 * Input Parameters:
 *   priv   - private S3MUA device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/
static inline uint32_t
imx9_s3mua_getreg32(struct imx9_s3muadev_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline uint32_t
get_ocram(struct imx9_s3muadev_s *priv, uint32_t offset)
{
  return getreg32(priv->ocram_base + offset);
}



/****************************************************************************
 * Name: imx9_s3mua_write_to_mbox
 *
 * Description:
 *   Write message to S3MUA mailbox
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   0 in success
 *
 ****************************************************************************/

static int imx9_s3mua_write_to_mbox(struct imx9_s3muadev_s *priv)
{
  struct header_t *header;
  uint32_t *msg = (void *)priv->tx_m;

  /* First word is header */
  header = (struct header_t*)priv->tx_m;


  if (header->size > S3MUA_MSG_SIZE)
    {
      s3info("Message too long %d \n", header->size);
      return -EINVAL;
    }
  s3info("write tx %08x<-%08x\n", priv->base + S3MUA_TX, msg[0]);

  for(int i = 0; i < header->size; i++)
    {
      int tx_channel;

      tx_channel = i % priv->num_tx_ch;
      while(!((1 << tx_channel) & imx9_s3mua_getreg32(priv, IMX9_MUA_TSR_OFFSET )));
      /* Write data */
      imx9_s3mua_putreg32(priv, S3MUA_TX + (tx_channel * 4), msg[i]);
    }
  return 0;
}
/****************************************************************************
 * Name: mx9_s3mua_read_from_mbox
 *
 * Description:
 *   Read datafrom S3MUA mailbox
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   0 as success
 *
 ****************************************************************************/
static int imx9_s3mua_read_from_mbox(struct imx9_s3muadev_s *priv)
{
  int length;
  uint32_t rx_channel;

  priv->rx_m.data[0] = 0;
  priv->rx_m.header.data = 0;

  /* wait data */
  while(!(MUA_SR_RFP & imx9_s3mua_getreg32(priv,IMX9_MUA_SR_OFFSET)));

  /* Read Header from channel 0 */
  priv->rx_m.header.data = imx9_s3mua_getreg32(priv,IMX9_MUA_RR_OFFSET);
  length = priv->rx_m.header.size;
  s3info("rx response length = %d \n",length);

  /* Read data from starting from channel 1*/
  rx_channel = 1;
  for(int i = 0; i < length - 1; i++)
    {
      /* Check if empty*/
      rx_channel = (i+1) % priv->num_rx_ch;
      while(!((1 << rx_channel) & imx9_s3mua_getreg32(priv, IMX9_MUA_RSR_OFFSET )));
      /* Read data*/
      priv->rx_m.data[i] = imx9_s3mua_getreg32(priv, IMX9_MUA_RR_OFFSET + (rx_channel * 4));
    }
  return 0;
}

/****************************************************************************
 * Name: imx9_s3mua_write_read
 *
 * Description:
 *   Write command to S3MUA and read response
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word - word to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static int imx9_s3mua_write_read(struct imx9_s3muadev_s *priv)
{
  int err;

  /* Mailbox operation is single write - single response */
  err = nxmutex_lock(&priv->lock);
  if (err < 0)
    {
      return err;
    }
  /* write command */
  err = imx9_s3mua_write_to_mbox(priv);

  if (!err)
    {
      err = imx9_s3mua_read_from_mbox(priv);
    }

  nxmutex_unlock(&priv->lock);
  return err;
}

int init_interface(void)
{
  struct imx9_s3muadev_s *priv = &g_s3muadev_s;
  struct ele_cmd cmd;

  set_msg_header(&cmd.header.data , INIT_FW_REQ, 1, MESSAGE_VER_6);
  priv->tx_m = &cmd;
  imx9_s3mua_write_read(priv);

  s3info("ELE status: 0x%x\n",priv->rx_m.data[0]);

  if (priv->rx_m.data[0] == ELE_OK)
    {
      priv->initialized = true;
      return 0;
    }
  return -1;
}

int imx9_ele_ping(void)
{
  struct imx9_s3muadev_s *priv = &g_s3muadev_s;

  static struct ele_cmd cmd;

  set_msg_header(&cmd.header.data , PING_FW_REQ, 1, MESSAGE_VER_6);
  priv->tx_m = &cmd;
  imx9_s3mua_write_read(priv);

  s3info("ELE ping status: 0x%x\n",priv->rx_m.data[0]);

  if (priv->rx_m.data[0] == ELE_OK)
    {
      return 0;
    }
  return -1;

}

int imx9_ele_fwstatus(void)
{
  struct imx9_s3muadev_s *priv = &g_s3muadev_s;
  static struct ele_cmd cmd;

  set_msg_header(&cmd.header.data , ELE_GET_FW_STATUS_REQ, 1, MESSAGE_VER_6);
  priv->tx_m = &cmd;

  imx9_s3mua_write_read(priv);

  s3info("ELE fw status  %d \n", (priv->rx_m.data[1]) & 0xff);

  if (((priv->rx_m.data[1]) & 0xff) == 1)
  {
    priv->initialized = true;
  }

  if (priv->rx_m.data[0] == ELE_OK)
    {
      return 0;
    }
  return -1;

}

int imx9_ele_get_info(void)
{
  struct imx9_s3muadev_s *priv = &g_s3muadev_s;

  static struct get_info_msg msg;
  static uint32_t info_resp[64] __attribute__((aligned(8))) = {0};

  msg.header.command = ELE_GET_INFO_REQ;
  msg.header.size = 4;
  msg.header.ver = MESSAGE_VER_6;
  msg.header.tag = TX_TAG;
  msg.buffer_size = 256;
  msg.export_addr_l = (uintptr_t)info_resp;
  msg.export_addr_h = 0;

  priv->tx_m = &msg;
  up_invalidate_dcache(msg.export_addr_l, msg.export_addr_l + sizeof(info_resp));
  imx9_s3mua_write_read(priv);

  up_invalidate_dcache(msg.export_addr_l, msg.export_addr_l + sizeof(info_resp));
  s3info("Get_info req status: 0x%x\n",priv->rx_m.data[0]);

  if (priv->rx_m.data[0] == ELE_OK)
    {
      s3info("Soc ID 0x%x \n", (info_resp[1] >> 16) & 0xFFFF);
      s3info("Soc revision 0x%x \n", (info_resp[1]) & 0xFFFF);
      s3info("TRNG state  0x%x \n", info_resp[39] & 0xFF);
      s3info("CSAL state  0x%x \n", (info_resp[39]>>8) & 0xFF);
      s3info("IMEM state  0x%x \n", (info_resp[39]>>16) & 0xFF);
      priv->rng_status = info_resp[39] & 0xFF ;
    }
  return 0;

}



int imx9_start_rng(void)
{
  struct imx9_s3muadev_s *priv = &g_s3muadev_s;
  static struct ele_cmd cmd;

  if(priv->rng_status == RNG_OK)
    {
      return 0;
    }

  set_msg_header(&cmd.header.data , ELE_START_RNG_REQ, 1, MESSAGE_VER_6);
  priv->tx_m = &cmd;
  imx9_s3mua_write_read(priv);


  s3info("rng start status: 0x%x\n",priv->rx_m.data[0]);

  if (priv->rx_m.data[0] == ELE_OK)
    {
      return 0;
    }
  return -1;

}


/* Initialization */
int imx9_ele_initialize(void)
{
  int err;
  //init_interface();
  err = imx9_start_rng();
  err = imx9_ele_ping();
  err = imx9_ele_fwstatus();
//  err = imx9_ele_get_info();

  return err;
}

int imx9_generate_key_blob(uint8_t *key, uint32_t num_bits, uint32_t key_id, void *key_blob)
{
  struct imx9_s3muadev_s *priv = &g_s3muadev_s;
  static struct keyblob_msg msg;

  /* Place keydata to OCRAM */
  static uint64_t blob_output[64] __attribute__((__section__(".secmem"))) = {0};
  static struct keyblob_in keybl __attribute__((aligned(8)))__attribute__((__section__(".secmem"))) = {0};

  if (num_bits != 128 || num_bits != 192 || num_bits != 256)
    {
      _err("Invalid key size %d\n", num_bits);
      return -EINVAL;
    }

  if ((!key) && (!key_blob))
    {
      return -EINVAL;
    }
  key_blob = blob_output;
  uint32_t key_len = num_bits / 8;

  set_msg_header(&msg.header.data , ELE_GENERATE_KEY_BLOB_REQ, 8, MESSAGE_VER_6);
  keybl.header.tag = KEYBLOB_TAG;
  keybl.header.version = KEYBLOB_VERSION;
  keybl.header.size = key_len + 8;
  keybl.option.algorithm = AES_CBC;
  keybl.option.flags = 0x1;
  keybl.option.size = key_len;

  msg.export_addr_h = 0,
  msg.load_addr_h = 0,
  msg.key_ident = key_id,
  msg.export_addr_l = (uintptr_t)blob_output;
  msg.load_addr_l = (uintptr_t)&keybl;
  msg.max_export_size = key_len + 56;
  msg.crc = calculate_xor_checksum((uint32_t *)&msg, 8);

  priv->tx_m = &msg;
  memcpy((void *)&keybl.key[0], (void*)&key[0], 16);

  up_invalidate_dcache(msg.load_addr_l,msg.load_addr_l + sizeof(struct keyblob_msg ) );

  imx9_s3mua_write_read(priv);
  /* Invalidate cache from blob result area*/
  up_invalidate_dcache(msg.export_addr_l,msg.export_addr_l + msg.max_export_size );

  s3info("Generate keyblob status: 0x%x\n",priv->rx_m.data[0]);
  return 0;

}

//#endif /* CONFIG_IMX9_LPS3MUA */
