/****************************************************************************
 * drivers/crypto/pnt/pnt_hcrypto.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <crypto/aes.h>
#include <crypto/cmac.h>
#include <string.h>
#include <strings.h>
#include <sys/random.h>

#include <se05x_scp03_crypto.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCRYPTO_AES_BLOCK 16

/****************************************************************************
 * Private Data
 ****************************************************************************/

static AES_CMAC_CTX g_cmac;
static AES_CTX g_aes;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aes_cbc(FAR const uint8_t *key, size_t keylen, FAR uint8_t *iv,
                   size_t ivlen, FAR const uint8_t *src, FAR uint8_t *dst,
                   size_t len, bool encrypt)
{
  uint8_t block[HCRYPTO_AES_BLOCK];
  size_t i;
  size_t j;

  if (key == NULL || iv == NULL || src == NULL || dst == NULL ||
      keylen != HCRYPTO_AES_BLOCK || ivlen != HCRYPTO_AES_BLOCK ||
      len % HCRYPTO_AES_BLOCK != 0 || aes_setkey(&g_aes, key, keylen) != 0)
    {
      return 1;
    }

  for (i = 0; i < len; i += HCRYPTO_AES_BLOCK)
    {
      if (encrypt)
        {
          for (j = 0; j < HCRYPTO_AES_BLOCK; j++)
            {
              block[j] = src[i + j] ^ iv[j];
            }

          aes_encrypt(&g_aes, block, &dst[i]);
          memcpy(iv, &dst[i], HCRYPTO_AES_BLOCK);
        }
      else
        {
          memcpy(block, &src[i], HCRYPTO_AES_BLOCK);
          aes_decrypt(&g_aes, block, &dst[i]);
          for (j = 0; j < HCRYPTO_AES_BLOCK; j++)
            {
              dst[i + j] ^= iv[j];
            }

          memcpy(iv, block, HCRYPTO_AES_BLOCK);
        }
    }

  explicit_bzero(&g_aes, sizeof(g_aes));
  explicit_bzero(block, sizeof(block));
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int hcrypto_get_random(FAR uint8_t *buffer, size_t len)
{
  while (len > 0)
    {
      ssize_t ret = getrandom(buffer, len, GRND_RANDOM);

      if (ret <= 0)
        {
          return 1;
        }

      buffer += ret;
      len -= ret;
    }

  return 0;
}

int hcrypto_cmac_oneshot(FAR uint8_t *key, size_t keylen,
                         FAR uint8_t *in, size_t inlen, FAR uint8_t *out,
                         FAR size_t *outlen)
{
  if (hcrypto_cmac_setup(key, keylen) == NULL ||
      hcrypto_cmac_update(&g_cmac, in, inlen) != 0)
    {
      return 1;
    }

  return hcrypto_cmac_final(&g_cmac, out, outlen);
}

FAR void *hcrypto_cmac_setup(FAR uint8_t *key, size_t keylen)
{
  if (key == NULL || keylen != AES_CMAC_KEY_LENGTH)
    {
      return NULL;
    }

  aes_cmac_init(&g_cmac);
  aes_cmac_setkey(&g_cmac, key);
  return &g_cmac;
}

int hcrypto_cmac_init(FAR void *ctx)
{
  return ctx == &g_cmac ? 0 : 1;
}

int hcrypto_cmac_update(FAR void *ctx, FAR uint8_t *in, size_t inlen)
{
  if (ctx != &g_cmac || (in == NULL && inlen != 0))
    {
      return 1;
    }

  aes_cmac_update(&g_cmac, in, inlen);
  return 0;
}

int hcrypto_cmac_final(FAR void *ctx, FAR uint8_t *out, FAR size_t *outlen)
{
  if (ctx != &g_cmac || out == NULL || outlen == NULL ||
      *outlen != AES_CMAC_DIGEST_LENGTH)
    {
      return 1;
    }

  aes_cmac_final(out, &g_cmac);
  explicit_bzero(&g_cmac, sizeof(g_cmac));
  return 0;
}

int hcrypto_aes_cbc_encrypt(FAR uint8_t *key, size_t keylen,
                            FAR uint8_t *iv, size_t ivlen,
                            FAR const uint8_t *src, FAR uint8_t *dst,
                            size_t len)
{
  return aes_cbc(key, keylen, iv, ivlen, src, dst, len, true);
}

int hcrypto_aes_cbc_decrypt(FAR uint8_t *key, size_t keylen,
                            FAR uint8_t *iv, size_t ivlen,
                            FAR const uint8_t *src, FAR uint8_t *dst,
                            size_t len)
{
  return aes_cbc(key, keylen, iv, ivlen, src, dst, len, false);
}
