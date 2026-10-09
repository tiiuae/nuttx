/*****************************************************************************
 * arch/arm/src/imxrt/imxrt_caam.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/init.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/rt117x/imxrt117x_caam.h"
#include "imxrt_caam.h"
#include "imxrt_periphclks.h"

#ifdef CONFIG_IMXRT_CAAM

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#ifndef ARMV7M_DCACHE_LINESIZE
#  define ARMV7M_DCACHE_LINESIZE 32
#endif

/* One entry each way. A ring deeper than that would only let a second
 * request queue behind a caller that is already waiting for the first.
 */

#define CAAM_RING_ENTRIES     1

/* CAAM writes the result by DMA, so the landing buffer owns whole cache
 * lines and shares them with nothing.
 */

#define CAAM_RNG_BLOCKLEN     512

#define CAAM_DESC_WORDS       8

/* Descriptor words, from the SEC reference descriptor encoding. */

#define CAAM_DESC_HDR(len)    (0xb0800000 | (len))
#define CAAM_OP_RNG_GENERATE  0x82500000
#define CAAM_OP_RNG_RESEED    0x00000002
#define CAAM_OP_RNG_INIT_SH0  0x82500006
#define CAAM_OP_RNG_GEN_SK    0x82501000
#define CAAM_JUMP_WAIT_CLASS1 0xa2000001
#define CAAM_LOAD_CLRW        0x10880004
#define CAAM_FIFO_STORE_RNG   0x60340000
#define CAAM_KEY_CLASS2       0x04000000
#define CAAM_SEQ_IN_PTR       0xf0000000
#define CAAM_SEQ_OUT_PTR      0xf8000000
#define CAAM_OP_BLOB_ENCAP    0x870d0000
#define CAAM_OP_BLOB_DECAP    0x860d0000

#define CAAM_BLOB_BUFLEN      (IMXRT_CAAM_BLOB_MAX + IMXRT_CAAM_BLOB_OVERHEAD)

/* Entropy sample length, in system clocks. A self test that fails is
 * retried with a longer one, which is how NXP's own code finds a value
 * that passes across voltage and temperature.
 */

#define CAAM_INSTANTIATE_SETTLE 20000

#define CAAM_ENT_DELAY_MIN    3200
#define CAAM_ENT_DELAY_MAX    12800
#define CAAM_ENT_DELAY_STEP   400

#define CAAM_TIMEOUT          100000
#define CAAM_SPIN             2000
#define CAAM_SLEEP_TICKS      1000

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* The rings and the descriptor are read by CAAM over DMA, and the result is
 * written back the same way, so each one owns its cache lines outright.
 */

static uint32_t g_input_ring[CAAM_RING_ENTRIES]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint32_t g_output_ring[CAAM_RING_ENTRIES * 2]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint32_t g_desc[CAAM_DESC_WORDS]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint8_t g_rngbuf[CAAM_RNG_BLOCKLEN]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint8_t g_keymod[IMXRT_CAAM_BLOB_KEYMOD]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint8_t g_blob_in[CAAM_BLOB_BUFLEN]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint8_t g_blob_out[CAAM_BLOB_BUFLEN]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static bool g_initialized;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: imxrt_caam_clean
 *****************************************************************************/

static void imxrt_caam_clean(void *addr, size_t len)
{
  up_clean_dcache((uintptr_t)addr, (uintptr_t)addr + len);
}

/*****************************************************************************
 * Name: imxrt_caam_invalidate
 *****************************************************************************/

static void imxrt_caam_invalidate(void *addr, size_t len)
{
  up_invalidate_dcache((uintptr_t)addr, (uintptr_t)addr + len);
}

/*****************************************************************************
 * Name: imxrt_caam_run
 *
 * Description:
 *   Submit the descriptor in g_desc to job ring zero and wait for it.
 *
 * Returned Value:
 *   Zero on success, -EIO if the ring reported an error, -ETIMEDOUT if it
 *   never answered.
 *
 *****************************************************************************/

static int imxrt_caam_ring_init(void);

static bool imxrt_caam_wait(void)
{
  int i;

  for (i = 0; i < CAAM_SPIN; i++)
    {
      if (getreg32(IMXRT_CAAM_ORSF) != 0)
        {
          return true;
        }
    }

  if (!OSINIT_TASK_READY() || up_interrupt_context())
    {
      for (i = 0; i < CAAM_TIMEOUT; i++)
        {
          if (getreg32(IMXRT_CAAM_ORSF) != 0)
            {
              return true;
            }
        }

      return false;
    }

  for (i = 0; i < CAAM_SLEEP_TICKS; i++)
    {
      nxsig_usleep(USEC_PER_TICK);

      if (getreg32(IMXRT_CAAM_ORSF) != 0)
        {
          return true;
        }
    }

  return false;
}

static int imxrt_caam_run(void)
{
  uint32_t status;

  imxrt_caam_clean(g_desc, sizeof(g_desc));

  g_input_ring[0] = (uint32_t)(uintptr_t)g_desc;
  imxrt_caam_clean(g_input_ring, sizeof(g_input_ring));

  putreg32(1, IMXRT_CAAM_IRJA);

  if (!imxrt_caam_wait())
    {
      _err("ERROR: job ring did not answer\n");
      return -ETIMEDOUT;
    }

  imxrt_caam_invalidate(g_output_ring, sizeof(g_output_ring));
  status = g_output_ring[1];

  /* Tell the ring the slot is free again whatever the outcome, or the next
   * request finds it still occupied.
   */

  putreg32(1, IMXRT_CAAM_ORJR);

  if (status != 0)
    {
      _err("ERROR: job failed, status 0x%08" PRIx32 "\n", status);
      return -EIO;
    }

  return OK;
}

/*****************************************************************************
 * Name: imxrt_caam_kick_trng
 *
 * Description:
 *   Set the entropy sample length and the frequency limits derived from it,
 *   which is what the self test in the state handle instantiation checks
 *   against.
 *
 *****************************************************************************/

static void imxrt_caam_kick_trng(uint32_t ent_delay)
{
  uint32_t val;

  modifyreg32(IMXRT_CAAM_RTMCTL, 0, CAAM_RTMCTL_PRGM);

  val = getreg32(IMXRT_CAAM_RTSDCTL) & ~CAAM_RTSDCTL_ENT_DLY_MASK;
  putreg32(val | (ent_delay << CAAM_RTSDCTL_ENT_DLY_SHIFT),
           IMXRT_CAAM_RTSDCTL);

  putreg32(ent_delay >> 2, IMXRT_CAAM_RTFRQMIN);
  putreg32(ent_delay << 4, IMXRT_CAAM_RTFRQMAX);

  modifyreg32(IMXRT_CAAM_RTMCTL, CAAM_RTMCTL_PRGM, 0);
}

/*****************************************************************************
 * Name: imxrt_caam_instantiate
 *
 * Description:
 *   Instantiate RNG state handle zero, generating the secure keys with it if
 *   nothing has done so since power on.
 *
 *****************************************************************************/

static int imxrt_caam_instantiate(bool gen_sk)
{
  int words = 2;

  g_desc[1] = CAAM_OP_RNG_INIT_SH0;

  if (gen_sk)
    {
      g_desc[2] = CAAM_JUMP_WAIT_CLASS1;
      g_desc[3] = CAAM_LOAD_CLRW;
      g_desc[4] = 1;
      g_desc[5] = CAAM_OP_RNG_GEN_SK;
      words = 6;
    }

  g_desc[0] = CAAM_DESC_HDR(words);

  return imxrt_caam_run();
}

/*****************************************************************************
 * Name: imxrt_caam_rng_init
 *****************************************************************************/

static int imxrt_caam_rng_init(void)
{
  uint32_t ent_delay;
  bool gen_sk;
  int ret = -EIO;

  if ((getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_IF0) != 0)
    {
      return OK;
    }

  gen_sk = (getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_SKVN) == 0;

  for (ent_delay = CAAM_ENT_DELAY_MIN;
       ent_delay <= CAAM_ENT_DELAY_MAX;
       ent_delay += CAAM_ENT_DELAY_STEP)
    {
      int settle;

      imxrt_caam_kick_trng(ent_delay);

      ret = imxrt_caam_instantiate(gen_sk);

      for (settle = CAAM_INSTANTIATE_SETTLE; settle > 0; settle--)
        {
          if ((getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_IF0) != 0)
            {
              return OK;
            }

          up_udelay(100);
        }

      imxrt_caam_ring_init();

      if ((getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_IF0) != 0)
        {
          return OK;
        }
    }

  _err("ERROR: RNG would not instantiate\n");
  return ret < 0 ? ret : -EIO;
}

/*****************************************************************************
 * Name: imxrt_caam_ring_init
 *****************************************************************************/

static int imxrt_caam_ring_init(void)
{
  int timeout;

  putreg32(CAAM_JRCR_RESET, IMXRT_CAAM_JRCR);

  for (timeout = CAAM_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(IMXRT_CAAM_JRINT) & CAAM_JRINT_ERR_HALT_MASK) !=
          CAAM_JRINT_ERR_HALT_INPROG)
        {
          break;
        }
    }

  if ((getreg32(IMXRT_CAAM_JRINT) & CAAM_JRINT_ERR_HALT_MASK) !=
      CAAM_JRINT_ERR_HALT_DONE)
    {
      _err("ERROR: job ring would not halt\n");
      return -ETIMEDOUT;
    }

  putreg32(CAAM_JRCR_RESET, IMXRT_CAAM_JRCR);

  for (timeout = CAAM_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(IMXRT_CAAM_JRCR) & CAAM_JRCR_RESET) == 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      _err("ERROR: job ring would not reset\n");
      return -ETIMEDOUT;
    }

  memset(g_input_ring, 0, sizeof(g_input_ring));
  memset(g_output_ring, 0, sizeof(g_output_ring));
  imxrt_caam_clean(g_input_ring, sizeof(g_input_ring));
  imxrt_caam_clean(g_output_ring, sizeof(g_output_ring));

  putreg32(0, IMXRT_CAAM_IRBA_H);
  putreg32((uint32_t)(uintptr_t)g_input_ring, IMXRT_CAAM_IRBA_L);
  putreg32(0, IMXRT_CAAM_ORBA_H);
  putreg32((uint32_t)(uintptr_t)g_output_ring, IMXRT_CAAM_ORBA_L);
  putreg32(CAAM_RING_ENTRIES, IMXRT_CAAM_IRS);
  putreg32(CAAM_RING_ENTRIES, IMXRT_CAAM_ORS);

  /* Completion is polled, so the ring interrupt is never wanted. */

  modifyreg32(IMXRT_CAAM_JRCFG1, 0, CAAM_JRCFG1_IMSK);

  return OK;
}

/*****************************************************************************
 * Name: imxrt_caam_blob
 *
 * Description:
 *   Run one blob job: encapsulate inlen bytes into a blob, or decapsulate a
 *   blob back into its data. The data never touches a caller buffer that
 *   CAAM writes by DMA, and the bounce buffers are wiped afterwards.
 *
 *****************************************************************************/

static int imxrt_caam_blob(uint32_t op, const uint8_t *keymod,
                           const uint8_t *in, size_t inlen,
                           uint8_t *out, size_t outlen)
{
  int ret;

  if (keymod == NULL || in == NULL || out == NULL ||
      inlen > CAAM_BLOB_BUFLEN || outlen > CAAM_BLOB_BUFLEN)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_caam_initialize();
  if (ret < 0)
    {
      goto out;
    }

  memcpy(g_keymod, keymod, sizeof(g_keymod));
  memcpy(g_blob_in, in, inlen);
  memset(g_blob_out, 0, sizeof(g_blob_out));
  imxrt_caam_clean(g_keymod, sizeof(g_keymod));
  imxrt_caam_clean(g_blob_in, sizeof(g_blob_in));
  imxrt_caam_clean(g_blob_out, sizeof(g_blob_out));

  g_desc[0] = CAAM_DESC_HDR(8);
  g_desc[1] = CAAM_KEY_CLASS2 | sizeof(g_keymod);
  g_desc[2] = (uint32_t)(uintptr_t)g_keymod;
  g_desc[3] = CAAM_SEQ_IN_PTR | inlen;
  g_desc[4] = (uint32_t)(uintptr_t)g_blob_in;
  g_desc[5] = CAAM_SEQ_OUT_PTR | outlen;
  g_desc[6] = (uint32_t)(uintptr_t)g_blob_out;
  g_desc[7] = op;

  ret = imxrt_caam_run();
  if (ret == OK)
    {
      imxrt_caam_invalidate(g_blob_out, sizeof(g_blob_out));
      memcpy(out, g_blob_out, outlen);
    }

out:
  explicit_bzero(g_keymod, sizeof(g_keymod));
  explicit_bzero(g_blob_in, sizeof(g_blob_in));
  explicit_bzero(g_blob_out, sizeof(g_blob_out));
  imxrt_caam_clean(g_blob_in, sizeof(g_blob_in));
  imxrt_caam_clean(g_blob_out, sizeof(g_blob_out));
  nxmutex_unlock(&g_lock);
  return ret;
}

/*****************************************************************************
 * Name: imxrt_caam_random
 *****************************************************************************/

static int imxrt_caam_random(uint8_t *buffer, size_t buflen, bool reseed)
{
  size_t done = 0;
  int ret;

  ret = imxrt_caam_initialize();
  if (ret < 0)
    {
      return ret;
    }

  while (done < buflen)
    {
      size_t chunk = buflen - done;

      if (chunk > sizeof(g_rngbuf))
        {
          chunk = sizeof(g_rngbuf);
        }

      memset(g_rngbuf, 0, sizeof(g_rngbuf));
      imxrt_caam_clean(g_rngbuf, sizeof(g_rngbuf));

      g_desc[0] = CAAM_DESC_HDR(4);
      g_desc[1] = CAAM_OP_RNG_GENERATE | (reseed ? CAAM_OP_RNG_RESEED : 0);
      g_desc[2] = CAAM_FIFO_STORE_RNG | sizeof(g_rngbuf);
      g_desc[3] = (uint32_t)(uintptr_t)g_rngbuf;

      ret = imxrt_caam_run();
      if (ret < 0)
        {
          memset(buffer, 0, buflen);
          return ret;
        }

      imxrt_caam_invalidate(g_rngbuf, sizeof(g_rngbuf));
      memcpy(buffer + done, g_rngbuf, chunk);
      done += chunk;
    }

  /* Leave nothing behind for the next caller to find. */

  memset(g_rngbuf, 0, sizeof(g_rngbuf));
  return OK;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: imxrt_caam_initialize
 *****************************************************************************/

int imxrt_caam_initialize(void)
{
  int ret;

  if (g_initialized)
    {
      return OK;
    }

  imxrt_clockall_caam();

  modifyreg32(IMXRT_CAAM_MCFGR, CAAM_MCFGR_AWCACHE_MASK,
              CAAM_MCFGR_AWCACHE_CACH | CAAM_MCFGR_AWCACHE_BUFF |
              CAAM_MCFGR_WDE | CAAM_MCFGR_LARGE_BURST);

  modifyreg32(IMXRT_CAAM_JRSTART, 0, CAAM_JRSTART_JR0);

  ret = imxrt_caam_ring_init();
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_caam_rng_init();
  if (ret < 0)
    {
      return ret;
    }

  g_initialized = true;
  return OK;
}

/*****************************************************************************
 * Name: imxrt_caam_get_random
 *****************************************************************************/

int imxrt_caam_get_random(uint8_t *buffer, size_t buflen, bool reseed)
{
  int ret;

  if (buffer == NULL || buflen == 0)
    {
      return -EINVAL;
    }

  if (!OSINIT_TASK_READY())
    {
      return imxrt_caam_random(buffer, buflen, reseed);
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_caam_random(buffer, buflen, reseed);
  nxmutex_unlock(&g_lock);
  return ret;
}

/*****************************************************************************
 * Name: imxrt_caam_blob_encap
 *****************************************************************************/

int imxrt_caam_blob_encap(const uint8_t *keymod, const uint8_t *data,
                          size_t len, uint8_t *blob)
{
  if (len == 0 || len > IMXRT_CAAM_BLOB_MAX)
    {
      return -EINVAL;
    }

  return imxrt_caam_blob(CAAM_OP_BLOB_ENCAP, keymod, data, len, blob,
                         len + IMXRT_CAAM_BLOB_OVERHEAD);
}

/*****************************************************************************
 * Name: imxrt_caam_blob_decap
 *****************************************************************************/

int imxrt_caam_blob_decap(const uint8_t *keymod, const uint8_t *blob,
                          size_t len, uint8_t *data)
{
  if (len == 0 || len > IMXRT_CAAM_BLOB_MAX)
    {
      return -EINVAL;
    }

  return imxrt_caam_blob(CAAM_OP_BLOB_DECAP, keymod, blob,
                         len + IMXRT_CAAM_BLOB_OVERHEAD, data, len);
}

#endif /* CONFIG_IMXRT_CAAM */
