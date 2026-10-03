/****************************************************************************
 * drivers/crypto/pnt/pnt_se05x_api.c
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

/* Copyright 2023 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pnt_se05x_api.h"

#include "../se05x_internal.h"
#include "pnt_util.h"
#include <nuttx/kmalloc.h>
#include <strings.h>
#include <sys/param.h>
#include <phNxpEse_Internal.h>
#include <se05x_APDU_apis.h>
#include <smCom.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATA_CHUNK_SIZE 100

#define SE05X_ECCURVE_ED25519       ((SE05x_ECCurve_t)0x40)
#define SE05X_ECCURVE_MONT_DH_25519 ((SE05x_ECCurve_t)0x41)
#define SE05X_ALGO_ED25519PURE      ((SE05x_ECSignatureAlgo_t)0xa3)
#define SE05X_P256_PUBLIC_SIZE      65
#define SE05X_25519_SIZE            32

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pnt_handle
{
  Se05xSession_t session;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const SE05x_ECSignatureAlgo_t
    signature_algorithm_mapping[SE05X_ALGORITHM_SIZE] =
{
        kSE05x_ECSignatureAlgo_NA,      kSE05x_ECSignatureAlgo_PLAIN,
        kSE05x_ECSignatureAlgo_SHA,     kSE05x_ECSignatureAlgo_SHA_224,
        kSE05x_ECSignatureAlgo_SHA_256, kSE05x_ECSignatureAlgo_SHA_384,
        kSE05x_ECSignatureAlgo_SHA_512, SE05X_ALGO_ED25519PURE
};

static const SE05x_ECCurve_t curve_mapping[] =
{
  kSE05x_ECCurve_NIST_P256, SE05X_ECCURVE_ED25519,
  SE05X_ECCURVE_MONT_DH_25519
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void reverse(FAR uint8_t *out, FAR const uint8_t *in, size_t len)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      out[i] = in[len - 1 - i];
    }
}

static void reverse_in_place(FAR uint8_t *buf, size_t len)
{
  size_t i;

  for (i = 0; i < len / 2; i++)
    {
      uint8_t t = buf[i];
      buf[i] = buf[len - 1 - i];
      buf[len - 1 - i] = t;
    }
}

static bool all_zero(FAR const uint8_t *buf, size_t len)
{
  uint8_t acc = 0;
  size_t i;

  for (i = 0; i < len; i++)
    {
      acc |= buf[i];
    }

  return acc == 0;
}

static bool set_enable_pin(FAR struct se05x_dev_s *se05x, bool state)
{
  if (se05x->config->set_enable_pin == NULL)
    {
      return TRUE;
    }

  return se05x->config->set_enable_pin(state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pnt_se05x_open(FAR struct se05x_dev_s *se05x)
{
#ifdef CONFIG_DEV_SE05X_SCP03
  FAR const struct se05x_scp03_keys_s *keys = se05x->config->scp03;
#endif
  int ret;

#ifdef CONFIG_DEV_SE05X_SCP03
  if (keys == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }
#endif

  se05x->pnt = kmm_zalloc(sizeof(struct pnt_handle));

  if (se05x->pnt == NULL)
    {
      ret = -EIO;
      goto errout;
    }

#ifdef CONFIG_DEV_SE05X_SCP03
  se05x->pnt->session.pScp03_enc_key = (FAR uint8_t *)keys->enc;
  se05x->pnt->session.pScp03_mac_key = (FAR uint8_t *)keys->mac;
  se05x->pnt->session.pScp03_dek_key = (FAR uint8_t *)keys->dek;
  se05x->pnt->session.scp03_enc_key_len = sizeof(keys->enc);
  se05x->pnt->session.scp03_mac_key_len = sizeof(keys->mac);
  se05x->pnt->session.scp03_dek_key_len = sizeof(keys->dek);
#endif

  if (!set_enable_pin(se05x, true))
    {
      ret = -EIO;
      goto errout_with_alloc;
    }

  se05x->pnt->session.skip_applet_select = 0;
  se05x->pnt->session.session_resume = 0;
  if (Se05x_API_SessionOpen(&(se05x->pnt->session), se05x) != SM_OK)
    {
      ret = -EIO;
      goto errout_with_alloc;
    }

  return OK;

errout_with_alloc:
  if (se05x->pnt->session.conn_context != NULL)
    {
      kmm_free(se05x->pnt->session.conn_context);
    }

  explicit_bzero(se05x->pnt, sizeof(struct pnt_handle));
  kmm_free(se05x->pnt);

errout:
  return ret;
}

void pnt_se05x_close(FAR struct se05x_dev_s *se05x)
{
  Se05x_API_SessionClose(&(se05x->pnt->session));
  (void)set_enable_pin(se05x, FALSE);
  explicit_bzero(se05x->pnt, sizeof(struct pnt_handle));
  kmm_free(se05x->pnt);
}

int pnt_se05x_get_info(FAR struct se05x_dev_s *se05x,
                       FAR struct se05x_info_s *se05x_info)
{
  bool result = select_card_manager(&(se05x->pnt->session));
  identify_rsp_t identify_response;
  if (result)
    {
      result = se05x_identify(&(se05x->pnt->session), &identify_response);
    }

  if (result)
    {
      se05x_info->oef_id = (identify_response.configuration_id[2] << 8) +
                           identify_response.configuration_id[3];
    }

  return result ? 0 : -EIO;
}

int pnt_se05x_get_version(FAR struct se05x_dev_s *se05x,
                          FAR struct se05x_version_s *version)
{
  uint8_t raw[7];
  size_t len = sizeof(raw);

  if (Se05x_API_GetVersion(&(se05x->pnt->session), raw, &len) != SM_OK ||
      len < sizeof(raw))
    {
      return -EIO;
    }

  version->major = raw[0];
  version->minor = raw[1];
  version->patch = raw[2];
  version->applet_config = (raw[3] << 8) | raw[4];
  version->secure_box = (raw[5] << 8) | raw[6];
  return 0;
}

int pnt_se05x_get_uid(FAR struct se05x_dev_s *se05x,
                      FAR struct se05x_uid_s *uid)
{
  SE05x_Result_t dummy = kSE05x_Result_NA;
  size_t uid_size = SE050_MODULE_UNIQUE_ID_LEN;

  smStatus_t status = Se05x_API_CheckObjectExists(
      &(se05x->pnt->session), KSE05X_APPLETRESID_UNIQUE_ID, &dummy);
  int result = status == SM_OK ? 0 : -ENODATA;
  if (result == 0)
    {
      status = Se05x_API_ReadObject(&(se05x->pnt->session),
                                    KSE05X_APPLETRESID_UNIQUE_ID, 0,
                                    (uint16_t)uid_size, uid->uid, &uid_size);
      result = status == SM_OK ? 0 : -EIO;
    }

  return result;
}

int pnt_se05x_generate_keypair(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_generate_keypair_s *generate_keypair_args)
{
  SE05x_Result_t exists = kSE05x_Result_NA;
  uint32_t rule = generate_keypair_args->policy;
  uint8_t policy_buf[9];
  Se05xPolicy_t policy;
  SE05x_ECCurve_t curve;
  smStatus_t status;

  if (generate_keypair_args->cipher >= nitems(curve_mapping))
    {
      return -EINVAL;
    }

  curve = curve_mapping[generate_keypair_args->cipher];
  policy_buf[0] = sizeof(policy_buf) - 1;
  memset(&policy_buf[1], 0, 4);
  policy_buf[5] = rule >> 24;
  policy_buf[6] = rule >> 16;
  policy_buf[7] = rule >> 8;
  policy_buf[8] = rule;
  policy.value = policy_buf;
  policy.value_len = sizeof(policy_buf);

  status = Se05x_API_CheckObjectExists(&(se05x->pnt->session),
                                       generate_keypair_args->id, &exists);

  if (status != SM_OK)
    {
      return -EIO;
    }

  if (exists == kSE05x_Result_SUCCESS)
    {
      return -EEXIST;
    }

  status = Se05x_API_WriteECKey(
      &(se05x->pnt->session), rule != 0 ? &policy : NULL, 0,
      generate_keypair_args->id, curve, NULL, 0, NULL, 0, kSE05x_INS_NA,
      kSE05x_KeyPart_Pair);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_set_public_key(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *set_publickey_args)
{
  smStatus_t status = Se05x_API_WriteECKey(
      &(se05x->pnt->session), NULL, 0, set_publickey_args->entry.id,
      kSE05x_ECCurve_NIST_P256, NULL, 0, set_publickey_args->content.buffer,
      set_publickey_args->content.buffer_size, kSE05x_INS_NA,
      kSE05x_KeyPart_Public);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_set_data(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *set_publickey_args)
{
  size_t remainder = set_publickey_args->content.buffer_size;
  smStatus_t status = SM_OK;
  uint16_t offset = 0;
  bool first_cycle = TRUE;

  while ((remainder > 0) && (status == SM_OK))
    {
      size_t chunk_size =
          remainder > DATA_CHUNK_SIZE ? DATA_CHUNK_SIZE : remainder;
      status = Se05x_API_WriteBinary(
          &(se05x->pnt->session), NULL, set_publickey_args->entry.id, offset,
          first_cycle ? set_publickey_args->content.buffer_size : 0,
          set_publickey_args->content.buffer + offset, chunk_size);
      remainder -= chunk_size;
      offset += chunk_size;
      first_cycle = FALSE;
    }

  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_get_key(FAR struct se05x_dev_s *se05x,
                      FAR struct se05x_key_transmission_s *get_key_args)
{
  se05x_asym_cipher_type_e cipher = get_key_args->entry.cipher;
  size_t want = cipher == SE05X_ASYM_CIPHER_EC_NIST_P_256 ?
                SE05X_P256_PUBLIC_SIZE : SE05X_25519_SIZE;
  smStatus_t status;

  if (cipher >= nitems(curve_mapping))
    {
      return -EINVAL;
    }

  get_key_args->content.buffer_content_size =
      get_key_args->content.buffer_size;
  status = Se05x_API_ReadObject(&(se05x->pnt->session),
                                get_key_args->entry.id, 0, 0,
                                get_key_args->content.buffer,
                                &get_key_args->content.buffer_content_size);
  if (status != SM_OK)
    {
      return -EIO;
    }

  if (get_key_args->content.buffer_content_size != want)
    {
      return -EINVAL;
    }

  if (cipher != SE05X_ASYM_CIPHER_EC_NIST_P_256)
    {
      reverse_in_place(get_key_args->content.buffer, want);
    }

  return 0;
}

int pnt_se05x_get_data(FAR struct se05x_dev_s *se05x,
                       FAR struct se05x_key_transmission_s *get_key_args)
{
  uint16_t remainder;
  smStatus_t status = Se05x_API_ReadSize(&(se05x->pnt->session),
                                         get_key_args->entry.id, &remainder);

  if (remainder > get_key_args->content.buffer_size)
    {
      status = SM_NOT_OK;
    }

  uint16_t offset = 0;

  while ((remainder > 0) && (status == SM_OK))
    {
      size_t chunk_size =
          remainder > DATA_CHUNK_SIZE ? DATA_CHUNK_SIZE : remainder;
      status = Se05x_API_ReadObject(
          &(se05x->pnt->session), get_key_args->entry.id, offset, chunk_size,
          get_key_args->content.buffer + offset, &chunk_size);
      remainder -= chunk_size;
      offset += chunk_size;
    }

  get_key_args->content.buffer_content_size = offset;
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_delete_key(FAR struct se05x_dev_s *se05x, uint32_t key_id)
{
  smStatus_t status =
      Se05x_API_DeleteSecureObject(&(se05x->pnt->session), key_id);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_derive_key(FAR struct se05x_dev_s *se05x,
                         FAR struct se05x_derive_key_s *derive_key_args)
{
  uint8_t public_key[SE05X_P256_PUBLIC_SIZE];
  size_t public_key_size = sizeof(public_key);
  FAR const uint8_t *peer = derive_key_args->public_key.buffer;
  FAR uint8_t *secret = derive_key_args->content.buffer;
  smStatus_t status;

  if (peer == NULL)
    {
      status = Se05x_API_ReadObject(&(se05x->pnt->session),
                                    derive_key_args->public_key_id, 0, 0,
                                    public_key, &public_key_size);
      if (status != SM_OK)
        {
          return -EIO;
        }
    }
  else
    {
      public_key_size = derive_key_args->public_key.buffer_content_size;
      if (public_key_size == SE05X_25519_SIZE)
        {
          reverse(public_key, peer, public_key_size);
        }
      else if (public_key_size == SE05X_P256_PUBLIC_SIZE)
        {
          memcpy(public_key, peer, public_key_size);
        }
      else
        {
          return -EINVAL;
        }
    }

  derive_key_args->content.buffer_content_size =
      derive_key_args->content.buffer_size;
  status = Se05x_API_ECDHGenerateSharedSecret(
      &(se05x->pnt->session), derive_key_args->private_key_id, public_key,
      public_key_size, secret,
      &derive_key_args->content.buffer_content_size);
  if (status != SM_OK)
    {
      return -EIO;
    }

  if (public_key_size == SE05X_25519_SIZE)
    {
      if (derive_key_args->content.buffer_content_size != SE05X_25519_SIZE)
        {
          return -EIO;
        }

      reverse_in_place(secret, SE05X_25519_SIZE);
      if (all_zero(secret, SE05X_25519_SIZE))
        {
          return -EINVAL;
        }
    }

  return 0;
}

int pnt_se05x_create_signature(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_signature_s *create_signature_args)
{
  FAR struct se05x_buffer_s *signature = &create_signature_args->signature;
  smStatus_t status;

  if (create_signature_args->algorithm >= SE05X_ALGORITHM_SIZE)
    {
      return -EINVAL;
    }

  signature->buffer_content_size = signature->buffer_size;
  status = Se05x_API_ECDSASign(
      &(se05x->pnt->session), create_signature_args->key_id,
      signature_algorithm_mapping[create_signature_args->algorithm],
      create_signature_args->tbs.buffer,
      create_signature_args->tbs.buffer_content_size, signature->buffer,
      &signature->buffer_content_size);
  if (status != SM_OK)
    {
      return -EIO;
    }

  if (create_signature_args->algorithm == SE05X_ALGORITHM_ED25519)
    {
      if (signature->buffer_content_size != 2 * SE05X_25519_SIZE)
        {
          return -EIO;
        }

      reverse_in_place(signature->buffer, SE05X_25519_SIZE);
      reverse_in_place(signature->buffer + SE05X_25519_SIZE,
                       SE05X_25519_SIZE);
    }

  return 0;
}

int pnt_se05x_verify_signature(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_signature_s *verify_signature_args)
{
  FAR struct se05x_buffer_s *signature = &verify_signature_args->signature;
  FAR const uint8_t *sig = signature->buffer;
  uint8_t ed25519[2 * SE05X_25519_SIZE];
  SE05x_Result_t se05x_result;

  if (verify_signature_args->algorithm >= SE05X_ALGORITHM_SIZE)
    {
      return -EINVAL;
    }

  if (verify_signature_args->algorithm == SE05X_ALGORITHM_ED25519)
    {
      if (signature->buffer_content_size != sizeof(ed25519))
        {
          return -EINVAL;
        }

      reverse(ed25519, sig, SE05X_25519_SIZE);
      reverse(ed25519 + SE05X_25519_SIZE, sig + SE05X_25519_SIZE,
              SE05X_25519_SIZE);
      sig = ed25519;
    }

  if (Se05x_API_ECDSAVerify(
          &(se05x->pnt->session), verify_signature_args->key_id,
          signature_algorithm_mapping[verify_signature_args->algorithm],
          verify_signature_args->tbs.buffer,
          verify_signature_args->tbs.buffer_content_size, sig,
          signature->buffer_content_size, &se05x_result) != SM_OK)
    {
      return -EACCES;
    }

  return se05x_result == kSE05x_Result_SUCCESS ? 0 : -EIO;
}
