#include "noise_backend.h"

#include <px4_platform_common/crypto_backend.h>
#include <px4_random.h>

#include <string.h>

int noise_random(uint8_t *out, size_t len)
{
  return px4_get_secure_random(out, len) == len ? 0 : -1;
}

int noise_dh_static(const struct noise_static_key *s,
                    const uint8_t pk[NOISE_DHLEN], uint8_t out[NOISE_DHLEN])
{
  crypto_session_handle_t h = crypto_open(CRYPTO_X25519);
  size_t len = NOISE_DHLEN;
  bool ok = crypto_session_handle_valid(h)
            && crypto_key_agreement(h, s->index, pk, NOISE_DHLEN, out, &len)
            && len == NOISE_DHLEN;

  crypto_close(&h);

  if (!ok)
    {
      memset(out, 0, NOISE_DHLEN);
      return -1;
    }

  return 0;
}

int noise_static_public(const struct noise_static_key *s,
                        uint8_t pk[NOISE_DHLEN])
{
  crypto_session_handle_t h = crypto_open(CRYPTO_X25519);
  size_t len = NOISE_DHLEN;
  bool ok = crypto_session_handle_valid(h)
            && crypto_get_public_key(h, s->index, pk, &len)
            && len == NOISE_DHLEN;

  crypto_close(&h);

  if (!ok)
    {
      memset(pk, 0, NOISE_DHLEN);
      return -1;
    }

  return 0;
}

int noise_session_key_set(struct noise_session_key *k,
                          const uint8_t key[NOISE_KEYLEN])
{
  crypto_session_handle_t h = crypto_open(CRYPTO_CHACHA20_POLY1305);
  int rc = -1;

  for (uint8_t i = 0; crypto_session_handle_valid(h)
       && i < CRYPTO_SESSION_KEY_COUNT; i++)
    {
      const uint8_t index = CRYPTO_SESSION_KEY_FIRST + i;

      if (crypto_set_key(h, 0, NULL, key, NOISE_KEYLEN, index))
        {
          k->index = index;
          k->backend = NULL;
          rc = 0;
          break;
        }
    }

  crypto_close(&h);
  return rc;
}

void noise_session_key_clear(struct noise_session_key *k)
{
  crypto_session_handle_t h = crypto_open(CRYPTO_CHACHA20_POLY1305);

  if (crypto_session_handle_valid(h) && k->index != 0)
    {
      crypto_set_key(h, 0, NULL, NULL, 0, k->index);
    }

  crypto_close(&h);
  k->index = 0;
  k->backend = NULL;
}

static bool with_nonce(crypto_session_handle_t h, uint64_t n)
{
  uint8_t iv[12] = {0};

  for (int i = 0; i < 8; i++)
    {
      iv[4 + i] = (uint8_t)(n >> (8 * i));
    }

  return crypto_session_handle_valid(h) && crypto_renew_nonce(h, iv, sizeof(iv));
}

int noise_session_encrypt(const struct noise_session_key *k, uint64_t nonce,
                          const uint8_t *pt, size_t pt_len, uint8_t *out)
{
  crypto_session_handle_t h = crypto_open(CRYPTO_CHACHA20_POLY1305);
  size_t ct_len = pt_len;
  size_t tag_len = NOISE_TAGLEN;
  bool ok = with_nonce(h, nonce)
            && crypto_encrypt_data(h, k->index, pt, pt_len, out, &ct_len,
                                   out + pt_len, &tag_len);

  crypto_close(&h);
  return ok ? 0 : -1;
}

int noise_session_decrypt(const struct noise_session_key *k, uint64_t nonce,
                          const uint8_t *ct, size_t ct_len, uint8_t *out)
{
  crypto_session_handle_t h;
  size_t pt_len;
  bool ok;

  if (ct_len < NOISE_TAGLEN)
    {
      return -1;
    }

  h = crypto_open(CRYPTO_CHACHA20_POLY1305);
  pt_len = ct_len - NOISE_TAGLEN;
  ok = with_nonce(h, nonce)
       && crypto_decrypt_data(h, k->index, ct, pt_len, ct + pt_len,
                              NOISE_TAGLEN, out, &pt_len);

  crypto_close(&h);
  return ok ? 0 : -1;
}
