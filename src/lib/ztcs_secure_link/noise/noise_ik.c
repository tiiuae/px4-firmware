#include "noise_ik.h"

#include <string.h>

#ifndef NOISE_HANDSHAKE_IN_KERNEL
#ifdef NOISE_HFS
static const char PROTOCOL[] = "Noise_IKhfs_25519+MLKEM768_ChaChaPoly_SHA256";
#else
static const char PROTOCOL[] = "Noise_IK_25519_ChaChaPoly_SHA256";
#endif

#define PROTOCOL_LEN (sizeof(PROTOCOL) - 1)

#ifdef NOISE_HFS
#define KEM_FIELD_LEN (NOISE_KEM_CTLEN + NOISE_TAGLEN)
#else
#define KEM_FIELD_LEN 0
#endif

static void mix_hash(struct noise_symmetric *ss, const uint8_t *data,
                     size_t len) {
  noise_sha256_2(ss->h, NOISE_HASHLEN, data, len, ss->h);
}

static void hkdf2(const uint8_t ck[NOISE_HASHLEN], const uint8_t *ikm,
                  size_t ikm_len, uint8_t out1[NOISE_HASHLEN],
                  uint8_t out2[NOISE_HASHLEN]) {
  uint8_t temp[NOISE_HASHLEN];
  uint8_t buf[NOISE_HASHLEN + 1];

  noise_hmac_sha256(ck, NOISE_HASHLEN, ikm, ikm_len, temp);

  buf[0] = 1;
  noise_hmac_sha256(temp, NOISE_HASHLEN, buf, 1, out1);

  memcpy(buf, out1, NOISE_HASHLEN);
  buf[NOISE_HASHLEN] = 2;
  noise_hmac_sha256(temp, NOISE_HASHLEN, buf, NOISE_HASHLEN + 1, out2);

  noise_wipe(temp, sizeof(temp));
  noise_wipe(buf, sizeof(buf));
}

static void mix_key(struct noise_symmetric *ss, const uint8_t *ikm,
                    size_t ikm_len) {
  uint8_t ck[NOISE_HASHLEN];
  hkdf2(ss->ck, ikm, ikm_len, ck, ss->k);
  memcpy(ss->ck, ck, NOISE_HASHLEN);
  ss->n = 0;
  ss->has_key = 1;
  noise_wipe(ck, sizeof(ck));
}

/* Noise: a name of HASHLEN bytes or fewer is the hash, zero-padded; a longer
 * one is hashed. The classical name is exactly 32 and the hybrid one is not,
 * and getting this wrong is silent until the station disagrees.
 */
static void symmetric_init(struct noise_symmetric *ss) {
  memset(ss, 0, sizeof(*ss));
  if (PROTOCOL_LEN <= NOISE_HASHLEN) {
    memcpy(ss->h, PROTOCOL, PROTOCOL_LEN);
  } else {
    noise_sha256((const uint8_t *)PROTOCOL, PROTOCOL_LEN, ss->h);
  }
  memcpy(ss->ck, ss->h, NOISE_HASHLEN);
  /* Empty prologue, but the MixHash still runs. */
  mix_hash(ss, (const uint8_t *)"", 0);
}

static void encrypt_and_hash(struct noise_symmetric *ss, const uint8_t *pt,
                             size_t pt_len, uint8_t *out) {
  noise_aead_encrypt(ss->k, ss->n, ss->h, NOISE_HASHLEN, pt, pt_len, out);
  ss->n++;
  mix_hash(ss, out, pt_len + NOISE_TAGLEN);
}

static int decrypt_and_hash(struct noise_symmetric *ss, const uint8_t *ct,
                            size_t ct_len, uint8_t *out) {
  uint8_t saved[NOISE_HASHLEN];
  memcpy(saved, ss->h, NOISE_HASHLEN);
  if (noise_aead_decrypt(ss->k, ss->n, saved, NOISE_HASHLEN, ct, ct_len, out) !=
      0) {
    return NOISE_ERR_DECRYPT;
  }
  ss->n++;
  mix_hash(ss, ct, ct_len);
  return NOISE_OK;
}

size_t noise_static_key_signing_input(const uint8_t x25519_public[32],
                                      uint8_t *out) {
  size_t ctx_len = sizeof(NOISE_STATIC_KEY_CONTEXT) - 1;
  memcpy(out, NOISE_STATIC_KEY_CONTEXT, ctx_len);
  memcpy(out + ctx_len, x25519_public, 32);
  return ctx_len + 32;
}

int noise_initiator_start(struct noise_initiator *ini,
                          const struct noise_static_key *s,
                          const uint8_t rs_pub[NOISE_DHLEN],
                          const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN],
                          uint8_t *out, size_t *out_len) {
  uint8_t dh[NOISE_DHLEN];
  uint8_t *p;

  memset(ini, 0, sizeof(*ini));
  symmetric_init(&ini->ss);

  ini->s = s;
  if (noise_static_public(s, ini->s_pub) != 0) {
    return NOISE_ERR_DH;
  }

  /* Pre-message: the responder's static key is known, which is what IK is. */
  mix_hash(&ini->ss, rs_pub, NOISE_DHLEN);

  if (noise_random(ini->e_priv, NOISE_DHLEN) != 0) {
    return NOISE_ERR_RANDOM;
  }
  noise_dh_public(ini->e_priv, ini->e_pub);

  p = out;
  *p++ = NOISE_TYPE_HANDSHAKE_INIT;

  memcpy(p, ini->e_pub, NOISE_DHLEN);
  mix_hash(&ini->ss, ini->e_pub, NOISE_DHLEN);
  p += NOISE_DHLEN;

  if (noise_dh(ini->e_priv, rs_pub, dh) != 0) {
    return NOISE_ERR_DH;
  }
  mix_key(&ini->ss, dh, NOISE_DHLEN);

#ifdef NOISE_HFS
  {
    uint8_t kem_pub[NOISE_KEM_PUBLEN];
    if (noise_random(ini->kem_seed, NOISE_KEM_SEEDLEN) != 0) {
      return NOISE_ERR_RANDOM;
    }
    if (noise_kem_public(ini->kem_seed, kem_pub) != 0) {
      return NOISE_ERR_BACKEND;
    }
    encrypt_and_hash(&ini->ss, kem_pub, NOISE_KEM_PUBLEN, p);
    p += NOISE_KEM_PUBLEN + NOISE_TAGLEN;
  }
#endif

  encrypt_and_hash(&ini->ss, ini->s_pub, NOISE_DHLEN, p);
  p += NOISE_DHLEN + NOISE_TAGLEN;

  if (noise_dh_static(ini->s, rs_pub, dh) != 0) {
    return NOISE_ERR_DH;
  }
  mix_key(&ini->ss, dh, NOISE_DHLEN);

  encrypt_and_hash(&ini->ss, identity, NOISE_IDENTITY_PAYLOAD_LEN, p);
  p += NOISE_IDENTITY_PAYLOAD_LEN + NOISE_TAGLEN;

  noise_wipe(dh, sizeof(dh));
  ini->stage = 1;
  *out_len = (size_t)(p - out);
  return NOISE_OK;
}

int noise_initiator_finish(struct noise_initiator *ini, const uint8_t *frame,
                           size_t frame_len, struct noise_session *out) {
  struct noise_symmetric ss;
  uint8_t dh[NOISE_DHLEN];
  uint8_t re[NOISE_DHLEN];
  uint8_t send[NOISE_KEYLEN];
  uint8_t recv[NOISE_KEYLEN];
  uint8_t empty[1];
  int rc;

  if (ini->stage != 1) {
    return NOISE_ERR_STATE;
  }
  if (frame_len != NOISE_MSG2_LEN) {
    return NOISE_ERR_INPUT;
  }
  if (frame[0] != NOISE_TYPE_HANDSHAKE_RESP) {
    return NOISE_ERR_INPUT;
  }

  ss = ini->ss;
  memcpy(re, frame + 1, NOISE_DHLEN);
  mix_hash(&ss, re, NOISE_DHLEN);

  rc = NOISE_ERR_DH;
  if (noise_dh(ini->e_priv, re, dh) != 0) {
    goto out;
  }
  mix_key(&ss, dh, NOISE_DHLEN);

#ifdef NOISE_HFS
  {
    uint8_t kem_ct[NOISE_KEM_CTLEN];
    uint8_t kem_ss[NOISE_KEM_SSLEN];
    rc = decrypt_and_hash(&ss, frame + 1 + NOISE_DHLEN,
                          NOISE_KEM_CTLEN + NOISE_TAGLEN, kem_ct);
    if (rc == NOISE_OK && noise_kem_decap(ini->kem_seed, kem_ct, kem_ss) != 0) {
      rc = NOISE_ERR_BACKEND;
    }
    if (rc == NOISE_OK) {
      mix_key(&ss, kem_ss, NOISE_KEM_SSLEN);
    }
    noise_wipe(kem_ss, sizeof(kem_ss));
    if (rc != NOISE_OK) {
      goto out;
    }
  }
#endif

  if (noise_dh_static(ini->s, re, dh) != 0) {
    rc = NOISE_ERR_DH;
    goto out;
  }
  mix_key(&ss, dh, NOISE_DHLEN);

  rc = decrypt_and_hash(&ss, frame + 1 + NOISE_DHLEN + KEM_FIELD_LEN,
                        NOISE_TAGLEN, empty);
  if (rc != NOISE_OK) {
    goto out;
  }

  hkdf2(ss.ck, NULL, 0, send, recv);

  memset(out, 0, sizeof(*out));
  rc = noise_session_key_set(&out->send, send);
  if (rc == 0) {
    rc = noise_session_key_set(&out->recv, recv);
  }
  if (rc != 0) {
    noise_session_wipe(out);
    rc = NOISE_ERR_BACKEND;
    goto out;
  }

  noise_wipe(&ini->ss, sizeof(ini->ss));
  noise_wipe(ini->e_priv, sizeof(ini->e_priv));
#ifdef NOISE_HFS
  noise_wipe(ini->kem_seed, sizeof(ini->kem_seed));
#endif
  ini->stage = 2;
  rc = NOISE_OK;

out:
  noise_wipe(&ss, sizeof(ss));
  noise_wipe(send, sizeof(send));
  noise_wipe(recv, sizeof(recv));
  noise_wipe(dh, sizeof(dh));
  return rc;
}

#endif

static void put_be64(uint8_t *p, uint64_t v) {
  for (int i = 0; i < 8; i++) {
    p[i] = (uint8_t)(v >> (56 - 8 * i));
  }
}

static uint64_t get_be64(const uint8_t *p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) {
    v = (v << 8) | p[i];
  }
  return v;
}

int noise_session_seal(struct noise_session *s, const uint8_t *pt, size_t pt_len,
                       uint8_t *out, size_t *out_len) {
  if (s->tx == UINT64_MAX) {
    return NOISE_ERR_EXHAUSTED;
  }
  out[0] = NOISE_TYPE_TRANSPORT;
  put_be64(out + 1, s->tx);
  if (noise_session_encrypt(&s->send, s->tx, pt, pt_len,
                            out + NOISE_TRANSPORT_HDR_LEN) != 0) {
    return NOISE_ERR_BACKEND;
  }
  s->tx++;
  *out_len = NOISE_TRANSPORT_HDR_LEN + pt_len + NOISE_TAGLEN;
  return NOISE_OK;
}

static int replay_accept(struct noise_session *s, uint64_t counter) {
  uint64_t back;
  uint64_t bit;

  if (counter == UINT64_MAX) {
    return NOISE_ERR_EXHAUSTED;
  }
  if (!s->rx_started) {
    s->rx_started = 1;
    s->rx_highest = counter;
    return NOISE_OK;
  }
  if (counter > s->rx_highest) {
    uint64_t shift = counter - s->rx_highest;
    s->rx_bitmap = shift >= 64 ? 0
                               : (s->rx_bitmap << shift) | (1ULL << (shift - 1));
    s->rx_highest = counter;
    return NOISE_OK;
  }
  if (counter == s->rx_highest) {
    return NOISE_ERR_REPLAY;
  }
  back = s->rx_highest - counter;
  if (back > 64) {
    return NOISE_ERR_REPLAY;
  }
  bit = 1ULL << (back - 1);
  if (s->rx_bitmap & bit) {
    return NOISE_ERR_REPLAY;
  }
  s->rx_bitmap |= bit;
  return NOISE_OK;
}

int noise_session_open(struct noise_session *s, const uint8_t *frame,
                       size_t frame_len, uint8_t *out, size_t *out_len) {
  uint64_t counter;
  size_t ct_len;
  int rc;

  if (frame_len < NOISE_TRANSPORT_HDR_LEN + NOISE_TAGLEN) {
    return NOISE_ERR_INPUT;
  }
  if (frame[0] != NOISE_TYPE_TRANSPORT) {
    return NOISE_ERR_INPUT;
  }
  counter = get_be64(frame + 1);
  ct_len = frame_len - NOISE_TRANSPORT_HDR_LEN;

  if (noise_session_decrypt(&s->recv, counter, frame + NOISE_TRANSPORT_HDR_LEN,
                            ct_len, out) != 0) {
    return NOISE_ERR_DECRYPT;
  }
  rc = replay_accept(s, counter);
  if (rc != NOISE_OK) {
    return rc;
  }
  *out_len = ct_len - NOISE_TAGLEN;
  return NOISE_OK;
}

void noise_session_wipe(struct noise_session *s) {
  noise_session_key_clear(&s->send);
  noise_session_key_clear(&s->recv);
  noise_wipe(s, sizeof(*s));
}

#ifndef NOISE_SESSION_KEY_BY_INDEX
int noise_session_key_set(struct noise_session_key *k,
                          const uint8_t key[NOISE_KEYLEN]) {
  memcpy(k->k, key, NOISE_KEYLEN);
  return 0;
}

void noise_session_key_clear(struct noise_session_key *k) {
  noise_wipe(k->k, sizeof(k->k));
}

int noise_session_encrypt(const struct noise_session_key *k, uint64_t nonce,
                          const uint8_t *pt, size_t pt_len, uint8_t *out) {
  noise_aead_encrypt(k->k, nonce, NULL, 0, pt, pt_len, out);
  return 0;
}

int noise_session_decrypt(const struct noise_session_key *k, uint64_t nonce,
                          const uint8_t *ct, size_t ct_len, uint8_t *out) {
  return noise_aead_decrypt(k->k, nonce, NULL, 0, ct, ct_len, out);
}
#endif
