#include "noise_ik.h"

#include <string.h>

/* Exactly NOISE_HASHLEN bytes, so h starts as the name itself with no hash. */
static const char PROTOCOL[] = "Noise_IK_25519_ChaChaPoly_SHA256";

/* The largest thing this protocol ever hashes is a sealed identity payload. */
#define MIX_HASH_MAX (NOISE_IDENTITY_PAYLOAD_LEN + NOISE_TAGLEN)

static void mix_hash(struct noise_symmetric *ss, const uint8_t *data,
                     size_t len) {
  uint8_t buf[NOISE_HASHLEN + MIX_HASH_MAX];
  if (len > MIX_HASH_MAX) {
    /* Skipping the update would leave a stale h and carry on. Poison it so
     * the handshake cannot complete instead.
     */
    memset(ss->h, 0, NOISE_HASHLEN);
    return;
  }
  memcpy(buf, ss->h, NOISE_HASHLEN);
  memcpy(buf + NOISE_HASHLEN, data, len);
  noise_sha256(buf, NOISE_HASHLEN + len, ss->h);
  noise_wipe(buf, sizeof(buf));
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

static void symmetric_init(struct noise_symmetric *ss) {
  memset(ss, 0, sizeof(*ss));
  memcpy(ss->h, PROTOCOL, NOISE_HASHLEN);
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
                          const uint8_t s_priv[NOISE_DHLEN],
                          const uint8_t rs_pub[NOISE_DHLEN],
                          const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN],
                          uint8_t *out, size_t *out_len) {
  uint8_t dh[NOISE_DHLEN];
  uint8_t *p;

  memset(ini, 0, sizeof(*ini));
  symmetric_init(&ini->ss);

  memcpy(ini->s_priv, s_priv, NOISE_DHLEN);
  noise_dh_public(ini->s_priv, ini->s_pub);

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

  encrypt_and_hash(&ini->ss, ini->s_pub, NOISE_DHLEN, p);
  p += NOISE_DHLEN + NOISE_TAGLEN;

  if (noise_dh(ini->s_priv, rs_pub, dh) != 0) {
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
  uint8_t dh[NOISE_DHLEN];
  uint8_t re[NOISE_DHLEN];
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

  memcpy(re, frame + 1, NOISE_DHLEN);
  mix_hash(&ini->ss, re, NOISE_DHLEN);

  if (noise_dh(ini->e_priv, re, dh) != 0) {
    return NOISE_ERR_DH;
  }
  mix_key(&ini->ss, dh, NOISE_DHLEN);

  if (noise_dh(ini->s_priv, re, dh) != 0) {
    return NOISE_ERR_DH;
  }
  mix_key(&ini->ss, dh, NOISE_DHLEN);

  rc = decrypt_and_hash(&ini->ss, frame + 1 + NOISE_DHLEN, NOISE_TAGLEN, empty);
  if (rc != NOISE_OK) {
    return rc;
  }

  memset(out, 0, sizeof(*out));
  hkdf2(ini->ss.ck, NULL, 0, out->send_key, out->recv_key);

  noise_wipe(dh, sizeof(dh));
  noise_wipe(&ini->ss, sizeof(ini->ss));
  noise_wipe(ini->e_priv, sizeof(ini->e_priv));
  ini->stage = 2;
  return NOISE_OK;
}

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
  noise_aead_encrypt(s->send_key, s->tx, NULL, 0, pt, pt_len,
                     out + NOISE_TRANSPORT_HDR_LEN);
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

  if (noise_aead_decrypt(s->recv_key, counter, NULL, 0,
                         frame + NOISE_TRANSPORT_HDR_LEN, ct_len, out) != 0) {
    return NOISE_ERR_DECRYPT;
  }
  rc = replay_accept(s, counter);
  if (rc != NOISE_OK) {
    return rc;
  }
  *out_len = ct_len - NOISE_TAGLEN;
  return NOISE_OK;
}

void noise_session_wipe(struct noise_session *s) { noise_wipe(s, sizeof(*s)); }
