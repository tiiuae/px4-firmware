/* NuttX backend. No flight-stack dependency, so this is the same code on
 * arm64 for i.MX93 and on arm for the RT parts.
 *
 * NOT YET COMPILED FOR TARGET. Verified on the host: the Noise core against
 * the Rust responder (c/interop.sh), the ChaCha20 below against the RFC 8439
 * vector and a reference stream, and the AEAD framing against a reference
 * AEAD at every padding boundary. Unverified: that these NuttX symbols and
 * struct names are spelled correctly, which one compile settles.
 */

#include "noise_backend.h"

#include "chacha20_ietf.h"

#include <crypto/curve25519.h>
#include <crypto/hmac.h>
#include <crypto/poly1305.h>
#include <crypto/sha2.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

void noise_sha256(const uint8_t *in, size_t len, uint8_t out[NOISE_HASHLEN]) {
  SHA2_CTX ctx;
  sha256init(&ctx);
  sha256update(&ctx, in, len);
  sha256final(out, &ctx);
}

void noise_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg,
                       size_t msg_len, uint8_t out[NOISE_HASHLEN]) {
  HMAC_SHA256_CTX ctx;
  hmac_sha256_init(&ctx, key, (u_int)key_len);
  hmac_sha256_update(&ctx, msg, (u_int)msg_len);
  hmac_sha256_final(out, &ctx);
}

int noise_dh(const uint8_t sk[NOISE_DHLEN], const uint8_t pk[NOISE_DHLEN],
             uint8_t out[NOISE_DHLEN]) {
  /* NuttX answers "the result is not the all-zero point", so passing it
   * through reports every success as a failure, and every low-order peer key
   * as a success.
   */
  return curve25519(out, sk, pk) != 0 ? 0 : -1;
}

#ifndef NOISE_STATIC_KEY_BY_INDEX
int noise_dh_static(const struct noise_static_key *s,
                    const uint8_t pk[NOISE_DHLEN],
                    uint8_t out[NOISE_DHLEN]) {
  return noise_dh(s->sk, pk, out);
}

int noise_static_public(const struct noise_static_key *s,
                        uint8_t pk[NOISE_DHLEN]) {
  noise_dh_public(s->sk, pk);
  return 0;
}

#endif

void noise_dh_public(const uint8_t sk[NOISE_DHLEN], uint8_t pk[NOISE_DHLEN]) {
  curve25519_generate_public(pk, sk);
}

/* RFC 8439 AEAD. NuttX ships ChaCha20 in the DJB layout, whose 64-bit nonce
 * and separate salt word are not what Noise specifies, so the stream comes
 * from chacha20_ietf.c instead. Poly1305 is taken from the OS: its interface
 * is the standard one-time-key form and carries no layout ambiguity.
 *
 * Block 0 of the keystream is the Poly1305 key, the payload starts at block 1,
 * and the tag covers aad || pad16 || ct || pad16 || len64(aad) || len64(ct).
 */

static void put_le64(uint8_t *p, uint64_t v) {
  for (int i = 0; i < 8; i++) {
    p[i] = (uint8_t)(v >> (8 * i));
  }
}

static void poly_pad(poly1305_state *st, size_t len) {
  static const uint8_t zeros[16] = {0};
  size_t rem = len % 16;
  if (rem) {
    poly1305_update(st, zeros, 16 - rem);
  }
}

static void aead_tag(const uint8_t key[32], const uint8_t nonce[12],
                     const uint8_t *ad, size_t ad_len, const uint8_t *ct,
                     size_t ct_len, uint8_t tag[16]) {
  uint8_t poly_key[64];
  poly1305_state st;
  uint8_t lens[16];

  memset(poly_key, 0, sizeof(poly_key));
  chacha20_ietf_block(key, nonce, 0, poly_key);
  poly1305_begin(&st, poly_key);

  if (ad_len) {
    poly1305_update(&st, ad, ad_len);
    poly_pad(&st, ad_len);
  }
  if (ct_len) {
    poly1305_update(&st, ct, ct_len);
    poly_pad(&st, ct_len);
  }
  put_le64(lens, ad_len);
  put_le64(lens + 8, ct_len);
  poly1305_update(&st, lens, 16);
  poly1305_finish(&st, tag);

  noise_wipe(poly_key, sizeof(poly_key));
  noise_wipe(&st, sizeof(st));
}

static void ietf_nonce(uint64_t n, uint8_t out[12]) {
  memset(out, 0, 4);
  for (int i = 0; i < 8; i++) {
    out[4 + i] = (uint8_t)(n >> (8 * i));
  }
}

void noise_aead_encrypt(const uint8_t key[NOISE_KEYLEN], uint64_t nonce,
                        const uint8_t *ad, size_t ad_len, const uint8_t *pt,
                        size_t pt_len, uint8_t *out) {
  uint8_t iv[12];
  ietf_nonce(nonce, iv);
  if (pt_len) {
    memmove(out, pt, pt_len);
    chacha20_ietf_xor(key, iv, 1, out, pt_len);
  }
  aead_tag(key, iv, ad, ad_len, out, pt_len, out + pt_len);
}

int noise_aead_decrypt(const uint8_t key[NOISE_KEYLEN], uint64_t nonce,
                       const uint8_t *ad, size_t ad_len, const uint8_t *ct,
                       size_t ct_len, uint8_t *out) {
  uint8_t iv[12];
  uint8_t tag[16];
  size_t pt_len;
  uint8_t diff = 0;

  if (ct_len < NOISE_TAGLEN) {
    return -1;
  }
  pt_len = ct_len - NOISE_TAGLEN;
  ietf_nonce(nonce, iv);
  aead_tag(key, iv, ad, ad_len, ct, pt_len, tag);

  /* Constant time: never branch on where the tag differs. */
  for (int i = 0; i < NOISE_TAGLEN; i++) {
    diff |= tag[i] ^ ct[pt_len + i];
  }
  if (diff) {
    return -1;
  }
  if (pt_len) {
    memmove(out, ct, pt_len);
    chacha20_ietf_xor(key, iv, 1, out, pt_len);
  }
  return 0;
}

#ifndef NOISE_RANDOM_EXTERNAL
/* /dev/random, not /dev/urandom: on this platform the pool is seeded from the
 * enclave TRNG at boot and a caller that reads before that must block rather
 * than receive zeros.
 */
int noise_random(uint8_t *out, size_t len) {
  int fd = open("/dev/random", O_RDONLY);
  ssize_t got;
  if (fd < 0) {
    return -1;
  }
  got = read(fd, out, len);
  close(fd);
  return (got == (ssize_t)len) ? 0 : -1;
}
#endif

void noise_wipe(void *p, size_t len) {
  volatile uint8_t *v = p;
  while (len--) {
    *v++ = 0;
  }
}
