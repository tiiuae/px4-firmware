/* Crypto primitives the Noise core needs, and nothing else.
 *
 * Two backends exist: NuttX crypto/ for the target, libsodium for host
 * tests. Keeping the core free of either is what lets the same state machine
 * be tested on a laptop and flown on an aircraft.
 */

#ifndef ZTCS_NOISE_BACKEND_H
#define ZTCS_NOISE_BACKEND_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NOISE_HASHLEN 32
#define NOISE_KEYLEN 32
#define NOISE_DHLEN 32
#define NOISE_TAGLEN 16

void noise_sha256(const uint8_t *in, size_t len, uint8_t out[NOISE_HASHLEN]);

void noise_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg,
                       size_t msg_len, uint8_t out[NOISE_HASHLEN]);

/* The static private key never appears here. A backend holds it wherever it
 * likes, including somewhere this code cannot read, and answers with the
 * shared secret and the public half.
 */

#ifdef NOISE_STATIC_KEY_BY_INDEX
struct noise_static_key {
  uint8_t index;
};
#else
struct noise_static_key {
  uint8_t sk[NOISE_DHLEN];
};
#endif

int noise_dh_static(const struct noise_static_key *s,
                    const uint8_t pk[NOISE_DHLEN],
                    uint8_t out[NOISE_DHLEN]);

int noise_static_public(const struct noise_static_key *s,
                        uint8_t pk[NOISE_DHLEN]);

/* X25519. Returns 0 on success, non-zero if the result is the all-zero point. */
int noise_dh(const uint8_t sk[NOISE_DHLEN], const uint8_t pk[NOISE_DHLEN],
             uint8_t out[NOISE_DHLEN]);

void noise_dh_public(const uint8_t sk[NOISE_DHLEN], uint8_t pk[NOISE_DHLEN]);

/* RFC 8439 AEAD. `nonce` is Noise's 64-bit counter; the 96-bit IETF nonce is
 * built inside the backend so the core never encodes it twice.
 */
void noise_aead_encrypt(const uint8_t key[NOISE_KEYLEN], uint64_t nonce,
                        const uint8_t *ad, size_t ad_len, const uint8_t *pt,
                        size_t pt_len, uint8_t *out);

int noise_aead_decrypt(const uint8_t key[NOISE_KEYLEN], uint64_t nonce,
                       const uint8_t *ad, size_t ad_len, const uint8_t *ct,
                       size_t ct_len, uint8_t *out);

/* Must come from a seeded pool. On NuttX that means /dev/random, which is why
 * the entropy driver is a precondition and not a detail.
 */
int noise_random(uint8_t *out, size_t len);

void noise_wipe(void *p, size_t len);

#ifdef __cplusplus
}
#endif

#endif
