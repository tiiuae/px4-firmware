/* Noise_IK_25519_ChaChaPoly_SHA256 initiator, aircraft side.
 *
 * Wire format and rationale: docs/rfc-secure-mavlink.md. The Rust responder
 * is crates/ztcs-noise-udp; the two are tested against each other.
 *
 * Sans-io: no sockets, no timers. Caller owns both.
 */

#ifndef ZTCS_NOISE_IK_H
#define ZTCS_NOISE_IK_H

#include <stddef.h>
#include <stdint.h>

#include "noise_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NOISE_TYPE_HANDSHAKE_INIT 1
#define NOISE_TYPE_HANDSHAKE_RESP 2
#define NOISE_TYPE_TRANSPORT 3

#define NOISE_TRANSPORT_HDR_LEN 9
#define NOISE_IDENTITY_PAYLOAD_LEN 97
#define NOISE_PAYLOAD_VERSION 1

#define NOISE_MSG1_LEN (1 + 32 + (32 + NOISE_TAGLEN) + (NOISE_IDENTITY_PAYLOAD_LEN + NOISE_TAGLEN))
#define NOISE_MSG2_LEN (1 + 32 + NOISE_TAGLEN)

#define NOISE_STATIC_KEY_CONTEXT "ztcs-mavlink-static-key:"

enum noise_result {
  NOISE_OK = 0,
  NOISE_ERR_INPUT = -1,
  NOISE_ERR_STATE = -2,
  NOISE_ERR_DECRYPT = -3,
  NOISE_ERR_DH = -4,
  NOISE_ERR_RANDOM = -5,
  NOISE_ERR_REPLAY = -6,
  NOISE_ERR_EXHAUSTED = -7,
};

struct noise_symmetric {
  uint8_t ck[NOISE_HASHLEN];
  uint8_t h[NOISE_HASHLEN];
  uint8_t k[NOISE_KEYLEN];
  uint64_t n;
  int has_key;
};

struct noise_session {
  uint8_t send_key[NOISE_KEYLEN];
  uint8_t recv_key[NOISE_KEYLEN];
  uint64_t tx;
  uint64_t rx_highest;
  uint64_t rx_bitmap;
  int rx_started;
};

struct noise_initiator {
  struct noise_symmetric ss;
  uint8_t e_priv[NOISE_DHLEN];
  uint8_t e_pub[NOISE_DHLEN];
  const struct noise_static_key *s;
  uint8_t s_pub[NOISE_DHLEN];
  int stage;
};

/* The bytes an identity key signs to vouch for a Noise static key. Writes
 * sizeof(NOISE_STATIC_KEY_CONTEXT) - 1 + 32 bytes.
 */
size_t noise_static_key_signing_input(const uint8_t x25519_public[32],
                                      uint8_t *out);

/* Builds message 1. `identity` is the 97-byte payload the caller has already
 * signed. `out` must hold NOISE_MSG1_LEN.
 */
int noise_initiator_start(struct noise_initiator *ini,
                          const struct noise_static_key *s,
                          const uint8_t rs_pub[NOISE_DHLEN],
                          const uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN],
                          uint8_t *out, size_t *out_len);

int noise_initiator_finish(struct noise_initiator *ini, const uint8_t *frame,
                           size_t frame_len, struct noise_session *out);

/* `out` must hold NOISE_TRANSPORT_HDR_LEN + pt_len + NOISE_TAGLEN. */
int noise_session_seal(struct noise_session *s, const uint8_t *pt, size_t pt_len,
                       uint8_t *out, size_t *out_len);

/* `out` must hold frame_len; the plaintext is shorter. Replay is checked only
 * after the tag verifies.
 */
int noise_session_open(struct noise_session *s, const uint8_t *frame,
                       size_t frame_len, uint8_t *out, size_t *out_len);

void noise_session_wipe(struct noise_session *s);

#ifdef __cplusplus
}
#endif

#endif
