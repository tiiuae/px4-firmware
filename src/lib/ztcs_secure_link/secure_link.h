/****************************************************************************
 * Secure MAVLink link, aircraft side. Terminates
 * Noise_IK_25519_ChaChaPoly_SHA256 against ztcs-mavlink-gateway; the wire
 * contract is docs/rfc-secure-mavlink.md in tiiuae/ZTCS.
 *
 * Sans-io: the caller owns the socket and the clock, which is what makes
 * the state machine testable off the aircraft.
 ****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "noise/noise_ik.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A MAVLink v2 frame plus 25 bytes of transport overhead, rounded up. */
#define SECURE_LINK_MTU 512

/* Over raw UDP a lost message 1 is a dead session, so retransmit is ours. */
#ifndef SECURE_LINK_RETRY_MIN_US
#define SECURE_LINK_RETRY_MIN_US     100000ULL   /* 100 ms */
#endif
#ifndef SECURE_LINK_RETRY_CAP_US
#define SECURE_LINK_RETRY_CAP_US    2000000ULL   /* 2 s */
#endif
#ifndef SECURE_LINK_RETRY_SLOW_US
#define SECURE_LINK_RETRY_SLOW_US   5000000ULL   /* 5 s once backed off */
#endif

#ifndef SECURE_LINK_SILENCE_US
#define SECURE_LINK_SILENCE_US     10000000ULL   /* nothing opened in 10 s */
#endif
#ifndef SECURE_LINK_DECRYPT_FAILS
#define SECURE_LINK_DECRYPT_FAILS        16      /* peer restarted, likely */
#endif

enum secure_link_state {
  SECURE_LINK_DOWN = 0,   /* no keys */
  SECURE_LINK_HANDSHAKING,
  SECURE_LINK_ESTABLISHED,
};

/* The identity payload is a signature over the static public key. This side
 * carries the signature, never the identity key that made it.
 */
struct secure_link_keys {
  uint8_t static_private[NOISE_DHLEN];
  uint8_t station_public[NOISE_DHLEN];
  uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN];
};

struct secure_link {
  enum secure_link_state state;
  struct secure_link_keys keys;
  struct noise_initiator ini;
  struct noise_session session;

  uint64_t next_retry_us;
  uint64_t retry_interval_us;
  uint64_t last_open_us;   /* last frame that passed the AEAD */
  uint32_t decrypt_fails;

  /* For the log: a link that rekeys often is a symptom. */
  uint32_t handshakes;
};

/* Returns 0, or a negative NOISE_ERR_*. */
int secure_link_init(struct secure_link *sl,
                     const struct secure_link_keys *keys, uint64_t now_us);

/* Datagram due now, or 0. Retransmit and the silence trigger live here, so
 * it runs whether or not there is traffic.
 */
int secure_link_poll(struct secure_link *sl, uint64_t now_us,
                     uint8_t *out, size_t cap);

/* Negative means no session yet: drop, do not retry. */
int secure_link_seal(struct secure_link *sl, uint64_t now_us,
                     const uint8_t *pt, size_t pt_len,
                     uint8_t *out, size_t cap);

/* Plaintext length, 0 if the datagram was protocol and consumed,
 * negative if rejected.
 */
int secure_link_open(struct secure_link *sl, uint64_t now_us,
                     const uint8_t *frame, size_t len,
                     uint8_t *out, size_t cap);

/* Generates the static private key on first boot if absent. False also
 * means "has a key, not enrolled yet", which is a normal first boot.
 */
bool secure_link_ensure_keys(struct secure_link_keys *keys);

/* For enrolment to sign. The private half has no accessor. */
bool secure_link_public_key(uint8_t out[NOISE_DHLEN]);

static inline bool secure_link_is_up(const struct secure_link *sl)
{
  return sl->state == SECURE_LINK_ESTABLISHED;
}

#ifdef __cplusplus
}
#endif
