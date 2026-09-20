/****************************************************************************
 * The identity key: the thing that says which aircraft this is.
 *
 * It signs the link key and never anything else, and it is the one key that
 * must be unreadable, so the curve follows the part rather than our taste.
 * A SoC with an enclave holds Ed25519; a SoC that needs an external secure
 * element holds ECDSA P-256, because no shipping element firmware exposes
 * Curve25519. The ground station derives a libp2p PeerId from either, so
 * enrolment and attestation do not know which this is.
 ****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "noise/noise_ik.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One of the NOISE_PAYLOAD_VERSION_* values, and the 32 bytes that follow it
 * in the payload: an Ed25519 public key, or the X coordinate of a P-256 one
 * whose parity the version byte carries.
 */
struct secure_link_identity {
  uint8_t version;
  uint8_t public_key[32];
};

/* Generates the key on first use. False means the part has no place to keep
 * one, which is fatal: an aircraft with a readable identity key has none.
 */
bool secure_link_identity_public(struct secure_link_identity *id);

/* Signature encoding follows the curve: Ed25519 is RFC 8032 over SHA-512,
 * P-256 is ECDSA over SHA-256 as r||s, both 64 bytes.
 */
bool secure_link_identity_sign(const uint8_t *msg, size_t msg_len,
                               uint8_t sig[64]);

#ifdef __cplusplus
}
#endif
