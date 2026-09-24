# ztcs_secure_link

Aircraft end of the secure MAVLink link. Terminates
`Noise_IK_25519_ChaChaPoly_SHA256` against the ground station, so MAVLink
leaves the aircraft encrypted and authenticated.

MAVLink is never parsed here. It is an opaque payload, so the same transport
carries anything else the link needs to move.

## What is in here

| Path               | What it is                                                  |
| ------------------ | ----------------------------------------------------------- |
| `secure_link.[ch]` | the session state machine: handshake retransmit, rekey       |
| `noise/`           | the Noise IK core, vendored (see below)                      |
| `test/`            | host driver and an end-to-end script                         |

Sans-io, like the ground-station side: this owns no socket and no timer. The
caller passes the clock in and sends the datagrams handed back. That is what
makes the state machine testable on a host, where a flight controller's
timing cannot be reproduced.

## The vendored Noise core

`noise/` is copied from `crates/ztcs-noise-udp/c` in
https://github.com/tiiuae/ZTCS and **is not edited here**. That repository is
the source of truth, holds the wire contract in `docs/rfc-secure-mavlink.md`,
and carries the Rust implementation the C is tested against.

Copied from ZTCS commit `2965cdf`.

## Testing

`test/e2e.sh` mints an operator key, an aircraft key set and a real
`.attest`, starts the ground station, and drives this library's state machine
against it over a real socket. It exercises `secure_link.c` itself rather than
a reimplementation, so what ships is what was tested.

```
ZTCS_DIR=~/Code/ztcs ./test/e2e.sh
```

## Before this is worth anything

The ephemeral key comes from `/dev/random`. On a board where the entropy pool
is never seeded, every session key is predictable and the protocol above it
does not matter. Confirm two boots produce different session keys before
trusting this.
