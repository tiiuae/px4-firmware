# ztcs_secure_link

The aircraft end of the secure link: `Noise_IK_25519_ChaChaPoly_SHA256` over UDP
to the ZTCS gateway. The payload is opaque, so MAVLink and the OTA client ride
the same link. Wire contract and threat model:
[RFC, secure MAVLink](https://github.com/tiiuae/ZTCS/blob/main/docs/rfc-secure-mavlink.md).
Built as part of the image: [build](../../../README.md#build).

| Path                                             | What                                                                        |
| ------------------------------------------------ | --------------------------------------------------------------------------- |
| `secure_link.[ch]`                               | session state machine: handshake retransmit, rekey, replay, silence         |
| `noise/`                                         | Noise IK core, vendored, not edited here                                    |
| `ZtcsLinkUdp.[ch]pp`                             | `secure_udp::Udp` over the link, for request and response users (OTA, TFTP) |
| `secure_link_keys.cpp`, `identity_px4crypto.cpp` | keys from keystore slots, identity from the enclave                         |
| `test/`                                          | host tests and an end-to-end run against the real gateway                   |

`secure_link.c` is sans-io: no socket, no timer. The caller passes the clock
and sends what it is handed, which is what lets a host test drive it.

## The vendored core

`noise/` is a copy of `crates/ztcs-noise-udp/c` in
[ZTCS](https://github.com/tiiuae/ZTCS/tree/main/crates/ztcs-noise-udp/c),
file for file identical to `e26966a`, plus `backend_px4.cpp` of its own. Change
it there, then copy.

## Rules for `ZtcsLinkUdp`

| Rule                                                            | What breaks without it                                                                                |
| --------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| only `send` drives the state machine                            | `recv` re-driving handshakes churns sessions                                                          |
| a send after silence starts a new session                       | the station releases an idle session after 60 s and a client polling once a minute never speaks again |
| `secure_link_open` checks room for the plaintext, not the frame | a reply within 25 bytes of the caller buffer is dropped, counted nowhere                              |
| socket timeouts go through `set_timeout_ms`                     | the base `Udp::set_socket_timeout` takes seconds                                                      |

## Test

```sh
./test/run.sh                          # transport, against a stub link
ZTCS_DIR=~/Code/ztcs ./test/e2e.sh     # state machine against the real gateway
```

`e2e.sh` mints an operator key, an aircraft key set and a real `.attest`, then
drives `secure_link.c` itself against the gateway over a socket.

## Do not

| Do not                                   | Because                                                                                        |
| ---------------------------------------- | ---------------------------------------------------------------------------------------------- |
| edit `noise/` here                       | ZTCS holds the Rust side the C is tested against                                               |
| trust it on a board without real entropy | the ephemeral key comes from `/dev/random`; an unseeded pool makes every session key guessable |
