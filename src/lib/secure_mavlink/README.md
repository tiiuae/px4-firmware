# secure_mavlink

The aircraft end of secure MAVLink: `Noise_IK_25519_ChaChaPoly_SHA256` over UDP
or a serial radio to the secure-mavlink ground station
([tiiuae/secure-mavlink](https://github.com/tiiuae/secure-mavlink)). Wire contract and
threat model: [RFC, secure link](https://github.com/tiiuae/secure-mavlink/blob/main/docs/rfc-secure-link.md).

| Path                                              | What                                                                              |
| ------------------------------------------------- | --------------------------------------------------------------------------------- |
| `secure_link.[ch]`                                | session state machine: handshake retransmit, rekey, replay, silence               |
| `cobs.[ch]`                                       | COBS framing for a serial radio                                                   |
| `noise/sm-noise-c/`                               | Noise IK core, pinned [sm-noise-c](https://github.com/tiiuae/sm-noise-c) submodule |
| `noise/backend_px4.cpp`, `noise/backend_kernel.c` | PX4 keystore and kernel adapters beside the submodule                             |
| `handshake_kernel.cpp`, `handshake_local.c`       | the handshake in the kernel (protected builds), or in this process (flat)         |
| `secure_link_keys.cpp`, `identity_px4crypto.cpp`  | keys from keystore slots, identity from the enclave                               |
| `test/`                                           | host tests and an end-to-end run against the real gateway                         |

`secure_link.c` is sans-io: no socket, no timer. The caller passes the clock
and sends what it is handed, which is what lets a host test drive it.

## Keys

| Key                      | Where                                | Used                      |
| ------------------------ | ------------------------------------ | ------------------------- |
| link, X25519             | keystore slot 15, the kernel does DH | every handshake           |
| identity, i.MX9          | P-256 in the ELE, never exported     | signs the link key, once  |
| identity, elsewhere      | Ed25519, keystore slot 17            | signs the link key, once  |
| signed identity (public) | `/fs/certs/ztcs-identity`            | read by every link at arm |

`secure_mavlink_enroll sign` signs the link key and stores the payload with the
link key it covers; afterwards it prints the stored one. A link arms only when
the file matches the current link and identity keys, and never signs.

The slot numbers and the identity path keep their original values so aircraft
enrolled before the rename keep working.

## The shared core

`noise/sm-noise-c/` is the pinned [sm-noise-c](https://github.com/tiiuae/sm-noise-c)
submodule, shared with the secure-mavlink ground station. `backend_px4.cpp` and
`backend_kernel.c` stay here: they are the PX4 keystore and kernel adapters.
Change the core in sm-noise-c, then deliberately commit the new pin. There is
no automatic update or requirement to match the ground station's pin.

Only the classical suite is built here. sm-noise-c also carries the hybrid
(ML-KEM-768) half; it stays out of the build unless `NOISE_HFS` is defined.

## Test

```sh
./test/run.sh                                        # state machine and COBS
./test/fuzz.sh 60                                    # libFuzzer over the frame parser
SECURE_MAVLINK_DIR=~/Code/secure-mavlink ./test/e2e.sh  # against the real gateway
```

`e2e.sh` mints an operator key, an aircraft key set and a real `.attest`, then
drives `secure_link.c` itself against the gateway over a socket and over a
serial radio. It defaults to a `secure-mavlink` checkout beside this repository.

## Do not

| Do not                                   | Because                                                                                                                   |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| edit `noise/sm-noise-c/` here            | it is a submodule; change sm-noise-c and bump the pin                                                                     |
| sign the identity at boot                | the i.MX93 ELE aborts, and resets the part, on repeated signatures: [ZTCS#133](https://github.com/tiiuae/ZTCS/issues/133) |
| trust it on a board without real entropy | the ephemeral key comes from `/dev/random`; an unseeded pool makes every session key guessable                            |
