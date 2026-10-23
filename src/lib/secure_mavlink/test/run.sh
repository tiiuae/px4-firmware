#!/usr/bin/env bash
# The transport is a state machine over a socket, so it is testable here.
# A host stack is not a 16k task stack: resource limits still need the board.
set -euo pipefail

here=$(dirname "$(realpath "$0")")
SM=$here/../noise/sm-noise-c
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

gcc -O2 -Wall -Wextra -std=gnu99 -I "$here/.." \
    -o "$out/secure_link_test" \
    "$here/secure_link_test.c" "$here/../secure_link.c" "$here/../handshake_local.c" \
    "$SM/noise_ik.c" "$SM/chacha20_ietf.c" \
    "$SM/backend_sodium.c" \
    -lsodium

gcc -O2 -Wall -Wextra -std=gnu99 -I "$here/.." \
    -o "$out/cobs_test" "$here/cobs_test.c" "$here/../cobs.c"

"$out/cobs_test"
"$out/secure_link_test"
