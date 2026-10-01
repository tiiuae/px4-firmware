#!/usr/bin/env bash
# The transport is a state machine over a socket, so it is testable here.
# A host stack is not a 16k task stack: resource limits still need the board.
set -euo pipefail

here=$(dirname "$(realpath "$0")")
ztcs=${ZTCS_DIR:-$HOME/Code/ztcs}
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

[ -d "$ztcs" ] || { echo "set ZTCS_DIR to a ZTCS checkout" >&2; exit 1; }

gcc -O2 -Wall -Wextra -std=gnu99 -I "$here/.." \
    -o "$out/secure_link_test" \
    "$here/secure_link_test.c" "$here/../secure_link.c" "$here/../handshake_local.c" \
    "$here/../noise/noise_ik.c" "$here/../noise/chacha20_ietf.c" \
    "$ztcs/crates/ztcs-noise-udp/c/backend_sodium.c" \
    -lsodium

gcc -O2 -Wall -Wextra -std=gnu99 -I "$here/.." \
    -o "$out/cobs_test" "$here/cobs_test.c" "$here/../cobs.c"

g++ -std=c++17 -Wall -Wextra -I "$here/fake" -I "$here/.." \
    -o "$out/link_udp_test" \
    "$here/link_udp_test.cpp" "$here/stub_link.cpp" "$here/../ZtcsLinkUdp.cpp" \
    -lpthread

"$out/cobs_test"
"$out/secure_link_test"
"$out/link_udp_test"
