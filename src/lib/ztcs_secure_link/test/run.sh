#!/usr/bin/env bash
# The transport is a state machine over a socket, so it is testable here.
# A host stack is not a 16k task stack: resource limits still need the board.
set -euo pipefail

here=$(dirname "$(realpath "$0")")
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

g++ -std=c++17 -Wall -Wextra -I "$here/fake" -I "$here/.." \
    -o "$out/link_udp_test" \
    "$here/link_udp_test.cpp" "$here/stub_link.cpp" "$here/../ZtcsLinkUdp.cpp" \
    -lpthread

"$out/link_udp_test"
