#!/usr/bin/env bash
set -euo pipefail

here=$(dirname "$(realpath "$0")")
ztcs=${ZTCS_DIR:-$HOME/Code/ztcs}
secs=${1:-600}
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

[ -d "$ztcs" ] || {
	echo "set ZTCS_DIR to a ZTCS checkout" >&2
	exit 1
}

clang -g -O1 -std=gnu99 -fsanitize=fuzzer,address,undefined \
	-fno-sanitize-recover=undefined -I "$here/.." \
	-o "$out/fuzz_link" \
	"$here/fuzz_link.c" "$here/../secure_link.c" "$here/../handshake_local.c" \
	"$here/../cobs.c" "$here/../noise/noise_ik.c" "$here/../noise/chacha20_ietf.c" \
	"$ztcs/crates/ztcs-noise-udp/c/backend_sodium.c" \
	-lsodium

mkdir -p "$out/corpus"
"$out/fuzz_link" -max_total_time="$secs" -max_len=1500 "$out/corpus"
