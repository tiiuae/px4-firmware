#!/usr/bin/env bash
set -euo pipefail

here=$(dirname "$(realpath "$0")")
SM=$here/../noise/sm-noise-c
secs=${1:-600}
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

clang -g -O1 -std=gnu99 -fsanitize=fuzzer,address,undefined \
	-fno-sanitize-recover=undefined -I "$here/.." \
	-o "$out/fuzz_link" \
	"$here/fuzz_link.c" "$here/../secure_link.c" "$here/../handshake_local.c" \
	"$here/../cobs.c" "$SM/noise_ik.c" "$SM/chacha20_ietf.c" \
	"$SM/backend_sodium.c" \
	-lsodium

mkdir -p "$out/corpus"
"$out/fuzz_link" -max_total_time="$secs" -max_len=1500 "$out/corpus"
