#!/bin/bash

set -euo pipefail

while read -r repo
do
  [[ "${repo}" == boards/ssrc/* ]] || \
  [[ "${repo}" == *saluki-sec-scripts ]] || \
  [[ "${repo}" == *pfsoc_crypto ]]  || \
  [[ "${repo}" == *pfsoc_keystore ]]  || \
  [[ "${repo}" == *pf_crypto ]] || \
  [[ "${repo}" == *secure_mavlink_udp_proxy ]] || \
  [[ "${repo}" == *process ]] && continue
  git submodule update --init --recursive "${repo}"
done <<< "$(git submodule status | awk '{print $2}')"
