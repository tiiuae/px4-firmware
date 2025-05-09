#!/bin/bash

set -euo pipefail

while read -r repo
do
  [[ "${repo}" == boards/ssrc/* ]] || \
  [[ "${repo}" == *saluki-sec-scripts ]] || \
  [[ "${repo}" == *pfsoc_crypto ]]  || \
  [[ "${repo}" == *pfsoc_keystore ]]  || \
  [[ "${repo}" == *imx9_keystore ]]  || \
  [[ "${repo}" == *pf_crypto ]] || \
  [[ "${repo}" == *assembly_agent ]] || \
  [[ "${repo}" == *px4_fw_update_client ]] || \
  [[ "${repo}" == *secure_udp ]] || \
  [[ "${repo}" == *rust_px4_nuttx ]] || \
  [[ "${repo}" == *rust_module_example ]] || \
  [[ "${repo}" == *secure_udp_proxy ]] || \
  [[ "${repo}" == *saluki_packaging ]] || \
  [[ "${repo}" == *process ]] && continue
  git submodule update --init --recursive "${repo}"
done <<< "$(git submodule status | awk '{print $2}')"
