#!/bin/bash

set -euo pipefail

while read -r repo
do
  [[ "${repo}" == boards/ssrc/* ]] || \
  [[ "${repo}" == *saluki-sec-scripts ]] || \
  [[ "${repo}" == *ssrc_crypto ]]  || \
  [[ "${repo}" == *pfsoc_crypto ]]  || \
  [[ "${repo}" == *pfsoc_keystore ]]  || \
  [[ "${repo}" == *imx9_keystore ]]  || \
  [[ "${repo}" == *pf_crypto ]] || \
  [[ "${repo}" == *px4_fw_update_client ]] || \
  [[ "${repo}" == *saluki_packaging ]] || \
  [[ "${repo}" == *rust_px4_nuttx ]] || \
  [[ "${repo}" == *assembly_agent ]] || \
  [[ "${repo}" == *moi_agent ]] || \
  [[ "${repo}" == src/modules/redundancy ]] || \
  [[ "${repo}" == *process ]] || \
  [[ "${repo}" == src/modules/enroll_agent ]] || \
  [[ "${repo}" == *calibration_bridge ]] && continue
  git submodule update --init --recursive "${repo}"
done <<< "$(git submodule status | awk '{print $2}')"
