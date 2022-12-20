#!/bin/bash

target="${1:-all}"

source /opt/ros/galactic/setup.sh

export SIGNING_TOOL=Tools/cryptotools.py

if [ "${target}" == v1 ]
then
  # Remove old build output
  rm -Rf build/ssrc_saluki-v1_default build/ssrc_saluki-v1_amp build/ssrc_saluki-v1_bootloader build/ssrc_saluki-v1_protected

  # Build
  make ssrc_saluki-v1_default
  make ssrc_saluki-v1_amp
  make ssrc_saluki-v1_bootloader
  make ssrc_saluki-v1_protected
elif [ "${target}" == v2 ]
then
  # Remove old build output
  rm -Rf build/ssrc_saluki-v2_default build/ssrc_saluki-v2_amp build/ssrc_saluki-v2_bootloader

  # Build
  make ssrc_saluki-v2_default
  make ssrc_saluki-v2_amp
  make ssrc_saluki-v2_bootloader
fi
