#!/bin/bash -eux

usage() {
  set +x
  echo ""
	echo " usage: $0 <output-dir> <build-target>"
	echo "   output-dir : directory for output artifacts"
	echo "   build-target : supported build targets:"
  echo "     saluki-v2_default"
  echo "     saluki-v2_bootloader"
  echo "     saluki-pi_default"
  echo "     saluki-pi_bootloader"
  echo
  exit 1
}

if [ -n ${SIGNING_ARGS} ]; then
  echo "using custom signing keys: ${SIGNING_ARGS}"
fi

dest_dir="${1:-}"
target="${2:-}"

if [ -z "$dest_dir" ]; then
  usage
fi

version=$(git describe --always --tags --dirty | sed 's/^v//')
script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)
iname_env=tii_px4_build

mkdir -p ${dest_dir}
pushd ${script_dir}

build_env="docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) --pull -f ./packaging/Dockerfile.build_env -t ${iname_env} ."
build_cmd_fw="docker run --rm -e SIGNING_ARGS=${SIGNING_ARGS} -v ${script_dir}:/px4-firmware/sources ${iname_env} ./packaging/build_px4fw.sh"

# Generate build_env
if [ "${target}" != px4fwupdater ]; then
  $build_env
fi

case $target in
  "saluki-v2_default")
    $build_cmd_fw ssrc_saluki-v2_default
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default.px4 ${dest_dir}/ssrc_saluki-v2_default-${version}.px4
    ;;
  "saluki-v2_custom_keys")
    # on custom keys case we build _default target but SIGNING_ARGS env variable is set above in build_cmd_fw
    $build_cmd_fw ssrc_saluki-v2_default
    cp ${script_dir}/build/ssrc_saluki-v2_default/ssrc_saluki-v2_default.px4 ${dest_dir}/ssrc_saluki-v2_custom_keys-${version}.px4
    ;;
  "saluki-v2_bootloader")
    $build_cmd_fw ssrc_saluki-v2_bootloader
    cp ${script_dir}/build/ssrc_saluki-v2_bootloader/ssrc_saluki-v2_bootloader.elf ${dest_dir}/ssrc_saluki-v2_bootloader-${version}.elf
    ;;
  "saluki-pi_default")
    $build_cmd_fw ssrc_saluki-pi_default
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default.px4 ${dest_dir}/ssrc_saluki-pi_default-${version}.px4
    ;;
  "saluki-pi_custom_keys")
    # on custom keys case we build _default target but SIGNING_ARGS env variable is set above in build_cmd_fw
    $build_cmd_fw ssrc_saluki-pi_default
    cp ${script_dir}/build/ssrc_saluki-pi_default/ssrc_saluki-pi_default.px4 ${dest_dir}/ssrc_saluki-pi_custom_keys-${version}.px4
    ;;
  "saluki-pi_bootloader")
    $build_cmd_fw ssrc_saluki-pi_bootloader
    cp ${script_dir}/build/ssrc_saluki-pi_bootloader/ssrc_saluki-pi_bootloader.elf ${dest_dir}/ssrc_saluki-pi_bootloader-${version}.elf
    ;;
   *)
    usage
    ;;
esac

echo "Done"

