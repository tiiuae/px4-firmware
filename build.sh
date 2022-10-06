#!/bin/bash -eux

usage() {
	echo " usage: $0 <output-dir> <build-target>"
	echo "   output-dir : directory for output artifacts"
	echo "   build-target : supported build targets:"
  echo "     px4fwupdater"
  echo "     px4_fmu-v5x_ssrc"
  echo "     ssrc_saluki-v1_default"
  echo "     ssrc_saluki-v1_protected"
  echo "     ssrc_saluki-v1_amp"
  echo "     ssrc_saluki-v1_bootloader"
  echo "     ssrc_saluki-v2_default"
  echo "     ssrc_saluki-v2_amp"
  echo "     ssrc_saluki-v2_bootloader"
  echo
  exit 1
}

dest_dir="${1:-}"
target="${2:-}"

if [ -z "$dest_dir" ] || [ -z "$target" ]; then
  usage
fi

version=$(git describe --always --tags --dirty | sed 's/^v//')
script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $1)
iname_env=tii_px4_build

mkdir -p ${dest_dir}
pushd ${script_dir}

build_env="docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) --pull -f ./packaging/Dockerfile.build_env -t ${iname_env} ."
build_cmd_fw="docker run --rm -v ${script_dir}:/px4-firmware/sources ${iname_env} ./packaging/build_px4fw.sh"
build_cmd_px4fwupdater="${script_dir}/packaging/build_px4fwupdater.sh -v ${version} -i ${dest_dir}"

# Generate build_env
if [ "${target}" != px4fwupdater ]; then
  $build_env
else
  $build_cmd_px4fwupdater
  exit 0
fi

ext=""
case $target in
  "px4_fmu-v5x_ssrc") ;&
  "ssrc_saluki-v1_default") ;&
  "ssrc_saluki-v1_protected") ;&
  "ssrc_saluki-v2_default")
    ext="px4"
    ;;
  "ssrc_saluki-v1_amp") ;&
  "ssrc_saluki-v2_amp")
    ext="px4"
    ;;
  "ssrc_saluki-v1_bootloader") ;&
  "ssrc_saluki-v2_bootloader")
    ext="elf"
    ;;
   *)
    usage
    ;;
esac


$build_cmd_fw ${target}
cp ${script_dir}/build/${target}/${target}.${ext} ${dest_dir}/${target}-${version}.${ext}

echo "Done"
