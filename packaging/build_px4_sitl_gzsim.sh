#!/bin/bash

set -eo pipefail

source /opt/ros/humble/setup.sh

pushd px4-firmware
	make px4_sitl_default
popd
