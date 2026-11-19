#!/bin/bash
# Incremental native/container build; invoke from any directory.
set -euo pipefail
WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$WORKSPACE"
export GZ_DISTRO=harmonic
make px4_sitl_default gzsim-plugins px4_gz_plugins mavsdk_tests "$@"
