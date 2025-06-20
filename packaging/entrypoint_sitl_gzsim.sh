#!/bin/bash
#

export PATH=/px4_sitl/bin:$PATH

case $PX4_VEHICLE_TYPE in
  mc)
    export PX4_SYS_AUTOSTART=4401
    export PX4_SIM_MODEL=holybro_x500
    ;;
  rover)
    export PX4_SYS_AUTOSTART=50005
    export PX4_SIM_MODEL=scout_mini
    ;;
  vtol)
    export PX4_SYS_AUTOSTART=4430
    export PX4_SIM_MODEL=striver_mini
    ;;
  fw)
    export PX4_SYS_AUTOSTART=4440
    export PX4_SIM_MODEL=skywalker_x8
    ;;
  custom)
    # user is responsible for setting PX4_SYS_AUTOSTART and PX4_SIM_MODEL
    ;;
  *)
    echo "ERROR: unknown vehicle type: $PX4_VEHICLE_TYPE"
    exit 1
    ;;
esac

export PX4_GZ_MODEL_NAME=$DRONE_DEVICE_ID
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-default}
export PX4_GZ_STANDALONE=1
export GZ_PARTITION=sim
export GZ_IP=${GZ_IP:-127.0.0.1}

source /opt/ros/humble/setup.sh

/px4_sitl/bin/px4 -d -s /px4_sitl/etc/init.d-posix/rcS -w /px4_sitl
