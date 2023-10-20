#!/bin/bash
#

export PATH=/px4_sitl/bin:$PATH

case $PX4_VEHICLE_TYPE in
  mc)
    export PX4_SYS_AUTOSTART=4401
    ;;
  rover)
    export PX4_SYS_AUTOSTART=50006
    ;;
  vtol)
    export PX4_SYS_AUTOSTART=4430
    ;;
  fw)
    export PX4_SYS_AUTOSTART=4440
    ;;
  uuv)
    export PX4_SYS_AUTOSTART=4403
    ;;
  *)
    echo "ERROR: unknown vehicle type: $PX4_VEHICLE_TYPE"
    exit 1
    ;;
esac

export PX4_GZ_MODEL_NAME=$DRONE_DEVICE_ID
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-default}
export GZ_PARTITION=sim
export GZ_RELAY=$(dig +short gazebo-server)
export GZ_IP=$(hostname -i)

source /opt/ros/humble/setup.sh

/px4/bin/px4 -d -s /px4/etc/init.d-posix/rcS -w /px4
