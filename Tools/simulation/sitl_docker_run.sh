#!/bin/bash

# docker run -it --privileged --rm \                                                        7m 13s 11:44:55
#     -v /path/to/px4-firmware:/home/user/Firmware:rw \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
#     -e DISPLAY=${DISPLAY} \
#     -e LOCAL_USER_ID="$(id -u)" \
#     -w /home/user/Firmware
#     --network=host  \
#     --name=container_name px4io/px4-dev-simulation-jammy \
#     "./Tools/simulation/sitl_docker_run.sh"

cd /home/user/Firmware

make px4_sitl gz_x500
