#!/bin/bash

#This script is designed to run HITL simulation and MAVSDK testing through a docker image.

#usage example
#./Tools/run_docker_hitl_gzsim_mavsdk_server.sh <docker_image> <script_and_arguments>

#examples
#To run hitl simulation
#./Tools/run_docker_hitl_gzsim_mavsdk_server.sh local_hitl_gzsim_server ./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_hitl.sdf
#To run mavsdk testing
#./Tools/run_docker_hitl_gzsim_mavsdk_server.sh local_hitl_gzsim_server ./test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/hitl_gz_harm.json --gui --case "Takeoff and Land"

#command for local building the docker image
#image for arm64(raspberry pi5)
#docker buildx build --platform linux/arm64 --build-arg platform=docker.io/ros:humble-ros-base            -t local_hitl_gzsim_server_arm64  -f px4-firmware/packaging/Dockerfile.hitl_gzsim_mavsdk .
#image for x86
#docker buildx build                        --build-arg platform=ghcr.io/tiiuae/gz-sim-server:sha-954fefc -t local_hitl_gzsim_server_x86    -f px4-firmware/packaging/Dockerfile.hitl_gzsim_mavsdk .


YELLOW='\033[0;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

if [ "$#" -eq 0 ]; then
    echo -e "You should specify ${RED}a docker image${NC} and ${YELLOW}a script for running.${NC}"
    echo -e "${GREEN}Usage example:${NC}"
    echo -e "  ./Tools/run_docker_hitl_gzsim_mavsdk_server.sh ${RED}<docker_image>${NC} ${YELLOW}<script_and_arguments>${NC}"
    echo
    echo -e "${GREEN}Examples ${NC}"
    echo -e "  1.To run hitl simulation"
    echo -e "    ./Tools/run_docker_hitl_gzsim_mavsdk_server.sh ${RED}local_hitl_gzsim_server_x86 ${YELLOW}./Tools/simulation/gz/hitl_run.sh ssrc_holybro_x500/model_hitl.sdf${NC}"
    echo
    echo -e "  2.To run mavsdk testing"
    echo -e "    ./Tools/run_docker_hitl_gzsim_mavsdk_server.sh ${RED}local_hitl_gzsim_server_x86 ${YELLOW}./test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/hitl_gz_harm.json --gui --case \"Takeoff and Land\"${NC}"
    echo
    echo -e "${GREEN}command for local building the docker image${NC}"
    echo -e "docker buildx build --build-arg platform=ghcr.io/tiiuae/gz-sim-server:sha-954fefc             -t ${RED}local_hitl_gzsim_server_x86${NC}    -f px4-firmware/packaging/Dockerfile.hitl_gzsim_mavsdk ."
    echo -e "docker buildx build --platform linux/arm64 --build-arg platform=docker.io/ros:humble-ros-base -t ${RED}local_hitl_gzsim_server_arm64${NC}  -f px4-firmware/packaging/Dockerfile.hitl_gzsim_mavsdk ."
    exit 1
fi

BASE_COMMAND=(
    docker run --rm
    --network host
    --env DISPLAY=${DISPLAY}
    --env PYTHONUNBUFFERED=1
    --env MESA_GL_VERSION_OVERRIDE=4.6
    --volume /tmp/.docker.xauth:/tmp/.docker.xauth
    --volume /tmp/.X11-unix:/tmp/.X11-unix
    --privileged
)

xhost si:localuser:root
${BASE_COMMAND[@]} "$@"
