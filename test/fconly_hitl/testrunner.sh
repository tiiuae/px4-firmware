#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


smart_plug_ip="172.18.32.109"

cd /home/tii/hitl/
./remove_server.sh
sleep 10
./start_server.sh &
# TODO: how to determine that gzserver is started and status is OK?
sleep 30

echo "${SCRIPT_DIR}"
cd "${SCRIPT_DIR}/../../Tools"
pwd

# Flash saluki with FW from the build
python3 px_uploader.py --port /dev/ttyUSB0 --baud-bootloader 2000000 /home/tii/Downloads/ssrc_saluki-v1_default-1.13.0-alpha1-5019-g38a6a4a9cd.px4 &
		
# Reboot saluki
kasa --host ${smart_plug_ip} off
sleep 2
kasa --host ${smart_plug_ip} on

sleep 180

cd /home/tii/hitl/

#python3 hitl_mission.py


