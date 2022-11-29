#!/bin/bash

# Set exit on error
# set -e

# vars
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
target_px4_firmware="${1}"
smart_plug_ip="172.18.32.109"

# Non-blocking-wait function
# baeldung.com/linux/background-process-get-exit-code
non_blocking_wait() {
	PID=$1
	if [ ! -d "/proc/$PID" ]; then
		wait $PID
		CODE=$?
	else
		CODE=127
	fi
	return $CODE
}

reboot_fc() {
	kasa --type plug --host ${smart_plug_ip} off
	sleep 2
	kasa --type plug --host ${smart_plug_ip} on
	return 0
}

echo "${SCRIPT_DIR}"
cd "${SCRIPT_DIR}/../../Tools"

# Flash saluki with FW from the build
python3 px_uploader.py --port /dev/ttyUSB0 --baud-bootloader 2000000 ../${target_px4_firmware} &
#python3 px_uploader.py --port /dev/ttyUSB0 --baud-bootloader 2000000 /home/tii/Downloads/ssrc_saluki-v1_default-1.13.0-alpha1-5019-g38a6a4a9cd.px4 &
pid_upload=$!

# Reboot saluki while uploader is running -> interrupts application loading and starts upload
reboot_fc

# Wait while upload process is ongoing
while /bin/true; do
	non_blocking_wait $pid_upload
	retcode=$?
	if [ $retcode -ne 127 ]; then
		echo "PID $PID terminated with exit code $retcode"
		break
	fi
	sleep 2
done

cd /home/tii/hitl/
./remove_server.sh
./start_server.sh
# TODO: how to determine that gzserver is started and status is OK?
sleep 30

# Reboot FC and give it time to start up
reboot_fc
#sleep 30

cd "${SCRIPT_DIR}"
python3 hitl_mission.py
if [ $? -eq 0 ]
then
	echo "HITL test OK"
	exit 0
else
	echo "HITL test failed"
	exit 1
fi

