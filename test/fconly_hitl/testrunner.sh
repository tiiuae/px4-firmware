#!/bin/bash

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

cd /home/tii/hitl/
./remove_server.sh
sleep 10
./start_server.sh &
# TODO: how to determine that gzserver is started and status is OK?
sleep 30

echo "${SCRIPT_DIR}"
cd "${SCRIPT_DIR}/../../Tools"

# Flash saluki with FW from the build
#uploader_command="python3 px_uploader.py --port /dev/ttyUSB0 --baud-bootloader 2000000 ../${target_px4_firmware}"
#echo ${uploader_command}
python3 px_uploader.py --port /dev/ttyUSB0 --baud-bootloader 2000000 ../${target_px4_firmware} &
pid_upload=$!

#sh -c "$uploader_command"' echo pid=$! > pidfile; wait $!; echo $? > exit_status' &

# Reboot saluki
kasa --host ${smart_plug_ip} off
sleep 2
kasa --host ${smart_plug_ip} on

while /bin/true; do
	echo "while..."
	non_blocking_wait $pid_upload
	retcode=$?
	if [ $retcode -ne 127 ]; then
		echo "PID $PID terminated with exit code $retcode"
		break
	fi
	sleep 2
done

#sleep 180

upload_retcode=$(<exit_status)
echo ${upload_retcode}

cd /home/tii/hitl/

python3 hitl_mission.py

if [ $? -eq 0 ] 
then 
  echo "HITL test OK" 
  return 0
else 
  echo "HITL test failed" >&2 
  return 1
fi
