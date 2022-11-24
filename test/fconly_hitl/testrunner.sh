#!/bin/bash

original_dir = "${0%/*}"

cd /home/tii/hitl/
./remove_server.sh
sleep 10
./start_server.sh &

#cd "${original_dir}"

python3 hitl_mission.py
