#!/bin/bash

cd /home/tii/hitl/
./remove_server.sh
./start_server.sh &

cd "${0%/*}"

python3 hitl_mission.py
