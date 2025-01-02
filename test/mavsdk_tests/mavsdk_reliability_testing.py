#!/usr/bin/env python3

import argparse
import sys
import subprocess
import json

import device_manager as dm


def create_test_config(known_args) :
    if (known_args.mission_type == "random"):
        name_test = "Fly a long mission for checking reliability"
    elif (known_args.mission_type == "area"):
        name_test = "Fly a long mission for checking reliability in a round area"
    else:
        #TODO: to implement
        name_test = "Fly a long mission for checking reliability with fixed waypoints"

    data = {
        name_test: {
            "time": known_args.duration,
            "leg": known_args.leg
    	}
    }

    with open("config_test.json", "w") as file:
        json.dump(data, file, indent=4, ensure_ascii=False)

def run_test(known_args, remaining_args):
    device = dm.DeviceManager()
    device.set_device(known_args.control_usb_hub)

    reboot_number = 0
    for i in range(known_args.iterations):
        print(f"Runing {i+1}/{known_args.iterations}...")
        try:
            command = "./test/mavsdk_tests/mavsdk_test_runner.py"
            prosces = subprocess.Popen([command] + remaining_args,  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            prosces.stdin.flush()
            output, errors = prosces.communicate()
            prosces.stdin.close()
            prosces.stdout.close()
            prosces.stderr.close()

            if (prosces.returncode != 0):
                print("output ", output)
                if (prosces.returncode == 2):
                    reboot_number += 1
                    print("Hard reboot!")
                    device.reboot_device(known_args.port)

        except subprocess.CalledProcessError as e:
            print(f"error: {e}")
            sys.exit(1)


    print (f"Device was hard rebooted {reboot_number} times")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=1,
                        help="how often to run all tests")
    parser.add_argument("--duration", type=int, default=60,
                        help="duration of a random misson (minutes)")
    parser.add_argument("--leg", type=int, default=100,
                        help="lag between waypoing (meters)")
    parser.add_argument("--mission_type", type=str, default="random",
                        help="mission types: random, area, replay")
    parser.add_argument("--control_usb_hub", type=str, default="",
                        help="the usb hub which will make hard reboot FC: Acroname, Raspi")
    parser.add_argument("--port", type=int, default=0,
                        help="the USB hub port with a connected device")

    known_args, remaining_args = parser.parse_known_args()

    create_test_config(known_args)
    run_test(known_args, remaining_args)

if __name__ == '__main__':
    main()
