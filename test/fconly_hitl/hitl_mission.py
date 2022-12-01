#from mavsdk import System
#import mavsdk.mission_raw
#import asyncio

from pymavlink import mavutil
from pymavlink import mavwp
import sys, time, os, traceback
import logging
#from copy import deepcopy

# https://mavlink.io/en/mavgen_python/

# TODO: should the script emit a heartbeat?
# # Send heartbeat from a GCS (types are define as enum in the dialect file).
# master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# Adapted from
# https://gist.github.com/donghee/8d8377ba51aa11721dcaa7c811644169#file-mission_upload_and_download-py-L69

sConnectionString = 'udp::14540'
oConnection = None

# Max retry time for a command is 60sec
px4_command_retries = 6
px4_retry_timer = 10

bSuccess = False


def main():

    try:

        # connect and wait for heartbeat
        master = mavutil.mavlink_connection(sConnectionString)
        master.wait_heartbeat()
        print('Connected and received HB')

        # To convert .plan to .mavlink, use qgc-mavlink-converter (github)
        wp = load_mission_file('mission_ad.mavlink')

        if handle_home_location(master, wp) == False:
            raise Exception('Failed to set home location')
        waypoint_count = upload_mission(master, wp)
        configure_mode_auto(master)
        send_heartbeat(master)
        if arm(master) == False:
            raise Exception('Failed to arm')

        # Handle mission execution
        nextwaypoint = 0
        relative_alt = 0

        # 10min
        tTimeout = time.time() + 60*10

        while True:
            if time.time() > tTimeout:
                do_exit(False, 'Timeout 10min')

            msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'MISSION_COUNT', 'HEARTBEAT'],blocking=True,timeout=0.5)
            try:
                if msg.get_type() == 'HEARTBEAT':
                    send_heartbeat(master)
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    handle_global_position(msg)
                    relative_alt = msg.relative_alt
                if msg.get_type() == 'MISSION_CURRENT':
                    nextwaypoint = handle_mission_current(msg, nextwaypoint)
                    #print('nextwaypoint: %s, waypoint_count: %s' % (nextwaypoint, waypoint_count))
                    if nextwaypoint >= waypoint_count - 1:
                        if relative_alt <= 1*1000*0.05:
                            print("Reached land altitude")
                            do_exit(True)
                time.sleep(0.1)
                send_heartbeat(master)

            except KeyboardInterrupt:
                raise Exception('KeyboardInterrupt')

    except:
        if bSuccess == True:
            return 0
        else:
            print(traceback.format_exc())
            return 1


def do_exit(bRet, sMessage=None):
    if sMessage != None:
        print('sMessage')
    if bRet == True:
        bSuccess = True
        sys.exit(0)
    else:
        sys.exit(1)

# Shared function for commands for which we want await success
# such as setting home position or arming
def command_retry_until_success(master, fCommand, lstArgs):
    print('command_retry_until_success')
    print('Command function id: %s' % id(fCommand))

    iArgCount = len(lstArgs)

    bSuccess = False
    nRetries = 0
    while bSuccess == False and nRetries < px4_command_retries:
        if iArgCount == 0:
            fCommand(master)
        elif iArgCount == 1:
            fCommand(master, lstArgs[0])
        elif iArgCount == 2:
            fCommand(master, lstArgs[0], lstArgs[1])
        elif iArgCount == 3:
            fCommand(master, lstArgs[0], lstArgs[1], lstArgs[2])
        elif iArgCount == 4:
            fCommand(master, lstArgs[0], lstArgs[1], lstArgs[2], lstArgs[3])
        elif iArgCount == 5:
            fCommand(master, lstArgs[0], lstArgs[1], lstArgs[2], lstArgs[3], lstArgs[4])
        else:
            print('Unsupported argument count: %s' % str(iArgCount))

        msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
        print (msg)
        if msg.result == 0:
            bSuccess = True

        if bSuccess == False:
            time.sleep(px4_retry_timer)
    return bSuccess


def handle_global_position(msg):
    #print('handle_global_position')
    pass

def handle_home_location(master, wp):
    print('handle_home_location')
    # Set home location
    oHomeLoc = wp.wp(0)
    print (oHomeLoc)

    # Set home location with retries allowed
    #bSuccess = False
    #nRetries = 0
    #while bSuccess == False and nRetries < px4_command_retries:
    #       cmd_set_home([oHomeLoc.x, oHomeLoc.y], 0, master)
    #       msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    #       print (msg)
    #       if msg.result == 0:
    #               bSuccess = True

    if command_retry_until_success(master, cmd_set_home, [[oHomeLoc.x, oHomeLoc.y], 0]) == False:
        return False

    print ('Set home location: {0} {1}'.format(oHomeLoc.x, oHomeLoc.y))
    time.sleep(0.2)

    home_location = cmd_get_home(master)
    print ('Get home location: {0} {1} {2}'.format(home_location[0], home_location[1], home_location[2]))
    time.sleep(0.2)

    return True

def cmd_set_home(master, home_location, altitude):
    print('cmd_set_home')
    master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, # set position
            0, # param1
            0, # param2
            0, # param3
    0, # param4
    home_location[0], # lat
    home_location[1], # lon
    altitude) # alt

def cmd_get_home(master):
    print('cmd_get_home')
    master.mav.command_long_send(
            master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
    0, 0, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    print (msg)
    msg = master.recv_match(type=['HOME_POSITION'],blocking=True)
    return (msg.latitude, msg.longitude, msg.altitude)


def load_mission_file(sName):
    print('load_mission_file')
    wp = mavwp.MAVWPLoader()
    wp.load(sName)
    print('Read %s waypoints' % str(wp.count()))
    return wp

def upload_mission(master, wp):
    print('upload_mission')
    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    # Iterate list of waypoints and send a mission request for each
    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        waypoint = wp.wp(i)

        # Make sure target system is correct in the waypoint items before sending
        waypoint.target_system = master.target_system
        master.mav.send(waypoint)
        print('Sending waypoint {0}'.format(i))

    # Await mission ack after the announced number of waypoints has been sent
    mission_ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True)
    print(mission_ack_msg.type)

    # Confirm waypoint upload to target
    master.waypoint_request_list_send()
    waypoint_count = 0

    msg = master.recv_match(type=['MISSION_COUNT'],blocking=True)
    waypoint_count = msg.count
    print ("Confirm number of waypoints on target: %s" % msg.count)

    for i in range(waypoint_count):
        master.waypoint_request_send(i)
        msg = master.recv_match(type=['MISSION_ITEM'],blocking=True)
        print ('Receving waypoint {0}'.format(msg.seq))
        print (msg)


    master.mav.mission_ack_send(master.target_system, master.target_component, 0) # OKAY

    print(msg)

    return waypoint_count


# https://discuss.px4.io/t/mav-cmd-do-set-mode-all-possible-modes/8495/2
# Manual: base_mode:217, main_mode:1, sub_mode:0
# Stabilized: base_mode:209, main_mode:7, sub_mode:0
# Acro: base_mode:209, main_mode:5, sub_mode:0
# Rattitude: base_mode:193, main_mode:8, sub_mode:0
# Altitude: base_mode:193, main_mode:2, sub_mode:0
# Offboard: base_mode:209, main_mode:6, sub_mode:0
# Position: base_mode:209, main_mode:3, sub_mode:0
# Hold: base_mode:217, main_mode:4, sub_mode:3
# Missition: base_mode:157, main_mode:4, sub_mode:4
# Return: base_mode:157, main_mode:4, sub_mode:5
# Follow me: base_mode:157, main_mode:4, sub_mode:8
def configure_mode_auto(master):
    print('configure_mode_auto')
    PX4_MAV_MODE = 209.0;
    PX4_CUSTOM_MAIN_MODE_AUTO = 4.0;
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4.0
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5.0;

    master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                            PX4_MAV_MODE,
                                            PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION, 0, 0, 0, 0)

    msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)

def arm_command(master):
    master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                      1,
                      0, 0, 0, 0, 0, 0)

def arm(master):
    print('arm')
    if command_retry_until_success(master, arm_command, []) == False:
        print('Arming failed :(')
        return False
    print('Armed!')
    return True

def send_heartbeat(master):
    #print('send_heartbeat')
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)

def handle_mission_current(msg, nextwaypoint):
#    print('handle_current_mission')
#    print(msg)
    if msg.seq > nextwaypoint:
        print ("Moving to waypoint %s" % msg.seq)
        nextwaypoint = msg.seq + 1
        print ("Next Waypoint %s" % nextwaypoint)
    return nextwaypoint

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
