#from mavsdk import System
#import mavsdk.mission_raw
#import asyncio

from pymavlink import mavutil
from pymavlink import mavwp
import sys, time, os, traceback
import logging
from px4_mavlink import px4mavlink

# https://mavlink.io/en/mavgen_python/

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

        oConn = px4mavlink()

        # Establish connection
        oConn.connect(sConnectionString, 15)

        # To convert .plan to .mavlink, use qgc-mavlink-converter (github)
        # wp = load_mission_file('mission_ad.mavlink')
        if oConn.isConnected == False:
            raise Exception('Failed to connect')

        if oConn.loadWaypoints_mavlink('mission_ad.mavlink') == False:
            raise Exception('Failed to load waypoints')

        if oConn.setAndVerifyHomeLocation() == False:
            raise Exception('Failed to set home location')

        if oConn.uploadWaypoints() == False:
            raise Exception('Failed to upload waypoints')

        if oConn.readWaypoints(True) == False:
            raise Exception('Waypoints verification failed')

        if oConn.setAutoMode() == False:
            raise Exception('Failed to set auto mode')

        oConn.sendHeartbeat()

        if oConn.arm() == False:
            raise Exception('Failed to arm')

        if oConn.monitorMissionLoop() == True:
            bSuccess = True
            do_exit(True)
        else:
            do_exit(False)

    except:
        if bSuccess == True:
            sys.exit(0)
        else:
            print(traceback.format_exc())
            sys.exit(1)


def do_exit(bRet, sMessage=None):
    if sMessage != None:
        print('sMessage')
    if bRet == True:
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
