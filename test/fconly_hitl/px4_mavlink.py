# PX4 command library

from pymavlink import mavutil
from pymavlink import mavwp
import sys, time, os, traceback
import logging

class px4mavlink:

	def __init__(self):
		self.master = None
		self.isConnected = False
		self.sConnectionString = None
		self.mavwp = None

		# PX4 connection params
		# Max retry time for a command is 60sec
		self.px4_command_retries = 6
		self.px4_retry_timer = 10

		# Mission statistics
		self.nextwaypoint = 0
		self.relative_alt = 0

		# Misc
		# Optional debug prints toggle
		self.msgDebug = True

	# Formatted print function
	# Todo: Add timestamps and file logging?
	def printFormat(self, sMsg, bDebugOnly=False):
		if bDebugOnly:
			if self.msgDebug:
				print(sMsg)
		else:
			print(sMsg)

	def commandRetryUntilTimeout(self):
		pass

	def commandRetryUntilSuccess(self, fCommand, lstArgs):
		self.printFormat('command retry until success')
		self.printFormat('Command function ID: %s' % id(fCommand), True)

		iArgCount = len(lstArgs)
		bSuccess = False
		nRetries = 0
		while bSuccess == False and nRetries < self.px4_command_retries:
			if iArgCount == 0:
				fCommand()
			elif iArgCount == 1:
				fCommand(lstArgs[0])
			elif iArgCount == 2:
				fCommand(lstArgs[0], lstArgs[1])
			elif iArgCount == 3:
				fCommand(lstArgs[0], lstArgs[1], lstArgs[2])
			elif iArgCount == 4:
				fCommand(lstArgs[0], lstArgs[1], lstArgs[2], lstArgs[3])
			elif iArgCount == 5:
				fCommand(lstArgs[0], lstArgs[1], lstArgs[2], lstArgs[3], lstArgs[4])
			else:
				print('Unsupported argument count: %s' % str(iArgCount))

			msg = self.master.recv_match(type=['COMMAND_ACK'],blocking=True)
			self.printFormat(msg, True)
			if msg.result == 0:
				bSuccess = True

			if bSuccess == False:
				time.sleep(self.px4_retry_timer)

		return bSuccess



	def connect(self, sConnectionString, iTimeoutSec=None):
		self.printFormat('px4mavlink.connect(%s, timeout=%s)' % (sConnectionString, str(iTimeoutSec)))
		self.master = mavutil.mavlink_connection(sConnectionString)
		self.master.wait_heartbeat(timeout=iTimeoutSec)
		self.printFormat('Connected and received HB')
		self.isConnected = True

	def loadWaypoints_mavlink(self, sPath):
		self.printFormat('loadWaypoints_mavlink')
		# waypoint loader should be a singleton?
		if self.mavwp == None:
			self.mavwp = mavwp.MAVWPLoader()

		try:
			self.mavwp.load(sPath)
			self.printFormat('Read %s waypoints' % str(self.mavwp.count()))
			return True
		except:
			self.printFormat(traceback.format_exc())
			return False

	# Upload waypoints
	# Load needs to be called before invoking upload
	def uploadWaypoints(self):
		self.printFormat('uploadWaypointMission')

		# Assert connection and waypoints loaded
		if not self.assertConnected() or not self.assertWaypointsLoaded():
			return False

		# Send "clear waypoints"
		self.master.waypoint_clear_all_send()

		# Send waypoint count to initialize upload sequence
		self.master.waypoint_count_send(self.mavwp.count())

		# Listen for MISSION_REQUEST and send a waypoint as a response to each
		for i in range(self.mavwp.count()):

			msg = self.master.recv_match(type=['MISSION_REQUEST'],blocking=True)
			self.printFormat(msg, True)

			# next waypoint and make sure target_system is set correctly
			waypoint = self.mavwp.wp(i)
			waypoint.target_system = self.master.target_system

			# send
			self.printFormat('sending waypoint', True)
			self.printFormat(waypoint, True)
			self.master.mav.send(waypoint)

		# Await mission ack message
		msg_ack = self.master.recv_match(type=['MISSION_ACK'], blocking=True)
		self.printFormat(msg_ack, True)

	def readWaypoints(self, bVerifyUploaded):
		self.printFormat('uploadWaypointMission')
		# Assert connection and waypoints loaded
		if not self.assertConnected():
			return False
		if bVerifyUploaded:
			if not self.assertWaypointsLoaded():
				return False

		# Request waypoint list
		self.master.waypoint_request_list_send()
		iWaypointCount = 0

		msg = self.master.recv_match(type=['MISSION_COUNT'],blocking=True)
		self.printFormat(msg, True)
		iWaypointCount = msg.count

		# Request each individual waypoint
		for i in range(iWaypointCount):
			self.master.waypoint_request_send(i)
			msg = self.master.recv_match(type=['MISSION_ITEM'],blocking=True)
			self.printFormat(msg, True)

		# Send confirmation
		self.master.mav.mission_ack_send(self.master.target_system, self.master.target_component, 0)

		if bVerifyUploaded:
			if iWaypointCount == self.mavwp.count():
				return True
			else:
				self.printFormat('Waypoint verification failed: invalid count %s (expected %s)' % (iWaypointCount, self.mavwp.count()))
				return False
		return True

	def setAndVerifyHomeLocation(self):
		self.printFormat('Set and verify home location')

		# Assert connection and waypoints loaded
		if not self.assertConnected() or not self.assertWaypointsLoaded():
			return False

		# Assume home location is the first waypoint(takeoff)
		oHomeLoc = self.mavwp.wp(0)
		if self.commandRetryUntilSuccess(self.commandSetHome, [[oHomeLoc.x, oHomeLoc.y], 0]) == False:
			return False

		self.printFormat('Set home location to %s, %s' % (oHomeLoc.x, oHomeLoc.y))
		time.sleep(0.2)

		oHomeLocRet = self.commandGetHome()
		self.printFormat('Got home location: %s %s %s' % (oHomeLocRet[0], oHomeLocRet[1], oHomeLocRet[2]))

		return True

	def setAutoMode(self):
		self.printFormat('Set auto mode')
		if self.commandRetryUntilSuccess(self.commandConfigureAutoMode, []) == False:
			return False
		return True

	def arm(self):
		print('Arming')
		if self.commandRetryUntilSuccess(self.commandArm, []) == False:
			print('Arming failed')
			return False
		print('Armed')
		return True


	def commandSetHome(self, lstHomeLoc, iAltitude):
		self.printFormat('CommandSetHome')
		self.master.mav.command_long_send(
		self.master.target_system, self.master.target_component,
		mavutil.mavlink.MAV_CMD_DO_SET_HOME,
			1, # set position
			0, # param1
			0, # param2
			0, # param3
			0, # param4
			lstHomeLoc[0], # lat
			lstHomeLoc[1], # lon
			iAltitude) # alt


	def commandGetHome(self):
		self.printFormat('CommandGetHome')
		self.master.mav.command_long_send(
			self.master.target_system, self.master.target_component,
			mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
		msg = self.master.recv_match(type=['COMMAND_ACK'],blocking=True)
		self.printFormat(msg, True)
		msg = self.master.recv_match(type=['HOME_POSITION'],blocking=True)
		self.printFormat(msg, True)
		return (msg.latitude, msg.longitude, msg.altitude)


	def commandConfigureAutoMode(self):
		self.printFormat('commandConfigureAutoMode')
		PX4_MAV_MODE = 209.0
		PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
		PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4.0
		PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5.0

		self.master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                            PX4_MAV_MODE,
                                            PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION, 0, 0, 0, 0)

	def commandArm(self):
		self.master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                      1,
                      0, 0, 0, 0, 0, 0)

	def sendHeartbeat(self):
		if self.assertConnected() == False:
			return False

		self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
		return True

	def handleGlobalPosition(self, msg):
		# Extract altitude
		self.relative_alt = msg.relative_alt
		return True

	def handleMissionCurrent(self, msg):
		if msg.seq > self.nextwaypoint:
			self.nextwaypoint = msg.seq # should this be +1?
			print('Waypoint reached, next way point: %s' % str(self.nextwaypoint))
		return self.nextwaypoint

	# Monitoring loop for a mission
	# Return True upon mission done (success)
	# Return False upon failure or timeout (failure)
	def monitorMissionLoop(self, iTimeOutMins=10):

		# Assert connection and waypoints loaded
		if not self.assertConnected() or not self.assertWaypointsLoaded():
			return False

		try:

			# Timeout, default 10min
			tTimeout = time.time() + 60*iTimeOutMins

			bSuccess = False
			while True:

				if time.time() > tTimeout:
					print('Mission timed out')

				# Read next message
				msg = self.master.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'MISSION_COUNT', 'HEARTBEAT'],blocking=True,timeout=0.5)

				# Reply to heartbeat
				if msg.get_type() == 'HEARTBEAT':
					self.sendHeartbeat()
				if msg.get_type() == 'GLOBAL_POSITION_INT':
					self.handleGlobalPosition(msg)
				if msg.get_type() == 'MISSION_CURRENT':
					nextwaypoint = self.handleMissionCurrent(msg)
					#print('nextwaypoint: %s, waypoint_count: %s' % (nextwaypoint, waypoint_count))
					if nextwaypoint >= self.mavwp.count() - 1:
						if self.relative_alt <= 1*1000*0.05:
							print("Reached land altitude")
							bSuccess = True
							break
				if msg.get_type() == 'MISSION_COUNT':
					print('MISSION_COUNT handler')
				time.sleep(0.1)

			return bSuccess

		except KeyboardInterrupt as kbi:
			print('Caught keyboardinterrupt')
			return False

		except Exception as ex:
			print(traceback.format_exc())
			return False

	# Reusable asserts
	def assertConnected(self):
		if self.master == None or self.isConnected == False:
			self.printFormat('Not connected')
			return False
		return True

	def assertWaypointsLoaded(self):
		if self.mavwp == None or self.mavwp.count() == 0:
			self.printFormat('No waypoints loaded')
			return False
		return True

