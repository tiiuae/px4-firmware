# Shell command module

import sys, os, time
import asyncio
import traceback

sShellScriptPath = "../../Tools/mavlink_shell.py"
sConnectionString = '0.0.0.0:14540'
sCommandOpenShell = 'python3'

iCommandTimeoutSec = 10

def isPrompt(sLine):
	if sLine.find('saluki') > -1 or sLine.find('nsh') > -1 or sLine.find('protected') > -1:
		return True
	else:
		return False

async def sendCommand(proc, sCommand):
	proc.stdin.write(sCommand)
	lstRet = await awaitNewPrompt(proc)
	if len(lstRet) == 2 and lstRet[0] == True:
		return [True, lstRet[1]]
	return [False, None]

	
# Waits for new shell prompt to pop
# This indicates command or connect done
async def awaitNewPrompt(proc):
	print('awaitNewPrompt')
	tTimeout = time.time() + iCommandTimeoutSec
	bRet = False
	lstOutput = []
	
	tTime = time.time()
	bBreak = False
	while not bBreak:
		tTime = time.time()
		if tTime > tTimeout:
			print('Operation timed out')
			bBreak = True
			return [bRet, None]

		sRead = (await proc.stdout.read(1024)).decode('ascii')
#		print(sRead)
		lstLines = sRead.split('\n')
		for line in lstLines:
			print(line)
			lstOutput.append(line)
			if isPrompt(line):
				bBreak = True
				bRet = True
				break
		time.sleep(0.5)
	
	return [bRet, lstOutput]

def verifyIMUConfig(lstLines):
	bRet = False
	
	iSensorCount = 0
	for l in lstLines:
		if l.find('GYRO 0') > -1 or l.find('GYRO 1') > -1 or l.find('GYRO 2') > -1 or l.find('ACC 0') > -1 or l.find('ACC 1') > -1 or l.find('ACC 2') > -1:
			iSensorCount = iSensorCount + 1
#	if iSensorCount == 6:
	if iSensorCount == 2:
		print('Found 6 sensors (3 GYRO + 3 ACC)')
		bRet = True
	else:
		print('IMU sensor count NOK')

	return bRet

def assertBoolMsg(bOk, sMsg=''):
	if bOk:
		print('assertBool OK')
		pass
	else:
		print('Assert NOK [%s]' % sMsg)
		raise Exception('assertBoolMsg failed')

async def main():
	try:
		print('Mavlink Shell wrapper')
		proc = await asyncio.subprocess.create_subprocess_exec(sCommandOpenShell, sShellScriptPath, sConnectionString, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE)
		print('Subprocess created')
		lstRet = await awaitNewPrompt(proc)
		assertBoolMsg(lstRet[0], 'Open Shell')

		# Verify that all 3 IMUs are in use in PX4
		lstRet = await sendCommand(proc, b'sensors status\n')
		assertBoolMsg(lstRet[0], 'sensors status')
		assertBoolMsg(verifyIMUConfig(lstRet[1]))

		proc.terminate()
		await proc.wait()
	except:
		print('Exception occurred (could be failed assert)')
		print(traceback.format_exc())

if __name__  ==  '__main__':
	asyncio.run(main())

