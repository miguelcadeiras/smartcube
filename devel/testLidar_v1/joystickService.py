from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import time
import logdata
import socketprovider
import threading as th
import sc_services

SERVICENAME="joystickService"
DEBUG=False

class dataColector:
	data=None
	lock=None 
	thDataColector=None
	
	def runLoop(self,lock):
		try:
			import pygame	
			import pygame.joystick as joystick
			pygame.init()
			joystick.init()
			joyCount=joystick.get_count()
			logdata.log ("DETECTED JOYSTICKS:", joyCount)
			joy=joystick.Joystick(0)
			joy.init()
			joyName=joy.get_name()
			joyAxes=joy.get_numaxes()
			logdata.log ("Name:", joyName, "Axes:", joyAxes)
		except:
			logdata.log ("ERROR: Cant INIT Pygame / Joystick")
			return None
		
		while True:
			try:
				pygame.event.pump()
				joyData=[round (joy.get_axis(0), 2), round (joy.get_axis(1), 2), round (joy.get_axis(2), 2), round (joy.get_axis(3), 2) ]
				if (DEBUG):
					logdata.log (joyData)

				with lock:
					self.data=joyData
				if (self.exit == True):
					break
				time.sleep(0.01)
			except:
				logdata.log ("ERROR: DataCollector Loop")
		logdata.log ("Stopping DataCollector")
	def getData(self):
		with self.lock:
			return (self.data)
	
	def start(self):
		self.exit=False
		logdata.log("Starting DataCollector thread")
		self.lock=th.Lock()
		self.thDataColector = th.Thread(target=self.runLoop, args=(self.lock,))
		self.thDataColector.setDaemon(True)
		self.thDataColector.start()
		self.thDataColector.join(4)
		#time.sleep(4)
	
	def stop(self):
		self.exit=True




def serviceStart(pipeOut=None, pipeIn=None, setDebug=False):
	DEBUG=setDebug
	logdata.logPipe=pipeOut
	logdata.log("Starting:", SERVICENAME)

	dc=dataColector()
	sp=socketprovider.socketProvider()
	dc.start()
	sp.getData=dc.getData
	dcrunning=False
	if dc.thDataColector.is_alive():
		try:
			sp.start(sc_services.JOYSTICK_RAW)
			dcrunning=True
		except KeyboardInterrupt:
			logdata.log("EXIT-2")


	while dcrunning==True:
		try:
			if dc.thDataColector.is_alive():
				dcrunning = True
			else:
				dcrunning = False
			time.sleep(0.1)
		except KeyboardInterrupt:
			logdata.log("EXIT-3")
			break
	dc.stop()
	sp.stop()
	logdata.log("Service Stopping")
	time.sleep(5)
	logdata.log("Service STOPPED")
	#joyStickHandler()

if __name__ == "__main__":
	print ("RUNNING STANDALONE")
	logdata.runningStandalone=True
	serviceStart()
	