import time
import logdata
import socketprovider
import threading as th
import sc_services
import numpy as np
import statistics
import math

SERVICENAME = "realsenseMinsService"
DEBUG = False
#DEBUG = True

DISTANCIA_BARRA=850
MARGEN_FRAME_REALSENSE=100
ALTURA_MIN=200
ALTURA=350

class dataColector:
	data = None
	lock = None
	thDataColector = None
	exit = None

	def getMinY(self, l_depth_image, margin=20, refDist=800):
		# Convert images to numpy arrays

		# depth_image = np.asanyarray(depth_frame.get_data()).astype('float')
		# depth_image[depth_image == 0] = np.nan
		# print(  "array_lenght:",len(depth_image[:170, margin]))
		size = l_depth_image.shape[1]
		# print("size",size,depth_image.shape)
		# print("np_array",depth_image[:170, margin])
		l_depth_image[l_depth_image == 0] = 5000
		minDist1List = []

		marginRight = size - margin
		minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)

		margin = margin - 20
		minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)

		margin = margin + 20 + 20
		minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)
		margin = margin + 20
		minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)

		minDist1 = int(statistics.median(minDist1List))

		# print( "mindist1: ",minDist1)
		minDist2 = np.min(l_depth_image[ALTURA_MIN:ALTURA, (marginRight - 10):marginRight]) - refDist

		minDist1 = minDist1 if not np.isnan(minDist1) else 5000.0
		minDist2 = minDist2 if not np.isnan(minDist2) else 5000.0
		# print(minDist1,minDist2)

		return (minDist1, minDist2)

	def runLoop(self, lock):

		from multiprocessing.connection import Client



		try:
			serviceAddress = ('localhost', sc_services.REALSENSE_DEPTH)
			conn = Client(serviceAddress)
		except:
			logdata.log("ERROR: Cant INIT REALSENSE MINS")
			return None

		try:
			while True:
				list_angulos = [[0, 90], [270, 359]]
				depth_image = conn.recv()
				minY = self.getMinY(depth_image, MARGEN_FRAME_REALSENSE, DISTANCIA_BARRA)
				if (DEBUG):
					logdata.log(minY)
				with lock:
					self.data = minY

				if (self.exit == True):
					break
		except Exception as e:
			logdata.log(e)
			logdata.log("ERROR: DataCollector Loop")

		logdata.log("Stopping DataCollector")

	def getData(self):
		with self.lock:
			return (self.data)

	def start(self):
		self.exit = False
		logdata.log("Starting DataCollector thread")
		self.lock = th.Lock()
		self.thDataColector = th.Thread(target=self.runLoop, args=(self.lock,))
		self.thDataColector.setDaemon(True)
		self.thDataColector.start()
		self.thDataColector.join(4)

	# time.sleep(4)

	def stop(self):
		self.exit = True


def serviceStart(pipeOut=None, pipeIn=None, setDebug=False):
	DEBUG = setDebug
	logdata.logPipe = pipeOut
	logdata.log("Starting:", SERVICENAME)

	dc = dataColector()
	sp = socketprovider.socketProvider()
	dc.start()
	sp.getData = dc.getData
	dcrunning = False
	if dc.thDataColector.is_alive():
		try:
			sp.start(sc_services.REALSENSE_MINS)
			dcrunning = True
		except KeyboardInterrupt:
			logdata.log("EXIT-2")

	while dcrunning == True:
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



if __name__ == "__main__":
	print("RUNNING STANDALONE")
	logdata.runningStandalone = True
	serviceStart()