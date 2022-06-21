import time
import logdata
import socketprovider
import threading as th
import sc_services
import numpy as np
import math

SERVICENAME = "lidarMinsService"
DEBUG = False


class dataColector:
	data = None
	lock = None
	thDataColector = None
	exit = None

	def calc_min2(self, list_angulos, slices):
		# arr = np.array(list_angulos[:-1])
		arr = np.array(list_angulos)
		arr = arr.astype(int)
		# arr[arr == -1] = 6000
		arr[arr < 10] = 6000
		list_minimos = []
		# print (arr)
		for slice in slices:
			distances_y = []
			desde = slice[0]
			hasta = slice[1]
			for ang in range(desde, hasta):
				y = -math.sin((ang - 90) * math.pi / 180) * (arr[ang])
				x = math.cos((ang - 90) * math.pi / 180) * (arr[ang])
				if abs(x) < 400 and abs(y) < 1000:
					# print (x,y)
					distances_y.append(y)
			if len(distances_y) > 0:
				m = min(distances_y)
				list_minimos.append(m)
		return list_minimos

	def runLoop(self, lock):

		from multiprocessing.connection import Client
		import numpy as np

		try:
			lidarServiceAddress = ('localhost', sc_services.LIDAR_RAW)
			conn = Client(lidarServiceAddress)
		except:
			logdata.log("ERROR: Cant INIT LIDAR MINS")
			return None

		try:
			while True:
				list_angulos = [[0, 90], [270, 359]]
				lidarAngles = conn.recv()
				local_minimos = self.calc_min2(lidarAngles, list_angulos)
				if (DEBUG):
					logdata.log(local_minimos)
				with lock:
					self.data = local_minimos.copy()
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
			sp.start(sc_services.LIDAR_MINS)
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