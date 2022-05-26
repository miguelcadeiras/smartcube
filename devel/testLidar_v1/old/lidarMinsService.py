import time
import numpy as np
import math
import sc_services as scsvc

# Select the correct UART port automatically
TCP_PORT_CLIENT = scsvc.LIDAR_RAW
TCP_PORT = scsvc.LIDAR_MINS

minimos = []

# calcula los minimos
def calc_min(list_angulos,slices):
	arr = np.array(list_angulos[:-1])
	arr = arr.astype(int)
	arr[arr==-1] = 6000
	list_minimos = []
	for slice in slices:
		desde=slice[0]
		hasta=slice[1]
		m=min(arr[desde:hasta])
		list_minimos.append(m)
	return list_minimos

def calc_min2(list_angulos,slices):
	#arr = np.array(list_angulos[:-1])
	arr = np.array(list_angulos)
	arr = arr.astype(int)
	#arr[arr == -1] = 6000
	arr[arr < 10] = 6000
	list_minimos = []
	#print (arr)
	for slice in slices:
		distances_y = []
		desde=slice[0]
		hasta=slice[1]
		for ang in range (desde, hasta):
			y = -math.sin((ang - 90) * math.pi / 180) * (arr[ang])
			x = math.cos((ang - 90) * math.pi / 180) * (arr[ang])
			if abs(x) < 400 and abs(y) < 1000:
				#print (x,y)
				distances_y.append(y)
		if len(distances_y) > 0:
			m=min(distances_y)
			list_minimos.append(m)
	return list_minimos


#Esta rutina lee el puerto
def run(lock):
	from multiprocessing.connection import Client
	# init()
	address = ('localhost', TCP_PORT_CLIENT)
	conn = Client(address)
	global minimos
	try:
		while True:
			list_angulos = [[0, 90], [270, 359]]
			lidarAngles = conn.recv()
			local_minimos=calc_min2(lidarAngles, list_angulos)
			print(local_minimos)
			with lock:
				minimos=local_minimos
	finally:
		print ("CLIENT END!")



if __name__ == "__main__":
	import multiprocessing as mp
	import threading as th
	from multiprocessing.connection import Listener

	address = ('localhost', TCP_PORT)  # family is deduced to be 'AF_INET'
	listener = Listener(address)


	def conHandler(conn, listener, lock):
		print('connection accepted from', listener.last_accepted)
		while True:
			with lock:
				try:
					conn.send(minimos)
				except:
					print ("CANT SEND!")
					break
			time.sleep(0.1)
		print ("DISCONNECTED")
		conn.close()
		print("THREAD END")



	# Global lock
	g_lock = th.Lock()
	print ("INZONE Service started")
	thSerialRead = th.Thread(target=run, args=(g_lock,))
	thSerialRead.setDaemon(True)
	thSerialRead.start()
	print ("Reading LIDAR on PORT", TCP_PORT_CLIENT)
	print ("Listening TCP on PORT", TCP_PORT)
	try:
		while True:
			c = listener.accept()
			th.Thread(target=conHandler, args=(c, listener,g_lock,)).start()
	except KeyboardInterrupt:
	   print ("EXIT")
	listener.close()
	p.join()