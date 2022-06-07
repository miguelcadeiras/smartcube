from multiprocessing.connection import Client
import sc_services
import numpy as np

address = ('localhost', sc_services.LIDAR_RAW)
conn = Client(address)
while True:
	remCounter=conn.recv()
	print (remCounter)

conn.close()
