from multiprocessing.connection import Client
import sc_services
import numpy as np

address = ('localhost', sc_services.JOYSTICK_RAW)
try:
	conn = Client(address)
except:
	print ("Connection failed:", address)
	quit()
while True:
	remCounter=conn.recv()
	print (remCounter)

conn.close()