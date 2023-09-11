from multiprocessing.connection import Client
import sc_services
import datetime
import numpy as np

#address = ('localhost', sc_services.REALSENSE_MINS)
address = (sc_services.REALSENSE_MINS_SERVER, sc_services.REALSENSE_MINS)
conn = Client(address)
while True:
	remCounter=conn.recv()
	print (datetime.datetime.now(),remCounter)

conn.close()
