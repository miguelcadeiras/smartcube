from multiprocessing.connection import Client
import numpy as np

address = ('localhost', 6001)
conn = Client(address)
while True:
	remCounter=conn.recv()
	print (remCounter)

conn.close()