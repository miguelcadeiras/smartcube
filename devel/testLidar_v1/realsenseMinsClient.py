from multiprocessing.connection import Client
import numpy as np

address = ('10.0.0.80', 6022)
conn = Client(address)
while True:
	remCounter=conn.recv()
	print (remCounter)

conn.close()