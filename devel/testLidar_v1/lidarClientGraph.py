from multiprocessing.connection import Client
import numpy as np
import tkinter
import math

tkroot = tkinter.Tk()
myCanvas = tkinter.Canvas(tkroot, bg="black", height=700, width=700)
myCanvas.pack()
tkroot.update()

address = ('localhost', 6000)
conn = Client(address)
while True:
	tkroot.update()
	np_distance=conn.recv()
	#np_distance[np_distance == -1] = 600
	myCanvas.delete('all')
	#print (np_distance)
	scale = 10

	myCanvas.create_oval(350 - 100, 350 - 100, 350 + 100, 350 + 100, outline="green")
	myCanvas.create_oval(350 - 200, 350 - 200, 350 + 200, 350 + 200, outline="green")
	myCanvas.create_oval(350 - 10, 350 - 10, 350 + 10, 350 + 10, outline="green")
	myCanvas.create_oval(350 - 50, 350 - 50, 350 + 50, 350 + 50, outline="green")
	myCanvas.create_oval(350 - 25, 350 - 25, 350 + 25, 350 + 25, outline="green")
	myCanvas.create_rectangle(350 - 40, 350, 350 + 40, 350 + 130, outline="purple")


	ang = 0
	py = math.sin(ang-90 * math.pi / 180) * (np_distance[ang] / scale);
	px = -math.cos(ang-90 * math.pi / 180) * (np_distance[ang] / scale);
	#myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="yellow")

	for ang in range(1, 360):
		if (np_distance[ang] > 10):
			y = math.sin((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);
			x = -math.cos((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);

			if (ang > 20 and ang < 340):
				myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="red")
			else:
				myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="white")
			px = x
			py = y
	ang = 0
	y = math.sin(ang - 90 * math.pi / 180) * (np_distance[ang] / scale);
	x = -math.cos(ang - 90 * math.pi / 180) * (np_distance[ang] / scale);
	myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="white")
conn.close()