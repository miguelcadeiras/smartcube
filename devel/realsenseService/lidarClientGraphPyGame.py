from multiprocessing.connection import Client
import numpy as np
import pygame
import math

pygame.init()

window = pygame.display.set_mode((700, 700))
window.fill((0, 0, 0))
pygame.display.update()

address = ('localhost', 6000)
conn = Client(address)
while True:
	pygame.event.pump()
	np_distance=conn.recv()
	#np_distance[np_distance == -1] = 600
	window.fill((0, 0, 0))
	#print (np_distance)
	scale = 10
	pygame.draw.circle(window, 'green', [350, 350], 10, 1)
	pygame.draw.circle(window, 'green', [350, 350], 25, 1)
	pygame.draw.circle(window, 'green', [350, 350], 50, 1)
	pygame.draw.circle (window, 'green', [350,350], 100,1)
	pygame.draw.circle(window, 'green', [350, 350], 200, 1)
	pygame.draw.rect(window, (0,100,255), (350-40, 350, 80, 130), 3)

	"""
	myCanvas.create_oval(350 - 100, 350 - 100, 350 + 100, 350 + 100, outline="green")
	myCanvas.create_oval(350 - 200, 350 - 200, 350 + 200, 350 + 200, outline="green")
	myCanvas.create_oval(350 - 10, 350 - 10, 350 + 10, 350 + 10, outline="green")
	myCanvas.create_oval(350 - 50, 350 - 50, 350 + 50, 350 + 50, outline="green")
	myCanvas.create_oval(350 - 25, 350 - 25, 350 + 25, 350 + 25, outline="green")
	myCanvas.create_rectangle(350 - 40, 350, 350 + 40, 350 + 130, outline="purple")
	"""

	ang = 0
	py = math.sin(ang-90 * math.pi / 180) * (np_distance[ang] / scale);
	px = -math.cos(ang-90 * math.pi / 180) * (np_distance[ang] / scale);


	for ang in range(1, 360):
		if (np_distance[ang] > 10):
			y = math.sin((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);
			x = -math.cos((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);

			if (ang > 20 and ang < 340):
				pygame.draw.line(window, 'red',[350-px, 350+py], [350-x, 350+y],1)
			#	myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="red")
			else:
				pygame.draw.line(window, 'white', [350 - px, 350 + py], [350 - x, 350 + y], 1)
			#	myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="white")
			px = x
			py = y
	ang = 0
	y = math.sin(ang - 90 * math.pi / 180) * (np_distance[ang] / scale);
	x = -math.cos(ang - 90 * math.pi / 180) * (np_distance[ang] / scale);
	pygame.draw.line(window, 'white', [350 - px, 350 + py], [350 - x, 350 + y], 1)
	pygame.display.update()
	#myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="white")
conn.close()