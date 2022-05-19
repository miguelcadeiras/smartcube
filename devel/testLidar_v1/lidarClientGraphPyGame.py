from multiprocessing.connection import Client
import numpy as np
import pygame
import math
import sc_services as scsvc

pygame.init()
pygame.font.init()

width=850
height=850
centerX=width/2
centerY=height/2

window = pygame.display.set_mode((width, height))
font = pygame.font.Font('freesansbold.ttf', 16)


window.fill((0, 0, 0))
pygame.display.update()

address = ('localhost', scsvc.LIDAR_RAW)
conn = Client(address)

address2 = ('localhost', scsvc.REALSENSE_MINS)
conn2 = Client(address2)

while True:
	pygame.event.pump()
	np_distance=conn.recv()
	realsenseMins= conn2.recv()
	#realsenseMins=(1000,1000)
	#np_distance[np_distance == -1] = 600
	window.fill((0, 0, 0))
	#print (np_distance)
	print (realsenseMins)
	scale = 10

	pygame.draw.circle(window, 'red', [centerX, centerY], 10, 1)
	pygame.draw.circle(window, 'red', [centerX, centerY], 25, 1)
	pygame.draw.circle(window, 'red', [centerX, centerY], 50, 1)
	pygame.draw.circle (window, 'yellow', [centerX, centerY], 100, 2)
	pygame.draw.circle(window, 'green', [centerX, centerY], 200, 2)
	pygame.draw.circle(window, 'green', [centerX, centerY], 300, 2)
	pygame.draw.circle(window, 'green', [centerX, centerY], 400, 2)

	text = font.render('50', True, 'red')
	window.blit(text, (centerX + 50, centerY - 16))
	text = font.render('100', True, 'yellow')
	window.blit(text, (centerX + 100, centerY - 16))
	text = font.render('200', True, 'green')
	window.blit(text, (centerX + 200, centerY - 16))
	text = font.render('300', True, 'green')
	window.blit(text, (centerX + 300, centerY - 16))
	text = font.render('400', True, 'green')
	window.blit(text, (centerX + 400, centerY - 16))

	pygame.draw.rect(window, (0,100,255), (centerX-35, centerX, 70, 120), 3)

	pygame.draw.rect(window, (50, 50, 50), (centerX - 40, centerY-50, 80, 50), 2)

	pygame.draw.line(window, 'yellow', [centerX + realsenseMins[0] / 10, centerY ], [centerX + realsenseMins[1] / 10, centerY+120], 4)

	#if (np_distance[0] == -1):
	#	np_distance[0]=4000
	#ang = 0

	#py = math.sin((ang-90) * math.pi / 180) * (np_distance[ang] / scale);
	#px = -math.cos((ang-90) * math.pi / 180) * (np_distance[ang] / scale);
	counter=0
	for ang in range(0, 360):
		if (np_distance[ang] > 0):
			counter+=1
			y = math.sin((ang - 90) * math.pi / 180) * (np_distance[ang] / scale)
			x = -math.cos((ang - 90) * math.pi / 180) * (np_distance[ang] / scale)
			pygame.draw.circle(window, (200,200,200), [centerX+x, centerY+y], 3, 2)
	text = font.render(str(counter), True, 'red')
	window.blit(text, (50,50))
	# for ang in range(1, 360):
	# 	if (np_distance[ang] > 0):
	# 		y = math.sin((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);
	# 		x = -math.cos((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);
	#
	# 		if (ang > 20 and ang < 340):
	# 			pygame.draw.line(window, (100,100,100),[centerX-px, centerY+py], [centerX-x, centerY+y],2)
	# 		else:
	# 			pygame.draw.line(window, 'white', [centerX - px, centerY + py], [centerX - x, centerY + y], 1)
	# 		px = x
	# 		py = y
	# ang = 0
	# y = math.sin((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);
	# x = -math.cos((ang - 90) * math.pi / 180) * (np_distance[ang] / scale);
	# pygame.draw.line(window, 'white', [centerX - px, centerY + py], [centerX - x, centerY + y], 1)
	pygame.display.update()

conn.close()