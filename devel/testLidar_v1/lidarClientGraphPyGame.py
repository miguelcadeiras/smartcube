from multiprocessing.connection import Client
import time
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

widthAuto=70
heightAuto=120

window = pygame.display.set_mode((width, height))
font = pygame.font.Font('freesansbold.ttf', 16)


window.fill((0, 0, 0))
pygame.display.update()
connLidar=None
connRSMins=None
connPixy=None

try:
	address = ('localhost', scsvc.LIDAR_RAW)
	connLidar = Client(address)
except Exception as e:
	print ("ERROR:", e)

try:
	address2 = ('localhost', scsvc.REALSENSE_MINS)
	connRSMins = Client(address2)
except Exception as e:
	print ("ERROR:", e)
try:
	address3 = ('localhost', scsvc.PIXY_RAW)
	connPixy = Client(address3)
except Exception as e:
	print ("ERROR:", e)

np_distance=None
realsenseMins=None
pixyVector=None


while True:
	pygame.event.pump()
	strStatus="STATUS: "
	if connLidar is not None:
		if connLidar.poll():
			np_distance=connLidar.recv()
	else:
		strStatus+="NO LIDAR "
	if connRSMins is not None:
		if connRSMins.poll():
			realsenseMins= connRSMins.recv()
	else:
		strStatus+="NO REALSENSE "

	if connPixy is not None:
		if connPixy.poll():
			pixyVector=connPixy.recv()
	else:
		strStatus+="NO PIXY "


	window.fill((0, 0, 0))
	text = font.render(strStatus, True, 'red')
	window.blit(text, (50, 10))

	#realsenseMins=(1000,1000)
	#np_distance[np_distance == -1] = 600

	#print (np_distance)
	#print (realsenseMins)
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

	if pixyVector is not None:
		pixyScale=1.3
		pixyWidth=78/pixyScale
		pixyHeight=52/pixyScale

		pixyFrameOriginX = centerX-pixyWidth/2
		pixyFrameOriginY = centerY
		pixyX0=pixyVector['x0'] / pixyScale
		pixyY0=pixyVector['y0'] / pixyScale
		pixyX1 = pixyVector['x1'] / pixyScale
		pixyY1 = pixyVector['y1'] / pixyScale
		pygame.draw.rect(window, (255, 255, 255), (pixyFrameOriginX, pixyFrameOriginY, pixyWidth, pixyHeight), 1)
		if pixyVector['v'] > 0:
			pygame.draw.circle(window, (255, 255, 255), [pixyFrameOriginX + pixyX1, pixyFrameOriginY + pixyY1], 5, 3)
			pygame.draw.line(window, 'yellow', [pixyFrameOriginX+pixyX0, pixyFrameOriginY+pixyY0],
			[pixyFrameOriginX+pixyX1, pixyFrameOriginY+pixyY1], 2)

	if realsenseMins is not None:
		pygame.draw.line(window, 'yellow', [centerX + realsenseMins[0] / 10, centerY ], [centerX + realsenseMins[1] / 10, centerY+120], 4)

	if np_distance is not None:
		counter=0
		for ang in range(0, 360):
			if (np_distance[ang] > 0):
				counter+=1
				y = math.sin((ang - 90) * math.pi / 180) * (np_distance[ang] / scale)
				x = -math.cos((ang - 90) * math.pi / 180) * (np_distance[ang] / scale)
				pygame.draw.circle(window, (200,200,200), [centerX+x, centerY+y], 3, 2)
		text = font.render(str(counter), True, 'red')
		window.blit(text, (50,50))

	pygame.display.update()
	time.sleep(0.040)

conn.close()