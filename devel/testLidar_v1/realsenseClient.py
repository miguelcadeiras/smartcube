from multiprocessing.connection import Client
import cv2
#import pygame
import time
import numpy as np

#pygame.init()

#window = pygame.display.set_mode((700, 700))
#window.fill((0, 0, 0))
#pygame.display.update()


address = ('localhost', 6021)
conn = Client(address)
lastTime=0
mouseX=0
mouseY=0


def mouseEvent(event,x,y,flags,param):
    global mouseX, mouseY
    mouseX=x
    mouseY=y

cv2.namedWindow('image')
cv2.setMouseCallback('image',mouseEvent)

while True:
    #pygame.event.pump()
    #color_image=conn.recv()
    depth_image = conn.recv()
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    #cv2.imshow("original", color_image)
    depthValue=depth_image[mouseY, mouseX]
    texto=str(depthValue)+ " (" + str(mouseX) + "," + str(mouseY) + ")"
    cv2.putText(depth_colormap, texto, (10,40), cv2.FONT_HERSHEY_SIMPLEX,1, (255,255,255), 2,cv2.LINE_AA)

    #cv2.imshow("image", depth_colormap[:100,100:300])
    cv2.imshow("image", depth_colormap)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break


conn.close()