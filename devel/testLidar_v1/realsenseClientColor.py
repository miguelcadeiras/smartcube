from multiprocessing.connection import Client
import cv2
import time
import sc_services
import numpy as np

address = (sc_services.REALSENSE_COLOR_SERVER, sc_services.REALSENSE_COLOR)
conn = Client(address)

cv2.namedWindow('image')


while True:

    color_image=conn.recv()
    #print (color_image.shape)
    time.sleep(0.010)
    cv2.imshow("image", color_image)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

conn.close()