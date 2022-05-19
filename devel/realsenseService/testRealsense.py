import socket  # Import socket module
import pyrealsense2 as rs
import keyboard
import numpy as np
import matplotlib.pyplot as plt
import time
import serial
import math


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False

arduino = serial.Serial(port='COM7', baudrate=9600, timeout=.1)

refereneDistance = 1200

def write_read(x):
    dataStr = "{"+str(int(x)) + "}"+"\n"
    #arduino.write(bytes(dataStr, 'utf-8'))
    arduino.write(dataStr.encode('utf-8'))
    data = arduino.readline()

    print('Nano says: ', data)



def startSense():
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    # Start streaming
    pipeline.start(config)


def GetMinY(depthFrame, margin=20, refDist=800):
    # Convert images to numpy arrays


    depth_image = np.asanyarray(depth_frame.get_data()).astype('float')
    # depth_image[depth_image == 0] = np.nan
     # print(  "array_lenght:",len(depth_image[:170, margin]))
    size = depth_image.shape[1]
    # print("size",size,depth_image.shape)
    # print("np_array",depth_image[:170, margin])
    depth_image[depth_image == 0] = 5000
    minDist1 = min(depth_image[:170, margin]) - refDist

    # print( "mindist1: ",minDist1)
    minDist2 = min(depth_image[:, size - margin]) - refDist

    minDist1 = minDist1 if not np.isnan(minDist1) else 5000.0
    minDist2 = minDist2 if not np.isnan(minDist2) else 5000.0
    # print(minDist1,minDist2)

    return (minDist1, minDist2)


try:
    print( "starting Sense.. ")
    startSense()

except Exception as e:
    print("startingSense ex:" + str(e))

while True:
    try:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        # color_frame = frames.get_color_frame()

        if not depth_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())


        vector = GetMinY(depth_frame, 150, refereneDistance)

        if vector[0]!=5000:
            write_read(vector[0])
            time.sleep(0.1)
            print('rs write to nano:',vector[0])
            # print(vector)

        if keyboard.is_pressed('q'):
            exit(0)

    except:
        pass





