import pyrealsense2 as rs
import numpy as np
import cv2
import time

depth_image = []
color_image = []

# Configure depth and color streams
pipeline = rs.pipeline()
global config
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Hardware Reset
#print("RESETING REALSENSE DEVICE...")
#device.hardware_reset()
#time.sleep(5)
found_rgb = False


def startSense():
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    # Start streaming
    pipeline.start(config)


try:
    print("Starting RealSense.. ")
    startSense()
    time.sleep(4)
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor (color_image, cv2.COLOR_BGR2GRAY)
        depth_image[depth_image > 1000] = 0
        #limitado=color_image
        limitado = np.where(depth_image > 200, gray_image, 0)
        if not depth_frame:
            continue

        cv2.imshow("image", gray_image)
        cv2.imshow("limitado", limitado)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        #cv2.imshow("original", color_image)
        #cv2.imshow("limitado", limitado)




finally:
    print("REALSENSE ERROR!")