import time
import pyrealsense2 as rs
import numpy as np

# Select the correct UART port automatically
TCP_PORT_IMAGE = 6020
TCP_PORT_DEPTH = 6021
TCP_PORT_MIN = 6022

DISTANCIA_BARRA=1550
MARGEN_FRAME_REALSENSE=150
ALTURA_MIN=0
ALTURA=210

depth_image = []
color_image = []


#Esta rutina lee el realsense
def run(lock):
    from multiprocessing.connection import Client
    import pyrealsense2 as rs
    import numpy as np



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
    print ("RESETING REALSENSE DEVICE...")
    device.hardware_reset()
    time.sleep (5)

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
        global depth_image
        global color_image
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame:
                continue

            local_depth_image = np.asanyarray(depth_frame.get_data())
            local_color_image = np.asanyarray(color_frame.get_data())
            with lock:
                depth_image=local_depth_image
                color_image=local_color_image
            #print(depth_image.shape)
            #print(color_image.shape)




    finally:
        print ("REALSENSE ERROR!")



if __name__ == "__main__":
    import threading as th


    #address_d = ('localhost', TCP_PORT_DEPTH)  # family is deduced to be 'AF_INET'
    #listener_d = Listener(address_d)



    def listenerImage(g_lock):
        from multiprocessing.connection import Listener
        address = ('localhost', TCP_PORT_IMAGE)  # family is deduced to be 'AF_INET'
        listener = Listener(address)

        def conHandler(conn, listener, lock):
            print('connection accepted from', listener.last_accepted)
            global color_image
            while True:
                with lock:
                    try:
                        conn.send(color_image)
                    except:
                        print("CANT SEND!")
                        break;
                time.sleep(0.1)
            print("DISCONNECTED")
            conn.close()
            print("THREAD END")


        print("Listening TCP on PORT", TCP_PORT_IMAGE)
        try:
            while True:
                c = listener.accept()
                th.Thread(target=conHandler, args=(c, listener, g_lock,)).start()
        except KeyboardInterrupt:
            print("EXIT")
        listener.close()

    def listenerDepth(g_lock):
        from multiprocessing.connection import Listener
        address = ('localhost', TCP_PORT_DEPTH)  # family is deduced to be 'AF_INET'
        listener = Listener(address)

        def conHandler(conn, listener, lock):
            print('connection accepted from', listener.last_accepted)
            global depth_image
            while True:
                with lock:
                    try:
                        conn.send(depth_image)
                    except:
                        print("CANT SEND!")
                        break
                time.sleep(0.1)
            print("DISCONNECTED")
            conn.close()
            print("THREAD END")


        print("Listening TCP on PORT", TCP_PORT_DEPTH)
        try:
            while True:
                c = listener.accept()
                th.Thread(target=conHandler, args=(c, listener, g_lock,)).start()
        except KeyboardInterrupt:
            print("EXIT")
        listener.close()

    def listenerMin(g_lock):
        from multiprocessing.connection import Listener
        import statistics
        address = ('localhost', TCP_PORT_MIN)  # family is deduced to be 'AF_INET'
        listener = Listener(address)

        def GetMinY(l_depth_image, margin=20, refDist=800):
            # Convert images to numpy arrays

            #depth_image = np.asanyarray(depth_frame.get_data()).astype('float')
            # depth_image[depth_image == 0] = np.nan
            # print(  "array_lenght:",len(depth_image[:170, margin]))
            size = l_depth_image.shape[1]
            # print("size",size,depth_image.shape)
            # print("np_array",depth_image[:170, margin])
            l_depth_image[l_depth_image == 0] = 5000
            minDist1List = []
            #minDist1 = np.min(depth_image[:ALTURA, (margin-10):margin]) - refDist
            marginRight = size - margin
            minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)

            margin=margin-20
            minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)

            margin = margin + 20 +20
            minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)
            margin = margin + 20
            minDist1List.append(np.min(l_depth_image[ALTURA_MIN:ALTURA, (margin - 10):margin]) - refDist)

            minDist1=int(statistics.median(minDist1List))

             # print( "mindist1: ",minDist1)
            minDist2 = np.min(l_depth_image[ALTURA_MIN:ALTURA, (marginRight-10):marginRight]) - refDist

            minDist1 = minDist1 if not np.isnan(minDist1) else 5000.0
            minDist2 = minDist2 if not np.isnan(minDist2) else 5000.0
            #print(minDist1,minDist2)

            return (minDist1, minDist2)

        def conHandler(conn, listener, lock):
            print('connection accepted from', listener.last_accepted)
            #global depth_image
            #local_depth_image=[]

            #local_depth_image = []
            while True:

                with lock:
                    local_depth_image=np.asarray(depth_image)

                minY = GetMinY(local_depth_image, MARGEN_FRAME_REALSENSE, DISTANCIA_BARRA)
                try:
                    conn.send(minY)
                except:
                    print("CANT SEND!")
                    break
                time.sleep(0.1)
            print("DISCONNECTED")
            conn.close()
            print("THREAD END")


        print("Listening TCP on PORT", TCP_PORT_MIN)
        try:
            while True:
                c = listener.accept()
                th.Thread(target=conHandler, args=(c, listener, g_lock,)).start()
        except KeyboardInterrupt:
            print("EXIT")
        listener.close()

    # Global lock
    g_lock = th.Lock()
    print ("REALSENSE Service started")
    thSenseRead = th.Thread(target=run, args=(g_lock,))
    thSenseRead.setDaemon(True)
    thSenseRead.start()

    thListenerImage = th.Thread(target=listenerImage, args=(g_lock,))
    thListenerImage.setDaemon(True)
    thListenerImage.start()

    thListenerDepth = th.Thread(target=listenerDepth, args=(g_lock,))
    thListenerDepth.setDaemon(True)
    thListenerDepth.start()

    thListenerMin = th.Thread(target=listenerMin, args=(g_lock,))
    thListenerMin.setDaemon(True)
    thListenerMin.start()

    thSenseRead.join()
    #p.join()