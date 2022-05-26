import time
import logdata
import socketprovider
import threading as th
import sc_services

SERVICENAME = "realsenseService"
DEBUG = False

class dataColector:
    data = None
    lock = None
    thDataColector = None
    depth_image = None
    color_image = None
    exit = None

    def runLoop(self, lock):
        import pyrealsense2 as rs
        import numpy as np
        pipeline = rs.pipeline()
        config = rs.config()

        try:
            pipeline_wrapper = rs.pipeline_wrapper(pipeline)
            pipeline_profile = config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            device_product_line = str(device.get_info(rs.camera_info.product_line))
            logdata.log ("RESETING REALSENSE DEVICE")
            device.hardware_reset()
            time.sleep(5)
            found_rgb = False

            for s in device.sensors:
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    found_rgb = True
                    break
            if not found_rgb:
                logdata.log("REALSENSE NOT FOUND")
                exit(0)

            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

            # Start streaming
            pipeline.start(config)


        except:
            logdata.log("ERROR: Cant INIT REALSENSE")
            return None

        try:
            logdata.log("REALSENSE Device started")
            while True:
                while True:
                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()

                    if not depth_frame:
                        continue

                    local_depth_image = np.asanyarray(depth_frame.get_data())
                    local_color_image = np.asanyarray(color_frame.get_data())
                    with lock:
                        self.depth_image = local_depth_image.copy()
                        self.color_image = local_color_image.copy()

                if (self.exit == True):
                    break
        except Exception as e:
            logdata.log(e)
            logdata.log("ERROR: DataCollector Loop")

        logdata.log("Stopping DataCollector")

    def getData(self):
        with self.lock:
            return (self.data)

    def getDataColor(self):
        with self.lock:
            return (self.color_image)

    def getDataDepth(self):
        with self.lock:
            return (self.depth_image)

    def start(self):
        self.exit = False
        logdata.log("Starting DataCollector thread")
        self.lock = th.Lock()
        self.thDataColector = th.Thread(target=self.runLoop, args=(self.lock,))
        self.thDataColector.setDaemon(True)
        self.thDataColector.start()
        self.thDataColector.join(9)

    # time.sleep(4)

    def stop(self):
        self.exit = True


def serviceStart(pipeOut=None, pipeIn=None, setDebug=False):
    DEBUG = setDebug
    logdata.logPipe = pipeOut
    logdata.log("Starting:", SERVICENAME)

    dc = dataColector()
    sp_color = socketprovider.socketProvider()
    sp_depth = socketprovider.socketProvider()
    dc.start()
    sp_color.getData = dc.getDataColor
    sp_depth.getData = dc.getDataDepth

    dcrunning = False
    if dc.thDataColector.is_alive():
        try:
            sp_color.start(sc_services.REALSENSE_COLOR)
            sp_depth.start(sc_services.REALSENSE_DEPTH)
            dcrunning = True
        except KeyboardInterrupt:
            logdata.log("EXIT-2")

    while dcrunning == True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            logdata.log("EXIT-3")
            break
    dc.stop()
    sp_color.stop()
    sp_depth.stop()
    logdata.log("Service Stopping")
    time.sleep(5)
    logdata.log("Service STOPPED")



if __name__ == "__main__":
    print("RUNNING STANDALONE")
    logdata.runningStandalone = True
    serviceStart()
