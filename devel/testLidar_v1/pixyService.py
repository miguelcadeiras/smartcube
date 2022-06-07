import time
import logdata
import socketprovider
import threading as th
import sc_services

SERVICENAME = "pixyService"
DEBUG = False
COM_PORT = "COM7"

class dataColector:
    data = None
    lock = None
    thDataColector = None


    def runLoop(self, lock):
        import serial
        import json
        import numpy as np
        try:

            serial_port = serial.Serial(port=COM_PORT, baudrate=115200, timeout=None)
        except:
            logdata.log("ERROR: Cant INIT LIDAR")
            return None
        local_vector=None

        try:
            while True:
                serialLine = serial_port.readline()
                # logdata.log (serialLine)
                try:
                    local_vector=json.loads(serialLine)
                except:
                    local_vector={'x0': -1, 'y0': -1, 'x1': -1, 'y1': -1, 'v': 0, 'i': 0}

                if DEBUG:
                    logdata.log (local_vector)

                with lock:
                    self.data = local_vector
                if DEBUG:
                    logdata.log (serialLine)
                if (self.exit == True):
                    break
        except Exception as e:
            logdata.log(e)
            logdata.log("ERROR: DataCollector Loop")

        logdata.log("Stopping DataCollector")

    def getData(self):
        with self.lock:
            return (self.data)

    def start(self):
        self.exit = False
        logdata.log("Starting DataCollector thread")
        self.lock = th.Lock()
        self.thDataColector = th.Thread(target=self.runLoop, args=(self.lock,))
        self.thDataColector.setDaemon(True)
        self.thDataColector.start()
        self.thDataColector.join(4)

    # time.sleep(4)

    def stop(self):
        self.exit = True


def serviceStart(pipeOut=None, pipeIn=None, setDebug=False):
    DEBUG = setDebug
    logdata.logPipe = pipeOut
    logdata.log("Starting:", SERVICENAME)

    dc = dataColector()
    sp = socketprovider.socketProvider()
    sp.sendInterval=0.025
    dc.start()
    sp.getData = dc.getData
    dcrunning = False
    if dc.thDataColector.is_alive():
        try:
            sp.start(sc_services.PIXY_RAW)
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
    sp.stop()
    logdata.log("Service Stopping")
    time.sleep(5)
    logdata.log("Service STOPPED")



if __name__ == "__main__":
    print("RUNNING STANDALONE")
    logdata.runningStandalone = True
    serviceStart()
