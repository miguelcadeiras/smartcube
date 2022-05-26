import time
import logdata
import socketprovider
import threading as th
import sc_services

SERVICENAME = "lidarService"
DEBUG = False
COM_PORT = "COM9"

class dataColector:
    data = None
    lock = None
    thDataColector = None

    def checksum(self, data):
        """
        Compute and return the checksum as an int.
        :param data: list of 20 bytes, in the order they arrived in
        :return: The checksum computed.
        """
        # group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append(data[2 * t] + (data[2 * t + 1] << 8))

        # compute the checksum on 32 bits
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d

        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        check_sum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
        check_sum &= 0x7FFF  # truncate to 15 bits
        return int(check_sum)

    def runLoop(self, lock):
        import serial
        import numpy as np
        try:

            serial_port = serial.Serial(port=COM_PORT, baudrate=115200, timeout=None)
        except:
            logdata.log("ERROR: Cant INIT LIDAR")
            return None
        local_np_distance = np.zeros(360, dtype='int')
        local_np_quality = np.zeros(360, dtype='int')
        local_np_rpm = np.zeros(360, dtype='int')
        try:
            while True:
                # Do not hog CPU power
                # time.sleep(0.00001)

                # Packet format:
                # [0xFA, 1-byte index, 2-byte speed, [2-byte flags/distance, 2-byte quality] * 4, 2-byte checksum]
                # All multi-byte values are little endian.
                packet_header = serial_port.read(1)
                if packet_header[0] != 0xFA:
                    continue
                # print (serial_port.in_waiting)
                packet_index = serial_port.read(1)
                if packet_index[0] < 0xA0 or packet_index[0] > 0xF9:
                    continue

                # Packet index | Range = [0,89]
                index = packet_index[0] - 0xA0

                # Read the rest of the packet
                data = serial_port.read(20)

                # Verify the packet's integrity
                expected_checksum = data[19] << 8 | data[18]
                actual_checksum = packet_header + packet_index + data[0:18]
                if self.checksum(actual_checksum) != expected_checksum:
                    logdata.log("PACKET ERROR: ", index)
                    continue

                # Speed in revolutions per minute
                speed_rpm = (data[1] << 8 | data[0]) / 64.0

                # A packet contains 4 distance/reliance readings
                for i in range(4):
                    byte_ndx = 4 * i + 2
                    # The first 16 bits are two flags + distance
                    distance = (data[byte_ndx + 1] << 8) | data[byte_ndx]

                    # The second 16 bits are the reliability (higher # = more reliable reading)
                    quality = (data[byte_ndx + 3] << 8) | data[byte_ndx + 2]
                    if (distance & 0x8000) > 0:
                        distance = -1
                    if (distance & 0x4000) > 0:
                        distance = -1

                    local_np_distance[index * 4 + i] = distance
                    local_np_quality[index * 4 + i] = quality
                    local_np_rpm[index * 4 + i] = speed_rpm

                if index == 89:
                    # copia el buffer local a dos np Arrays globales
                    # print (local_np_rpm[0:60])
                    # print (local_np_distance[0:60])
                    # print (local_np_quality[300:360])
                    with lock:
                        self.data = local_np_distance.copy()
                    if DEBUG:
                        logdata.log (local_np_distance)
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
    dc.start()
    sp.getData = dc.getData
    dcrunning = False
    if dc.thDataColector.is_alive():
        try:
            sp.start(sc_services.LIDAR_RAW)
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
