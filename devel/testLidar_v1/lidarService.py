import serial
import time
import numpy as np

# Select the correct UART port automatically
TCP_PORT = 6000
COM_PORT = 'COM3'
BAUDRATE = 115200

serial_port = None
is_lidar_running = False
kill_lidar = False

np_distance=np.zeros(360, dtype='int')
np_integrity=np.zeros(360, dtype='int')

def checksum(data):
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


def cleanup():
    """
    Clean up Serial and GPIO port.
    Should be called before closing the program.
    """
    global serial_port
    if serial_port is not None and serial_port.is_open:
        serial_port.close()
    serial_port = None


def init():
    """
    Initialize Serial and GPIO Port.
    """
    global BOARD_NUM, COM_PORT, BAUDRATE, is_lidar_running, kill_lidar, serial_port

    is_lidar_running = False
    kill_lidar = False
    # serial_port = serial.Serial(port=COM_PORT, baudrate=BAUDRATE, timeout=None)


def run(lock):
    """
    Read from LIDAR readings from serial port.
    Stores readings into multiprocessing buffer.
    :param buffer: multiprocessing 1D array of size 720 (360 x 2).
    :param lock: multiprocessing lock.
    :param msg_pipe: pipe to send messages to calling process.
    """
    global is_lidar_running, serial_port

    serial_port = serial.Serial(port='COM3', baudrate=115200, timeout=None)
    speed_rpm = 0
    #buffer=Array[0] * 720
    local_np_distance = np.zeros(360, dtype='int')
    local_np_quality = np.zeros(360, dtype='int')
    global np_distance
    global np_integrity

    # init()

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
            if checksum(actual_checksum) != expected_checksum:
                print ("PACKET ERROR!")
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
                local_np_quality[index*4+i] = quality

            if index == 89:
                # copia el buffer local a dos np Arrays globales

                with lock:
                    np_distance=local_np_distance
                    np_integrity=local_np_quality

    finally:
        cleanup()


if __name__ == "__main__":
    import multiprocessing as mp
    import threading as th
    from multiprocessing.connection import Listener

    address = ('localhost', TCP_PORT)  # family is deduced to be 'AF_INET'
    listener = Listener(address)


    def conHandler(conn, listener, lock):
        print('connection accepted from', listener.last_accepted)
        global np_distance
        while True:
            with g_lock:
                try:
                    conn.send(np_distance)
                except:
                    print ("CANT SEND!")
                    break;
            time.sleep(0.5)
        print ("DISCONNECTED")
        conn.close()
        print("THREAD END")



    # Global lock
    g_lock = th.Lock()
    print ("LIDAR Services started")
    thSerialRead = th.Thread(target=run, args=(g_lock,))
    thSerialRead.setDaemon(True)
    thSerialRead.start()
    print ("Reading LIDAR on COM", COM_PORT)
    print ("Listening TCP on PORT", TCP_PORT)
    try:
        while True:
            c = listener.accept()
            th.Thread(target=conHandler, args=(c, listener,g_lock,)).start()
            """
            time.sleep(2.00)
            with g_lock:
                print(np_distance)
            # print(np_integrity)
            """
    except KeyboardInterrupt:
       print ("EXIT")
    listener.close()
    p.join()