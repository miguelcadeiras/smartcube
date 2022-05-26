import serial
import time


# Select the correct UART port automatically
COM_PORT = 'COM3'
BAUDRATE = 115200

serial_port = None
is_lidar_running = False
kill_lidar = False


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
    #serial_port = serial.Serial(port=COM_PORT, baudrate=BAUDRATE, timeout=None)


def run(buffer, lock, msg_pipe):
    """
    Read from LIDAR readings from serial port.
    Stores readings into multiprocessing buffer.
    :param buffer: multiprocessing 1D array of size 720 (360 x 2).
    :param lock: multiprocessing lock.
    :param msg_pipe: pipe to send messages to calling process.
    """
    global is_lidar_running, serial_port

    serial_port = serial.Serial(port='COM3', baudrate=115200, timeout=None)

    #init()

    try:
        while True:
            # Do not hog CPU power
            #time.sleep(0.00001)

            # Packet format:
            # [0xFA, 1-byte index, 2-byte speed, [2-byte flags/distance, 2-byte quality] * 4, 2-byte checksum]
            # All multi-byte values are little endian.
            packet_header = serial_port.read(1)
            if packet_header[0] != 0xFA:
                continue

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
                # Checksum error
                with lock:
                    for i in range(4):
                        buffer[8 * index + 2 * i + 0] = 0
                        buffer[8 * index + 2 * i + 1] = -3
                continue

            # Speed in revolutions per minute
            speed_rpm = (data[1] << 8 | data[0]) / 64.0
            print (speed_rpm)
            # A packet contains 4 distance/reliance readings
            for i in range(4):
                byte_ndx = 4 * i + 2
                # The first 16 bits are two flags + distance
                distance = (data[byte_ndx + 1] << 8) | data[byte_ndx]
                
                # The second 16 bits are the reliability (higher # = more reliable reading)
                quality  = (data[byte_ndx + 3] << 8) | data[byte_ndx + 2]
                # Look for "invalid data" flag
                if (distance & 0x8000) > 0:
                    with lock:
                        # byte 0 contains the error code
                        buffer[8 * index + 2 * i + 0] = data[byte_ndx]
                        buffer[8 * index + 2 * i + 1] = -1
                    continue
                # Look for "signal strength warning" flag
                # adding distance might be okay
                elif (distance & 0x4000) > 0:
                    with lock:
                        buffer[8 * index + 2 * i + 0] = distance
                        buffer[8 * index + 2 * i + 1] = -2
                    continue
                else:
                    # Remove flags and write distance/quality to numpy array
                    with lock:
                        buffer[8 * index + 2 * i + 0] = distance & 0x3FFF
                        buffer[8 * index + 2 * i + 1] = quality

            if index == 89:
                msg_pipe.send(0xC0)
    finally:
        cleanup()


if __name__ == "__main__":
    import multiprocessing as mp
    import numpy as np
    import tkinter
    import math

    tkroot = tkinter.Tk()
    myCanvas = tkinter.Canvas(tkroot, bg="black", height=700, width=700)
    myCanvas.pack()
    tkroot.update()
    myCanvas.create_line(10, 100, 350, 350, fill = "white")

    # Multiprocessing array containing the synchronous state
    shared_buffer = mp.Array('i', 720)
    # The Lidar Data
    # index -> angle
    # data -> (distance in milimeter, integrity)
    lidar_data = np.frombuffer(shared_buffer.get_obj(), dtype=int).reshape((360, 2))
    # Global lock
    g_lock = mp.Lock()
    # Multiprocessing communication pipe
    parent_conn, child_conn = mp.Pipe()

    f = None

    #init()
    p = mp.Process(target=run, args=(shared_buffer, g_lock, child_conn,))
    p.start()
    try:
        while True:
            tkroot.update()
            time.sleep(0.010)
            if parent_conn.poll():
                message = parent_conn.recv()
                if message == 0xC0:
                    myCanvas.delete('all')
                    with g_lock:
                        np_distance  = lidar_data[:, 0]
                        np_integrity = lidar_data[:, 1]
                        # Filter out errors
                        np_distance[np_integrity < 1] = 0
                        print(np_distance)
                        #print(np_integrity)
                        scale = 10
                        ang = 0
                        myCanvas.create_oval(350-100, 350-100, 350+100, 350+ 100, outline="green")
                        myCanvas.create_oval(350-200, 350-200, 350+200, 350+200, outline="green")
                        myCanvas.create_oval(350-10, 350-10, 350+10, 350 + 10, outline="green")
                        myCanvas.create_oval(350 - 50, 350 - 50, 350 + 50, 350 + 50, outline="green")
                        myCanvas.create_oval(350 - 25, 350 - 25, 350 + 25, 350 + 25, outline="green")
                        myCanvas.create_rectangle(350-40, 350, 350+40, 350+130, outline="purple")
                        py = math.sin(ang * math.pi / 180) * (np_distance[ang] / scale);
                        px = -math.cos(ang * math.pi / 180) * (np_distance[ang] / scale);
                        for ang in range(1, 360):
                            if (np_distance[ang] > 10):
                                y = math.sin((ang-90) * math.pi / 180) * (np_distance[ang] / scale);
                                x = -math.cos((ang-90) * math.pi / 180) * (np_distance[ang] / scale);
                                if (ang>30 and ang < 330):
                                    myCanvas.create_line(350 - px, 350 + py, 350 - x, 350 + y, fill="red")
                                else:
                                    myCanvas.create_line(350-px, 350+py, 350-x, 350+y, fill="white")
                                px=x
                                py=y



    except KeyboardInterrupt:
        parent_conn.send(LidarParentOps.KILL)


    p.join()