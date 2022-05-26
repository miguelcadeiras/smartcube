from multiprocessing.connection import Client
import sc_services as scsvc
import time
import serial
import threading as th
import numpy as np

LIDAR_PAUSE_DISTANCE=500

realsenseMins=(5000, 5000)
lidarMins=(0,0)
joyData=[0,0,0,0]


def joystickClient():
    address = ('localhost', scsvc.JOYSTICK_RAW)
    tcp_connected = True

    try:
        conn = Client(address)
    except:
        print("CANT CONNECT JOYSTICK SERVICE", scsvc.JOYSTICK_RAW )
        tcp_connected = False
        exit(-1)

    def readData(conn):
        global joyData
        while True:
            if tcp_connected == True:
                joyData = conn.recv()
                print (joyData)
            else:
                joyData=[0.0, 0.0, 0.0, 0.0]

        conn.close()
    thReadLidarMin = th.Thread(target=readData, args=(conn,))
    thReadLidarMin.setDaemon(True)
    thReadLidarMin.start()

def lidarMinClient():
    address = ('localhost', scsvc.LIDAR_MINS)
    tcp_connected = True

    try:
        conn = Client(address)
    except:
        print("CANT CONNECT LIDAR MIN SERVICE 6001")
        tcp_connected = False
        exit(-1)

    def readLidarMin(conn):
        global lidarMins
        while True:
            if tcp_connected == True:
                localLidarMins = conn.recv()
                lidarMins=localLidarMins
                #print(localLidarMins)
        conn.close()
    thReadLidarMin = th.Thread(target=readLidarMin, args=(conn,))
    thReadLidarMin.setDaemon(True)
    thReadLidarMin.start()



def realsenseMinClient():
    address = ('localhost', scsvc.REALSENSE_MINS)
    tcp_connected = True

    try:
        conn = Client(address)
    except:
        print("CANT CONNECT REALSENSE MIN SERVICE 6022")
        tcp_connected = False
        exit(-1)

    def readRealsenseMin(conn):
        global realsenseMins
        while True:
            if tcp_connected == True:
                localRealsenseMins = conn.recv()
                realsenseMins=localRealsenseMins
                #print(localRealsenseMins)
        conn.close()
    thReadRealsenseMin = th.Thread(target=readRealsenseMin, args=(conn,))
    thReadRealsenseMin.setDaemon(True)
    thReadRealsenseMin.start()

    #thReadRealsenseMin.join()


arduino_nano = serial.Serial(port='COM4', baudrate=115200)
time.sleep(2)
#arduino_nano.reset_input_buffer()


def nanoReceive():
    while True:
        data = arduino_nano.readline()
        print('Nano says: ', data)

def megaWrite (speedr, speedl):
    dataStr = "{spr:" + str(round(speedr, 3)) + ";spl:" + str(round(speedl,3)) + "}" + "\n"
    arduino_nano.write(dataStr.encode('utf-8'))
    print ('To nano:', dataStr)


thNanoReceive = th.Thread(target=nanoReceive)
thNanoReceive.setDaemon(True)
thNanoReceive.start()

#realsenseMinClient()
#lidarMinClient()
joystickClient()

print ("RUNNING")
while True:

    speedRight = joyData[1] * 0.32 * -1
    speedLeft = joyData[1] * 0.32 * -1
    if abs(speedRight) < 0.05:
        speedRight=0
    if abs(speedLeft) < 0.05:
        speedLeft=0

    direction = joyData[2]
    if abs(direction) < 0.07:
        direction = 0
    if direction < 0:
        speedLeft = speedLeft * (1.0-abs(direction))
    if direction > 0:
        speedRight = speedRight * (1.0-abs(direction))

    megaWrite(speedRight, speedLeft)

    time.sleep(0.050)


