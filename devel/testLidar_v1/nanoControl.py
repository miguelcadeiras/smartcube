from multiprocessing.connection import Client
import sc_services as scsvc
import time
import serial
import threading as th
import numpy as np

LIDAR_PAUSE_DISTANCE=500

realsenseMins=(5000, 5000)
lidarMins=(0,0)



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


arduino_nano = serial.Serial(port='COM7', baudrate=9600)
#time.sleep(1)
#arduino_nano.reset_input_buffer()


def nanoReceive():
    while True:
        data = arduino_nano.readline()
        print('Nano says: ', data)

def nanoWrite (x):
    dataStr = "{" + str(x) + "}" + "\n"
    arduino_nano.write(dataStr.encode('utf-8'))
    print ('To nano:', dataStr)


thNanoReceive = th.Thread(target=nanoReceive)
thNanoReceive.setDaemon(True)
thNanoReceive.start()

realsenseMinClient()
lidarMinClient()

print ("RUNNING")
while True:

    minRs=realsenseMins[0]
    nanoWrite(minRs)
    if len(lidarMins) > 0:
        minL=min(lidarMins)
    else:
        minL=5000

    if minL < LIDAR_PAUSE_DISTANCE:
        print (minL)
        nanoWrite("LD:0")
        print("STOP")

    time.sleep(0.100)


