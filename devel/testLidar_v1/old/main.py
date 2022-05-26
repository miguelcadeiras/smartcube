import numpy as np
import time
import serial
import math

arduino_nano = serial.Serial(port='COM7', baudrate=9600, timeout=.3)
arduino_lidar = serial.Serial(port='COM1', baudrate=115200, timeout=.3)

time.sleep(2);
arduino_nano.reset_input_buffer()
arduino_lidar.reset_input_buffer()

def write_nano(x):
    dataStr = "{"+str(x) + "}"+"\n"
    #arduino.write(bytes(dataStr, 'utf-8'))
    arduino_nano.write(dataStr.encode('utf-8'))

    data = arduino_nano.readline()

    print('Nano says: ', data)

def read_lidar():
    list = []

    arduino_lidar.write ('a'.encode('utf-8'))
    data=arduino_lidar.readline()
    data = data.decode('utf-8' )
    # print(type(data),data)
    list = data

    list=data.split (";")
    # print (list)
    return list

def calc_min(list_angulos,slices):
    # list_angulos = list(map(int, list_angulos))
    arr = np.array(list_angulos[:-1])
    arr = arr.astype(int)
    ##arr = arr.astype(np.float32)
    arr[arr==-1] = 600

    #print(arr)

    list_minimos = []
    for slice in slices:
        desde=slice[0]
        hasta=slice[1]
        m=min(arr[desde:hasta])
        #m = arr[slice].min
        print ( "min: ",slice,m)
        list_minimos.append(m)

    return list_minimos

while True:
    # pedir datos al Lidar
    # calcular minima distancia
    # caso objeto cercano --> mandar alerta al nano

    data_angulo = read_lidar()


    list_angulos = [[0, 90], [270, 359]]
    minimos=calc_min(data_angulo, list_angulos)
    if (min(minimos) < 50):
        write_nano("LD:0")
        print ("STOP")
    else:
        print ("RUN")

    time.sleep(.500)