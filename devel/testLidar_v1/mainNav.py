from multiprocessing.connection import Client
import sc_services as scsvc
import time
import serial
import threading as th
import msvcrt
import numpy as np

LIDAR_PAUSE_DISTANCE=500

realsenseMins=(5000, 5000)
lidarMins=(0,0)
joyData=[0,0,0,0]
pixyData={'x0':-1,'y0':-1,'x1':-1,'y1':-1,'v':0, 'i':0}
motionData={'vmot': 0.0, 'vcomp': 0.0, 'accX': 0.0, 'accY': 0.0, 'accZ': 0.0, 'enc': 0.0, 'mstat': 0, 'spl': 0.0, 'spr': 0.0, 'dl': 0.0, 'dr': 0.0}

MOTION_TURN_RIGHT=0
MOTION_TURN_LEFT=1

NAVIGATION_MODE_MANUAL=0
NAVIGATION_MODE_PIXY=1
NAVIGATION_MODE_REALSENSE=2
NAVIGATION_MODE_JOYSTICK=2

navigationMode=NAVIGATION_MODE_MANUAL

def pixyClient():
    address = ('localhost', scsvc.PIXY_RAW)
    tcp_connected = True

    try:
        conn = Client(address)
    except:
        print("CANT CONNECT PIXY SERVICE", scsvc.PIXY_RAW )
        tcp_connected = False
        exit(-1)

    def readData(conn):
        global pixyData
        while True:
            if tcp_connected == True:
                pixyData = conn.recv()
            else:
                pixyData={'x0':-1,'y0':-1,'x1':-1,'y1':-1,'v':0, 'i':0}
        conn.close()
    thReadLidarMin = th.Thread(target=readData, args=(conn,))
    thReadLidarMin.setDaemon(True)
    thReadLidarMin.start()

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


arduino_mega = serial.Serial(port='COM4', baudrate=115200)
time.sleep(2)
#arduino_mega.reset_input_buffer()


def megaReceive():
    import json
    global motionData
    while True:
        data = arduino_mega.readline()
        #print (data)
        #print (data[0])
        jsonData={}
        try:
            jsonData = json.loads(data.decode())
            motionData.update(jsonData)
        except Exception as e:
            print ("----")
            print ("MEGA RECEIVE: ")
            print (data)
            print ("ERROR:" + str(e))


        #print (motionData)
        #print('Nano says: ', data)

def megaWrite (speedr, speedl):
    dataStr = "{spr:" + str(round(speedr, 3)) + ";spl:" + str(round(speedl,3)) + "}" + "\n"
    arduino_mega.write(dataStr.encode('utf-8'))
    #print ('TO MEGA:', dataStr)


thMegaReceive = th.Thread(target=megaReceive)
thMegaReceive.setDaemon(True)
thMegaReceive.start()

#realsenseMinClient()
#lidarMinClient()
#joystickClient()
pixyClient()

print ("RUNNING")

def joyDrive():
    speedRight = joyData[1] * 0.32 * -1
    speedLeft = joyData[1] * 0.32 * -1
    if abs(speedRight) < 0.05:
        speedRight = 0
    if abs(speedLeft) < 0.05:
        speedLeft = 0

    direction = joyData[2]
    if abs(direction) < 0.07:
        direction = 0
    if direction < 0:
        speedLeft = speedLeft * (1.0 - abs(direction))
    if direction > 0:
        speedRight = speedRight * (1.0 - abs(direction))
    return speedRight, speedLeft

def millis():
    return time.time()*1000

class ManualDrive:
    speedLeft=0.0
    speedRight=0.0
    maxSpeed=0.32

    def stop(self):
        self.speedLeft=0.0
        self.speedRight=0.0

    def fastForward(self):
        self.speedLeft = self.maxSpeed
        self.speedRight = self.maxSpeed

    def forward(self):
        self.speedLeft = self.maxSpeed/2
        self.speedRight = self.maxSpeed/2

    def backwards(self):
        self.speedLeft = -self.maxSpeed / 2
        self.speedRight = -self.maxSpeed / 2

    def turnLeft90(self):
        self.speedLeft = 0.04
        self.speedRight = self.maxSpeed / 2

    def turnRight90(self):
        self.speedLeft = self.maxSpeed / 2
        self.speedRight = 0.00

    def turnLeft90_inPlace(self):
        self.speedLeft = 0.0
        self.speedRight =  self.maxSpeed / 2

    def turnRight90_inPlace(self):
        self.speedLeft = 0.0
        self.speedRight = self.maxSpeed / 2

    def aTurn(self, dir):
        if dir==MOTION_TURN_LEFT:
            self.speedLeft = self.maxSpeed * 0.95
            self.speedRight = self.maxSpeed * 1.1
        if dir==MOTION_TURN_RIGHT:
            self.speedLeft=self.maxSpeed * 0.95
            self.speedRight=self.maxSpeed * 1.1

    def bTurn(self, dir):
        if dir == MOTION_TURN_LEFT:
            self.speedLeft = self.maxSpeed * 0.85
            self.speedRight = self.maxSpeed * 1.00
        if dir == MOTION_TURN_RIGHT:
            self.speedLeft = self.maxSpeed * 1.00
            self.speedRight = self.maxSpeed * 0.85

    def cTurn(self, dir):
        if dir==MOTION_TURN_LEFT:
            self.speedLeft = self.maxSpeed * 0.7
            self.speedRight = self.maxSpeed * 1.05
        if dir==MOTION_TURN_RIGHT:
            self.speedLeft=self.maxSpeed * 1.05
            self.speedRight=self.maxSpeed * 0.7

class EncoderData:
    distance=0
    lastDistance=0
    lastArduinoEncDistance=0
    lastEncoderUpdate=0

    def resetDistance(self):
        self.distance=0
        self.lastDistance=0

    def update(self):

        arduinoEncDistance=motionData['enc']

        if (arduinoEncDistance > self.lastArduinoEncDistance):
            lastEncoderUpdate=millis()
            self.distance += arduinoEncDistance - self.lastArduinoEncDistance
            self.lastArduinoEncDistance = arduinoEncDistance

            if (self.distance - self.lastDistance >= 0.05):
                self.lastDistance=self.distance
                #str(self.distance)
                #newest_method_string = f"{numvar:.9f}"
                vser.Serial.println("{mts;" + f"{self.distance: .2f}" + ";}")

class RealsenseDrive:
    error=0
    errorPrev=0
    integral=0
    pid=0
    pGain = 2.0
    iGain = 0.1
    dGain = 0.4
    realSenseDetectedTimer=0
    speedRight=0.0
    speedLeft=0.0
    def update(self, realsenseData):
        err=0.0  # ACA VA EL DATO QUE VIENE DE REALSENSE
        self.errorPrev = self.error
        self.error = err / 10;

        if (abs(self.error) > 25):
            self.realSenseDetectedTimer = millis();

        if (abs(self.error) > 60):
            self.barDetected = false

        # INTEGRADOR
        self.integral += self.error

        if (self.integral > 100):
            self.integral = 100
        if (self.integral < -100):
            self.integral = -100

        p = (self.error * self.pGain)
        i = (self.integral * self.iGain)
        d = (self.error - self.errorPrev) * self.dGain
        self.pid = p + i + d

        # VELOCIDAD DEFAULT
        self.speedLeft = self.MAX_SPEED
        self.speedRight = self.MAX_SPEED

        # SI EL ERROR ES MAYOR A 10 VEL=50%
        if (abs(self.error) > 10):
            self.speedLeft = 50
            self.speedRight = 50

        if (self.pid > 0):
            self.speedRight = self.speedRight - abs(self.pid);
        elif (self.pid < 0):
            self.speedLeft = self.speedLeft - abs(self.pid);

        if (self.speedLeft < 0):
            self.speedLeft = 0.0
        if (self.speedRight < 0):
            self.speedRight = 0.0

        return self.speedRight, self.speedLeft

class pixyDrive:
    lastVectorTime=0
    errorPrev=0
    error=0
    integral=0
    pid=0
    pGain = 2.5 #PID_P_GAIN; 2.5
    iGain = 0.1 #PID_I_GAIN; 0.1
    dGain = 0.7 #PID_D_GAIN; 0.7

    MAX_SPEED=100 #0.32
    LOW_SPEED=20 #ORIGINAL 40 #0.16

    speedLeft=0
    speedRight=0

    noLineTimeout=4000
    noVector=False

    maxSpeedTimer=0

    def update(self, pixyData):
        pixyWidth=79
        pixyHeight=52

        if pixyData['v'] > 0:
            self.speedLeft = 0
            self.speedRight = 0

            self.noVector=False
            self.lastVectorTime=millis()
            self.errorPrev=self.error

            # POR SI EL VECTOR ESTA INVERTIDO
            vectX = pixyData['x1']
            if pixyData['y1'] > pixyData['y0']:
                vectX = pixyData['x0']

            self.error = vectX - (pixyWidth / 2) - 1

            # INTEGRADOR
            self.integral += self.error

            if (self.integral > 100):
                self.integral = 100
            if (self.integral < -100):
                self.integral = -100

            p = (self.error * self.pGain)
            i = (self.integral * self.iGain)
            d = (self.error - self.errorPrev) * self.dGain
            self.pid = p + i + d

            #print (self.pid)

            # DESDE ACA -------------------

            # VELOCIDAD DEFAULT 100 %
            #if (millis() - self.maxSpeedTimer > 500):
            if (millis() - self.maxSpeedTimer > 300):
                self.speedLeft=self.MAX_SPEED
                self.speedRight=self.MAX_SPEED
            else:
                self.speedLeft=self.LOW_SPEED
                self.speedRight=self.LOW_SPEED


            # SI EL VECTOR ES MUY CHICO NO CAMBIA NADA
            if (abs(pixyData['y1'] - pixyData['y0']) < 30):
                #noAction=true
                self.maxSpeedTimer=millis()


            # SI EL ERROR ES MAYOR A 20 -> VEL = 50%
            if (abs(self.error) > 20):
                self.speedLeft=self.LOW_SPEED
                self.speedRight=self.LOW_SPEED
                self.maxSpeedTimer=millis()


            #SI EL VECTOR ES CHICO (MENOR A LA MITAD) VEL=30%
            maxPixyVectorSize = pixyHeight
            if (abs(pixyData['y1'] - pixyData['y0']) < 30 < (maxPixyVectorSize / 2) + 10):
                self.speedLeft=self.LOW_SPEED
                self.speedRight=self.LOW_SPEED
                self.maxSpeedTimer=millis()


            if (abs(pixyData['x1'] - pixyData['x0']) > 40):
                self.speedLeft=self.LOW_SPEED
                self.speedRight=self.LOW_SPEED
                self.maxSpeedTimer=millis()

            if (self.pid > 0):
                self.speedRight = self.speedRight - abs(self.pid);
            elif (self.pid < 0):
                self.speedLeft = self.speedLeft - abs(self.pid);

            if (self.speedLeft < 0):
                self.speedLeft = 0.0
            if (self.speedRight < 0):
                self.speedRight = 0.0
        else:
            if millis()-self.lastVectorTime > self.noLineTimeout:
                self.noVector = True
                self.speedLeft = 0.0
                self.speedRight = 0.0

        return self.speedRight, self.speedLeft

class ArduSerial(serial.Serial):
    comport=None
    ser=None

    def __init__(self, comport):
        self.comport=comport

    def begin(self, baudrate):
        serial.Serial.__init__ (self, port=self.comport, baudrate=baudrate)

    def print (self, str):
        self.write(str.encode('utf-8'))

    def println(self, str):
        str+='\r\n'
        self.write(str.encode('utf-8'))

    def available(self):
        return self.in_waiting


class VirtualSerialHandler:
    Serial=None

    def printHelp(self):
        self.Serial.print("SmartMainNav v.")
        self.Serial.println("0.1")
        self.Serial.println("w,s,a d  | FORWARD,BACKWARDS, LEFT,RIGHT")
        self.Serial.println("z,c      | TURN IN PLACE LEFT / RIGHT")
        self.Serial.println("f        | FAST FORWARD")
        self.Serial.println("0        | STOP")
        self.Serial.println("q        | STOP AUTO DRIVE")
        self.Serial.println("g        | EMERGENCY STOP RESET")
        self.Serial.println("p        | FOLLOW LINE")
        self.Serial.println("r        | REALSENSE")

        self.Serial.println("t        | Z AXIS INFO")
        self.Serial.println("u        | Z AXIS INFO")
        self.Serial.println("i,o      | PRINT STATUS INFO, PRINT SPEED ")

        self.Serial.println("m,n      | INCREASE,DECREASE Default Speed")
        self.Serial.println("L,R,D    | NEXT INTERSECT: LEFT,RIGHT")
        self.Serial.println("1,2,3    | RESET: DefaultSpeed, Center Drive, Encoder")
        self.Serial.println("4,5,6    | US SENSOR TOGGLE, NO ENCODER TOGGLE, TILT TOGGLE")

        self.Serial.println("e        | RETURNS ENCODER POSITION SINCE RESET")
        self.Serial.println("E        | RETURNS MOTORS POSITION SINCE RESET")
        self.Serial.println("v,b      | VERVOSE MODE,VERBOSE DRIVE MODE ")
        self.Serial.println("x        | READ BATTERIES VOLTAG (24v & 36v)")

        self.Serial.println("7,8,9  |aTurn, bturn,cTurn right")
        self.Serial.println("/,*,-  |aTurn, bturn,cTurn left")
        self.Serial.println("?        | Help")

    def start(self):
        self.Serial=ArduSerial('COM11')
        self.Serial.begin(115200)
        self.Serial.println("separete_drive_5 ")
        self.printHelp()

    def update(self):
        global navigationMode

        if self.Serial.available():
            inCh=self.Serial.read()
            ch=inCh.decode()

            if ch=='?':
                self.printHelp()
            elif ch=='v': #verbose
                self.Serial.print('v')
            elif ch=='b': #verbose drive
                self.Serial.print('b')
            elif ch=='f': #move forward fast
                self.Serial.println("moving FastForward")
                manualdrive.fastForward()
            elif ch=='w': #move forward
                self.Serial.println("moving Forward")
                manualdrive.forward()
            elif ch=='s': # move backwards
                self.Serial.println("moving Backwards")
                manualdrive.backwards()
            elif ch=='a': # turnleft90
                self.Serial.println("Turning Left")
                manualdrive.turnLeft90()
            elif ch=='d': # turnoright90
                self.Serial.println("Turning Right")
                manualdrive.turnRight90()
            elif ch=='z': # turnleft90_inplace
                self.Serial.print("z")
                manualdrive.turnLeft90_inPlace()
            elif ch=='c': # turnright90_inplace
                self.Serial.print("z")
                manualdrive.turnRight90_inPlace()
            elif ch=='0' or ch=='q': # STOP
                self.Serial.println("STOP")
                navigationMode = NAVIGATION_MODE_MANUAL
                manualdrive.stop()
            elif ch=='g': # emergency reset
                self.Serial.print("g")
            elif ch=='p': # pixy drive
                self.Serial.println("Pixy Drive Activated...")
                navigationMode = NAVIGATION_MODE_PIXY
            elif ch=='r': # realsense drive
                self.Serial.println("Realsense Drive Activated...")
                navigationMode = NAVIGATION_MODE_REALSENSE
            elif ch == 'j':  # joystick drive
                self.Serial.println("Realsense Drive Activated...")
                navigationMode = NAVIGATION_MODE_JOYSTICK

            elif ch=='K': # KART TEST
                self.Serial.println("KART CONNECTION SUCCESS")
            elif ch=='e': # encoder getMts
                self.Serial.println(str(motionData['enc']))
                self.Serial.println(str(encData.distance))

            elif ch=='x':
                self.Serial.println("{vmot;"+str(motionData['vmot'])+";}")
                self.Serial.println("{vcomp;" + str(motionData['vcomp']) + ";}")
            elif ch=='3':
                self.Serial.println("ENCODER RESET");
                encData.resetDistance()

            elif ch=='4':
                pixydrive.pGain-=0.1
                self.Serial.println("PGAIN:"+ str(pixydrive.pGain))
            elif ch=='5':
                pixydrive.pGain += 0.1
                self.Serial.println("PGAIN:"+ str(pixydrive.pGain))
            elif ch=='6':
                pixydrive.iGain-=0.1
                self.Serial.println("IGAIN:"+ str(pixydrive.iGain))
            elif ch=='7':
                pixydrive.iGain += 0.1
                self.Serial.println("IGAIN:"+ str(pixydrive.iGain))
            elif ch=='8':
                pixydrive.dGain-=0.1
                self.Serial.println("DGAIN:"+ str(pixydrive.dGain))
            elif ch=='9':
                pixydrive.dGain += 0.1
                self.Serial.println("DGAIN:"+ str(pixydrive.dGain))
            elif ch == '.':
                self.Serial.println("-------------------------------")
                self.Serial.println("PGAIN:"+ str(pixydrive.pGain))
                self.Serial.println("IGAIN:"+ str(pixydrive.iGain))
                self.Serial.println("DGAIN:"+ str(pixydrive.dGain))

pixydrive = pixyDrive()
manualdrive = ManualDrive()

print ("Waiting 5 seconds")
time.sleep(5)
print ("READY to Drive!")

vser=VirtualSerialHandler()
vser.start()

encData=EncoderData()

spMax=0.32
spRight=0.0
spLeft=0.0

while True:
    vser.update()
    encData.update()



    if navigationMode==NAVIGATION_MODE_MANUAL:
        spLeft=manualdrive.speedLeft
        spRight=manualdrive.speedRight
        megaWrite(spRight, spLeft)
    elif navigationMode==NAVIGATION_MODE_PIXY:
        manualdrive.stop()
        pixy_spRight, pixy_spLeft=pixydrive.update(pixyData)
        spRight=(spMax * pixy_spRight) / 100
        spLeft=(spMax * pixy_spLeft) / 100

        megaWrite(spRight, spLeft)
        if pixydrive.noVector==True:
            print ("PIXY NO LINE!")
            navigationMode=NAVIGATION_MODE_MANUAL
            manualdrive.stop()
    elif navigationMode==NAVIGATION_MODE_REALSENSE:
        manualdrive.stop()


    time.sleep(0.025)


