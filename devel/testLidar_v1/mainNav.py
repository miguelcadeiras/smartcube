from multiprocessing.connection import Client
import sc_services as scsvc
import time
import serial
import threading as th
import json
import msvcrt
import numpy as np

LIDAR_PAUSE_DISTANCE=500

realsenseMins=(5000, 5000)

lidarMins=(1000,1000)
joyData=[0,0,0,0]
pixyData={'x0':-1,'y0':-1,'x1':-1,'y1':-1,'v':0, 'i':0}
motionData={'vmot': 0.0, 'vcomp': 0.0, 'accX': 0.0, 'accY': 0.0, 'accZ': 0.0, 'enc': 0.0, 'mstat': 0, 'spl': 0.0, 'spr': 0.0, 'dl': 0.0, 'dr': 0.0}

MOTION_TURN_RIGHT=0
MOTION_TURN_LEFT=1

NAVIGATION_MODE_MANUAL=0
NAVIGATION_MODE_PIXY=1
NAVIGATION_MODE_REALSENSE=2
NAVIGATION_MODE_JOYSTICK=3
NAVIGATION_MODE_CMD=4

navigationModeNames = ("MANUAL", "PIXY", "REALSENSE", "JOYSTICK", "COMMAND")

navigationMode=NAVIGATION_MODE_MANUAL


class ServiceClient:
    def connect(self):
        try:
            address = (self.serveraddress, self.port)
            self.conn = Client(address)
            self.available=True
        except:
            print("CANT CONNECT PORT", self.port, "ON", self.serveraddress)
            self.available=False

    def dataUpdate(self):
        raise NotImplementedError

    def checkDataUpdate(self):
        t=self.lastDataUpdateTime
        if t==-1:
            self.running=False
            return 1

        if (time.time() - t) > self.dataUpdateTimeout:
            self.running = False
            return 1

        self.running=True
        return 0

    def connWorker(self):
        while True:
            if self.available == True:
                try:
                    self.dataUpdate()
                except:
                    break
            else:
                break
        self.available = False

    def sendData(self, data):
        self.conn.send(data)

    def __init__(self, serveraddress, port):
        self.connThread=None
        self.conn=None
        self.available=False
        self.serveraddress=serveraddress
        self.port=port
        self.lastDataUpdateTime=-1
        self.dataUpdateTimeout = 1
        self.running=False

        self.connect()
        self.connThread=th.Thread(target=self.connWorker)
        self.connThread.setDaemon(True)
        self.connThread.start()

class RealsenseMinsClient(ServiceClient):
    def dataUpdate(self):
        global realsenseMins
        localRealsenseMins = self.conn.recv()
        realsenseMins = localRealsenseMins
        self.lastDataUpdateTime = time.time()


class LidarMinClient(ServiceClient):
    def dataUpdate(self):
        global lidarMins
        localLidarMins = self.conn.recv()
        lidarMins = localLidarMins
        self.lastDataUpdateTime = time.time()


class JoystickClient(ServiceClient):
    def dataUpdate(self):
        global joyData
        joyData = self.conn.recv()
        self.lastDataUpdateTime = time.time()

class PixyClient(ServiceClient):
    def dataUpdate(self):
        global pixyData
        pixyData = self.conn.recv()
        self.lastDataUpdateTime = time.time()


print ("STARTING MAINNAV")
print ("")

print ("CONNECTING ARDUINO MEGA ON COM4  ", sep='')
arduino_mega = serial.Serial(port='COM4', baudrate=115200)
time.sleep(2)
print ("[OK]")

def megaReceive():
    import json
    global motionData
    while True:
        data = arduino_mega.readline()
        jsonData={}
        try:
            jsonData = json.loads(data.decode(errors="ignore"))
            motionData.update(jsonData)
        except Exception as e:
            print ("MEGA RECEIVE: ", data)
            #print ("ERROR:" + str(e))


        #print (motionData)
        #print('Nano says: ', data)

def megaWrite (speedr, speedl):
    dataStr = "{spr:" + str(round(speedr, 3)) + ";spl:" + str(round(speedl,3)) + "}" + "\n"
    arduino_mega.write(dataStr.encode('utf-8'))
    #print ('TO MEGA:', dataStr)

def megaBeep (time=100, rep=1):
    dataStr="{\"beep\":["+str(time)+","+str(rep)+"]}\n"
    arduino_mega.write(dataStr.encode('utf-8'))


thMegaReceive = th.Thread(target=megaReceive)
thMegaReceive.setDaemon(True)
thMegaReceive.start()

print("")
print ("STARTING CLIENTS")
rsMinsClient=RealsenseMinsClient(scsvc.REALSENSE_MINS_SERVER, scsvc.REALSENSE_MINS)
print("REALSENSE MINS:", rsMinsClient.available)
lidarMinClient=LidarMinClient(scsvc.LIDAR_MINS_SERVER, scsvc.LIDAR_MINS)
print("LIDAR MINS:", lidarMinClient.available)
joystickClient=JoystickClient(scsvc.JOYSTICK_RAW_SERVER, scsvc.JOYSTICK_RAW)
print("JOYSTICK:", joystickClient.available)
pixyClient=PixyClient(scsvc.PIXY_RAW_SERVER, scsvc.PIXY_RAW)
print("PIXY:", pixyClient.available)
print ("CLIENTS STARTED")



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
    lastEncoderIncreaseTime=0
    speed=0
    speedLastDistance=0
    speedTimer=0
    def calcSpeed(self):
        if millis()-self.speedTimer > 1000:
            self.speed=(self.lastArduinoEncDistance-self.speedLastDistance) / 1
            self.speedLastDistance=self.lastArduinoEncDistance
            self.speedTimer=millis()



    def resetTimer(self):
        self.lastEncoderIncreaseTime = millis()
    def getTimer(self):
        return millis()-self.lastEncoderIncreaseTime
    def resetDistance(self):
        self.distance=0
        self.lastDistance=0

    def update(self):

        arduinoEncDistance=motionData['enc']

        if (arduinoEncDistance > self.lastArduinoEncDistance):
            self.resetTimer()
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
    pGain = 2.0 # ORIGINAL 2.0
    iGain = 0.01 # ORIGINAL 0.1
    dGain = 1.0 # ORIGINAL 0.4
    realSenseDetectedTimer=0
    speedRight=0.0
    speedLeft=0.0
    MAX_SPEED = 100
    LOW_SPEED = 85
    barDetected=False
    def update(self, rsData):
        err = rsData[0]  # ACA VA EL DATO QUE VIENE DE REALSENSE
        self.errorPrev = self.error
        self.error = err / 10  # err /10

        #self.barDetected = True

        if abs(self.error) > 25:
            self.barDetected = True
            self.realSenseDetectedTimer = millis()

        if millis()-self.realSenseDetectedTimer > 250:
            self.barDetected = False

        if abs(self.error) <= 25:
            self.barDetected = True
            self.realSenseDetectedTimer = millis()

        if abs(self.error) > 60:
            self.barDetected = False

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
        #print ("E P I D PID", err, p, i, d , self.pid)
        # VELOCIDAD DEFAULT
        self.speedLeft = self.MAX_SPEED
        self.speedRight = self.MAX_SPEED

        # SI EL ERROR ES MAYOR A 10 VEL=50%
        if (abs(self.error) > 10):
            self.speedLeft = self.LOW_SPEED
            self.speedRight = self.LOW_SPEED

        if (self.pid > 0):
            self.speedRight = self.speedRight - abs(self.pid);
        elif (self.pid < 0):
            self.speedLeft = self.speedLeft - abs(self.pid);

        if (self.speedLeft < 0):
            self.speedLeft = 0.0
        if (self.speedRight < 0):
            self.speedRight = 0.0

        return self.speedRight, self.speedLeft

class JoyDrive:
    speedLeft=0
    speedRight=0
    maxSpeed=0.32

    def update(self, joyData):
        self.speedRight = joyData[1] * 0.32 * -1
        self.speedLeft = joyData[1] * 0.32 * -1
        if abs(self.speedRight) < 0.05:
            self.speedRight = 0
        if abs(self.speedLeft) < 0.05:
            self.speedLeft = 0

        direction = joyData[2]
        if abs(direction) < 0.07:
            direction = 0
        if direction < 0:
            self.speedLeft = self.speedLeft * (1.0 - abs(direction))
        if direction > 0:
            self.speedRight = self.speedRight * (1.0 - abs(direction))

        return self.speedRight, self.speedLeft

class PixyDrive:
    lastVectorTime=0
    errorPrev=0
    error=0
    integral=0
    pid=0
    pGain = 2.0 #PID_P_GAIN; 2.5
    iGain = 0.1 #PID_I_GAIN; 0.1
    dGain = 0.4 #PID_D_GAIN; 0.7

    MAX_SPEED=100 #0.32
    LOW_SPEED=40 #ORIGINAL 40 #0.16

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
                self.speedLeft = 0
            if (self.speedRight < 0):
                self.speedRight = 0
        else:
            if millis()-self.lastVectorTime > self.noLineTimeout:
                self.noVector = True
                self.speedLeft = 0
                self.speedRight = 0

        return self.speedRight, self.speedLeft


class PixyDriveTEST:
    lastVectorTime=0
    errorPrev=0
    error=0
    integral=0
    pid=0
    pGain = 2.0 #PID_P_GAIN; 2.5
    iGain = 0.1 #PID_I_GAIN; 0.1
    dGain = 0.4 #PID_D_GAIN; 0.7

    MAX_SPEED=120 #0.32
    MIN_SPEED=0
    LOW_SPEED=40 #ORIGINAL 40 #0.16

    speedLeft=0
    speedRight=0

    noLineTimeout=2000
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
            #print ("PID:", self.pid)
            self.speedRight = 100 - self.pid
            self.speedLeft = 100 + self.pid

            #if (self.pid > 0):
            #    self.speedRight = self.speedRight - abs(self.pid);
            #    self.speedLeft
            #elif (self.pid < 0):
            #    self.speedLeft = self.speedLeft - abs(self.pid);

            if (self.speedLeft > self.MAX_SPEED):
                self.speedLeft=self.MAX_SPEED
            if (self.speedRight > self.MAX_SPEED):
                self.speedRight=self.MAX_SPEED
            if (self.speedLeft < 0):
                self.speedLeft = 0
            if (self.speedRight < 0):
                self.speedRight = 0
        else:
            if millis()-self.lastVectorTime > self.noLineTimeout:
                self.noVector = True
                self.speedLeft = 0
                self.speedRight = 0
        #print ("R-L",self.speedRight, self.speedLeft)
        return self.speedRight, self.speedLeft


class ArduSerial(serial.Serial):
    comport=None
    ser=None

    def __init__(self, comport):
        self.comport=comport

    def begin(self, baudrate):
        serial.Serial.__init__ (self, port=self.comport, baudrate=baudrate, rtscts=False, dsrdtr=False)

    def print (self, str):
        self.write(str.encode('utf-8'))

    def println(self, str):
        str+='\r\n'
        self.write(str.encode('utf-8'))
        #print ("TO VIRT SERIAL:", str, end="")

    def available(self):
        return self.in_waiting

class MainNavStatus:
    statusData={}
    servicesStatus={}
    def __init__(self):
        pass
    def updateData(self):
        self.servicesStatus.update({"lidarMins": lidarMinClient.running})
        self.servicesStatus.update({"realsenseMins": rsMinsClient.running})
        self.servicesStatus.update({"pixy": pixyClient.running})
        self.servicesStatus.update({"joystick": joystickClient.running})

        self.statusData.update({"emergencyStop": motion.emergencyStop})
        self.statusData.update({"services": self.servicesStatus})
        self.statusData.update({"lidarEnabled": lidarEnabled})
        self.statusData.update({"lidarPause": lidarPause})
        self.statusData.update({"navigationMode": navigationModeNames[navigationMode]})
        self.statusData.update({"setSpeed": motion.spMax})
        self.statusData.update({"encoderSpeed": encData.speed})
        self.statusData.update({"motionData": motionData})

    def getData(self):
        jsonData=json.dumps(self.statusData, indent=None)
        return jsonData


class VirtualSerialHandler:
    Serial=None
    readingLine=False
    linebuffer=bytearray()

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

        self.Serial.println("h        | MAIN NAV STATUS")
        self.Serial.println("t        | Z AXIS INFO")
        self.Serial.println("u        | Z AXIS INFO") # este estaria duplicado?? o es distinta la info?
        self.Serial.println("i,o      | PRINT STATUS INFO, PRINT SPEED ")

        self.Serial.println("m,n      | INCREASE,DECREASE Default Speed")
        self.Serial.println("L,R,D    | NEXT INTERSECT: LEFT,RIGHT")
        self.Serial.println("1,2,3    | RESET: DefaultSpeed, Center Drive, Encoder")
        self.Serial.println("4,5,6    | US SENSOR TOGGLE, NO ENCODER TOGGLE, TILT TOGGLE")

        self.Serial.println("e        | RETURNS ENCODER POSITION SINCE RESET")
        self.Serial.println("E        | RETURNS MOTORS POSITION SINCE RESET")
        self.Serial.println("v,b      | VERBOSE MODE,VERBOSE DRIVE MODE ")
        self.Serial.println("x        | READ BATTERIES VOLTAG (24v & 36v)")

        self.Serial.println("7,8,9  |aTurn, bturn,cTurn right")
        self.Serial.println("/,*,-  |aTurn, bturn,cTurn left")
        self.Serial.println("?        | Help")

    def start(self):
        self.Serial=ArduSerial('COM11')
        self.Serial.begin(115200)
        self.Serial.println("separete_drive_5")
        self.printHelp()

    def update(self):
        global navigationMode

        if not self.readingLine:
            if self.Serial.available():
                inCh=self.Serial.read()
                ch=inCh.decode(errors="ignore")

                if ch=='?':
                    self.printHelp()
                elif ch=='v': #verbose
                    self.Serial.print('v')
                elif ch=='b': #verbose drive
                    self.Serial.print('b')
                elif ch=='f': #move forward fast
                    self.Serial.println("moving FastForward")
                    print("F - MOVING FAST FORWARD")
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.fastForward()
                elif ch=='w': #move forward
                    self.Serial.println("moving Forward")
                    print("W - MOVING FORWARD")
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.forward()
                elif ch=='s': # move backwards
                    self.Serial.println("moving Backwards")
                    print('S - MOVING BACKWARDS')
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.backwards()
                elif ch=='a': # turnleft90
                    self.Serial.println("Turning Left")
                    print('A - Turning Left')
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.turnLeft90()
                elif ch=='d': # turnoright90

                    self.Serial.println("Turning Right")
                    print('D - Turning Right')
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.turnRight90()
                elif ch=='z': # turnleft90_inplace
                    self.Serial.print("z")
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.turnLeft90_inPlace()
                elif ch=='c': # turnright90_inplace
                    self.Serial.print("z")
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.turnRight90_inPlace()
                elif ch=='t' or ch=='u': # Z AxisInfo
                    self.Serial.println(str(motionData['accZ']))
                elif ch=='0' or ch=='q': # STOP
                    self.Serial.println("STOP")
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.stop()
                elif ch=='g': # emergency reset
                    self.Serial.println("EMERGENCY RESET")
                    navigationMode = NAVIGATION_MODE_MANUAL
                    manualdrive.stop()
                    motion.emergencyStop=False
                elif ch=='h': # MAIN NAV STATUS
                    self.Serial.println(mainNavStatus.getData())
                elif ch=='p': # pixy drive
                    self.Serial.println("Pixy Drive Activated...")
                    navigationMode = NAVIGATION_MODE_PIXY
                elif ch == 'r': # realsense drive
                    self.Serial.println("Realsense Drive Activated...")
                    navigationMode = NAVIGATION_MODE_REALSENSE
                elif ch == 'j':  # joystick drive
                    self.Serial.println("Joystick Drive Activated...")
                    navigationMode = NAVIGATION_MODE_JOYSTICK
                elif ch == '4':  # lidar toggle
                    global lidarEnabled
                    lidarEnabled=not(lidarEnabled)
                    print ("Lidar Enabled: " + str(lidarEnabled))
                    self.Serial.println("Lidar Enabled: " + str(lidarEnabled))

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
                elif ch == '{':
                    self.linebuffer = bytearray()
                    self.linebuffer.append(inCh[0])
                    self.readingLine = True


        else:
            if self.Serial.available():
                inCh = self.Serial.read()
                ch = inCh.decode(errors="ignore")

                self.linebuffer.append(inCh[0])
                if ch == '}':
                    self.readingLine = False
                    print("CMD:", self.linebuffer.decode(errors="ignore"))
                    self.parseLine(self.linebuffer.decode(errors="ignore"))
                elif ch == '\n' or ch == '\r':
                    self.readingLine = False

    def parseLine(self, cmdString):
        global navigationMode
        try:
            cmd = json.loads(cmdString)
            print("cmdString: ",cmdString,type(cmd)) # {"rsdist": 850}
        except Exception as e:
            print ("PARSELINE - ERROR PARSING - SERIAL INPUT:", cmdString)
            print (e)
            return



        if "sp" in cmd:
            motion.setNavSpeed(cmd['sp'])

            print("SPEED - SP:",cmd['sp'])
            self.Serial.println("SPEED - SP:"+str(cmd['sp']))
        elif "rsdist" in cmd:
            try:
                print ("RSDIST: ", cmd['rsdist'])
                rsMinsClient.sendData(cmd['rsdist'])
                self.Serial.println("RSDIST: "+str(cmd['rsdist']))

            except:
                print ("ERROR SENDING RSDIST", cmd['rsdist'])
                self.Serial.println("ERROR SENDING RDIST :"+str(cmd['rsdist']))

        elif "gofw" in cmd:
            cmddrive.addQueueGoForward(cmd['gofw'])
            navigationMode = NAVIGATION_MODE_CMD
            print("gofw: "+str(cmd['gofw']))
            self.Serial.println("gofw: " + str(cmd['gofw']))

        elif "turn" in cmd:
            cmddrive.addQueueTurn(cmd['turn'])
            navigationMode = NAVIGATION_MODE_CMD
            print("turn: " + str(cmd['turn']))
            self.Serial.println("turn: " + str(cmd['turn']))
        else:
            self.Serial.println('COMMAND NOT IDENTIFIED:'+cmdString)
            print('COMMAND NOT IDENTIFIED: '+cmdString)


    def sendMsg(self, type, details):
        msgStr="{" + str(type) + ";" + str(details) +";}"
        self.Serial.println (msgStr)

    # --- END CLASS --------------------------------

class CmdDrive:
    STATUS_DONE=0
    STATUS_RUNNING_FWD=1
    STATUS_RUNNING_TURN= 2
    STATUS_RUNNING_WAIT = 3
    status=0

    startDistance=0
    setDistance=0

    startAngle=0
    setAngle=0
    dstAngle=0

    speedLeft=0
    speedRight=0

    maxSpeed=0.20
    minSpeed=0.10

    cmdQueue=[]

    previousNavMode=NAVIGATION_MODE_MANUAL

    def update(self):
        encDistance=motionData['enc']
        accAngle=motionData['accZ']

        self.speedLeft = 0
        self.speedRight = 0
        # AVANCE
        if self.status==self.STATUS_RUNNING_FWD:
            travelled=encDistance-self.startDistance
            error=(self.setDistance-travelled) / self.setDistance
            #print (error)
            if travelled < self.setDistance:
                if self.setDistance-travelled > 0.2:
                    self.speedLeft = self.maxSpeed
                    self.speedRight = self.maxSpeed
                else:
                    self.speedLeft = self.maxSpeed * error
                    self.speedRight = self.maxSpeed * error
                    if self.speedLeft < self.minSpeed:
                        self.speedLeft = self.minSpeed
                    if self.speedRight < self.minSpeed:
                        self.speedRight = self.minSpeed

            else:
                self.cmdFinished()
        # GIRO
        if self.status == self.STATUS_RUNNING_TURN:
            # SI ESTOY DOBLANDO A LA DERECHA
            if self.setAngle > 0:
                if self.startAngle > self.dstAngle:
                    if accAngle >=0:
                        accAngle=accAngle-360

                if accAngle < self.dstAngle:
                    self.speedLeft = self.maxSpeed
                    if abs(abs(accAngle) - abs(self.dstAngle)) < 8:
                        self.speedLeft = self.minSpeed
                else:
                    self.cmdFinished()

            # SI ESTOY DOBLANDO A LA IZQUIERDA
            else:
                if self.startAngle < self.dstAngle:
                    if accAngle <= 0:
                        accAngle = accAngle + 360

                if accAngle > self.dstAngle:
                    self.speedRight = self.maxSpeed
                    if abs(abs(accAngle) - abs(self.dstAngle)) < 8:
                        self.speedRight = self.minSpeed
                else:
                    self.cmdFinished()

        return self.speedRight, self.speedLeft

    def cmdFinished(self):
        global navigationMode
        self.status=self.STATUS_DONE
        if len(self.cmdQueue) == 0:
            print ("CMD NAV FINISHED")
            print (self.previousNavMode)
            navigationMode = self.previousNavMode
            vser.sendMsg("ex", "cmddone")



    def emptyQueue(self):
        self.cmdQueue.clear()
        self.status = self.STATUS_DONE

    def processQueue(self):
        #print(self.cmdQueue)
        if self.status == self.STATUS_DONE and len(self.cmdQueue) > 0:
            cmd=self.cmdQueue[0]
            if cmd[0] == self.STATUS_RUNNING_FWD:
                self.goForward(cmd[1])
            elif cmd[0] == self.STATUS_RUNNING_TURN:
                self.turn(cmd[1])
            self.cmdQueue.pop(0)



    def addQueueGoForward (self, distance):
        if navigationMode!=NAVIGATION_MODE_CMD:
            self.previousNavMode=navigationMode
        self.cmdQueue.append([self.STATUS_RUNNING_FWD, distance])

    def addQueueTurn(self, angle):
        if navigationMode!=NAVIGATION_MODE_CMD:
            self.previousNavMode=navigationMode
        self.cmdQueue.append([self.STATUS_RUNNING_TURN, angle])

    def goForward(self, distance):
        if distance <= 0:
            self.status=self.STATUS_DONE
            return
        else:
            self.status=self.STATUS_RUNNING_FWD
            self.startDistance=motionData['enc']
            self.setDistance=distance

    def turn(self, angle):
        if angle > 180 or angle == 0 or angle < -180:
            self.status = self.STATUS_DONE
            return
        else:
            self.status=self.STATUS_RUNNING_TURN
            self.startAngle=motionData['accZ']
            self.setAngle=angle
            self.dstAngle=self.startAngle+angle
            if self.dstAngle > 180:
                self.dstAngle = self.dstAngle - 360
            elif self.dstAngle < -180:
                self.dstAngle = self.dstAngle + 360


class Motion:
    ABSOLUTE_MAX_SPEED=1.0
    spMax=0.32
    spRight=0.0
    spLeft=0.0
    emergencyStop=False

    def setNavSpeed(self, navSpeed):
        if navSpeed > self.ABSOLUTE_MAX_SPEED:
            navSpeed=self.ABSOLUTE_MAX_SPEED
        self.spMax=navSpeed

    def setWheelsSpeedAbsolute(self, spRight, spLeft):
        self.spRight=spRight
        self.spLeft=spLeft

    def setWheelSpeedPercentage(self, spRight, spLeft):
        self.spRight=(self.spMax * spRight) / 100
        self.spLeft=(self.spMax * spLeft) / 100

    def reduceWheelsSpeed (self, ratio):
        if ratio > 0:
            self.spRight=self.spRight/ratio
            self.spLeft=self.spLeft/ratio

    def getWheelsSpeed(self):
        return self.spRight, self.spLeft

    def updateMotors(self):
        if self.emergencyStop==True:
            self.spRight=0
            self.spLeft=0
        megaWrite(self.spRight, self.spLeft)

print ("SETTING DRIVE MODES ")
pixydrive = PixyDriveTEST()
manualdrive = ManualDrive()
realsenseDrive = RealsenseDrive()
cmddrive = CmdDrive()
joydrive = JoyDrive()

print ("STARTING VIRTUAL SERIAL")
vser=VirtualSerialHandler()
vser.start()

print ("ENCODER")
encData=EncoderData()
motion = Motion()

print ("MAINNAVSTATUS")
mainNavStatus=MainNavStatus()





lidarPause=False
lidarPauseTime=0

lidarEnabled=True


print ("Waiting 5 seconds")
time.sleep(5)
print ("READY to Drive!")

startTime=time.time();

while True:
    mainNavStatus.updateData()
    joystickClient.checkDataUpdate()
    rsMinsClient.checkDataUpdate()
    lidarMinClient.checkDataUpdate()
    pixyClient.checkDataUpdate()

    #print ("ACCZ:", str(motionData['accZ']), "START:", cmddrive.startAngle, "DST:", cmddrive.dstAngle, "SET:", cmddrive.setAngle)
    vser.update()
    encData.update()
    encData.calcSpeed()
    #print ("ENC-SPEED:", encData.speed)
    realsense_spRight, realsense_spLeft = realsenseDrive.update(rsData=realsenseMins)
    pixy_spRight, pixy_spLeft = pixydrive.update(pixyData)

    if navigationMode == NAVIGATION_MODE_MANUAL:
        lidarPause=False
        encData.resetTimer()
        cmddrive.emptyQueue()

        motion.setWheelsSpeedAbsolute(manualdrive.speedRight, manualdrive.speedLeft)
        motion.updateMotors()

    elif navigationMode == NAVIGATION_MODE_JOYSTICK:
        encData.resetTimer()

        joydrive.update(joyData)
        #print(joydrive.speedRight, joydrive.speedLeft)
        motion.setWheelsSpeedAbsolute(joydrive.speedRight, joydrive.speedLeft)
        motion.updateMotors()

    elif navigationMode == NAVIGATION_MODE_CMD:
        cmddrive.processQueue()
        cmd_spRight, cmd_spLeft = cmddrive.update()
        motion.setWheelsSpeedAbsolute(cmd_spRight, cmd_spLeft)

    elif navigationMode == NAVIGATION_MODE_PIXY:
        if not lidarPause:
            print ("PIXY ONLY DRIVE", "PID ERROR:", pixydrive.error)
        manualdrive.stop()
        motion.setWheelSpeedPercentage(pixy_spRight, pixy_spLeft)
        if pixydrive.noVector==True:
            print ("PIXY NO LINE!")
            megaBeep(250, 1)
            vser.sendMsg("ex", "pixyoff")
            navigationMode=NAVIGATION_MODE_MANUAL

    elif navigationMode == NAVIGATION_MODE_REALSENSE:
        manualdrive.stop()
        # PASAR A REALSENSE CUAND ESTAN LOS DOS MINS ACTIVOS PERO NO SALIR DE REALSENSE SI SE PIERDE EL DE ATRAS
        #motion.setWheelSpeedPercentage(realsense_spRight, realsense_spLeft)
        #OJO QUE ESTA PARA LA BARRA DEL OTRO LADO!!!
        motion.setWheelSpeedPercentage(realsense_spLeft, realsense_spRight)
        if realsenseDrive.barDetected == True:
            if not lidarPause:
                print ("RS DRIVE:", "BAR DETECTED:", realsenseDrive.barDetected, "MINS:", realsenseMins, "PID ERR:", realsenseDrive.error)
        elif pixydrive.noVector == False:
            if not lidarPause:
                print("RS DRIVE: NO BAR - USING PIXY")
            motion.setWheelSpeedPercentage(pixy_spRight, pixy_spLeft)
        else:
            megaBeep(250, 1)
            print("RS DRIVE: NO REALSENSE - NO PIXY - STOPPING!")
            navigationMode = NAVIGATION_MODE_MANUAL
            manualdrive.stop()
    # CRITICT TILT
    if abs(motionData['accX']) >= 4 or abs(motionData['accY']) >= 4:
        if motion.emergencyStop==False:
            vser.sendMsg("ex", "critictilt")
            megaBeep(500, 1)
        manualdrive.stop()
        motion.emergencyStop=True

    # PAUSA POR LIDAR
    if (navigationMode != NAVIGATION_MODE_MANUAL):
        if len(lidarMins) > 0:
            if min(lidarMins) < 500:

                if lidarEnabled==True:
                    if lidarPause==False:
                        print("OBJETO A MENOS DE 0.5 METROS -", lidarMins)
                        megaBeep(50, 3)
                        vser.sendMsg("ex", "upause")
                    lidarPause=True
                    motion.setWheelsSpeedAbsolute(0, 0)
                    motion.updateMotors()
                    encData.resetTimer()
                    lidarPauseTime=millis()

            elif min(lidarMins) < 800:
                print ("OBJETO A MENOS DE 0.8 METROS -", lidarMins)
                if lidarEnabled==True:
                    motion.reduceWheelsSpeed(ratio=2)

        if (millis()-lidarPauseTime) > 2000 or lidarEnabled==False:
            lidarPause = False
            motion.updateMotors()

    # SE DETIENE POR FALTA DE ENCODER
    noEncoderTimeout=3000
    if navigationMode==NAVIGATION_MODE_CMD:
        noEncoderTimeout=6000


    if navigationMode != NAVIGATION_MODE_MANUAL and encData.getTimer() > noEncoderTimeout and lidarPause==False:
        print("NO ENCODER DATA")
        vser.sendMsg("ex", "enoreading")
        navigationMode = NAVIGATION_MODE_MANUAL
        manualdrive.stop()


    #time.sleep(0.5)
    time.sleep(0.025)

print ("CRITICAL ERROR.. NAVIGATION STOPPED.. CHECK SERVICES")
vser.sendMsg("ex", "criticalerror")
while (True):
    pass
    time.sleep(0.5)
