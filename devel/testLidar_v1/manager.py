from tkinter import *
from tkinter.scrolledtext import *
import multiprocessing
import joystickService
import lidarServiceV2
import lidarMinsService,wrapCam_proc
import realsenseService
import realsenseMinsService
import pixyService
import time

import threading as th

autoStart=False


import subprocess
#p=subprocess.Popen('python C:/smartcube/devel/testLidar_v1/mainNav.py', shell=True, stdout=subprocess.PIPE)


class ExternalProc:
    procHandler = None
    t1 = None

    def __init__(self, text, execCommand, procTitle="", newConsole=False, useShell=False):
        self.tkText = text
        self.execCommand = execCommand
        self.procTitle = procTitle
        self.newConsole = newConsole
        self.useShell = useShell

    def logOutput(self):
        while True:
            try:
                line = self.procHandler.stdout.readline()
            except:
                break
            if line:
                self.tkText.addText(line)
            else:
                time.sleep(0.010)

    def isRuninng(self):
        if self.procHandler is not None:
            if self.procHandler.is_alive():
                return True
        return False

    def start(self):
        if self.isRuninng():
            print("ALREADY RUNNING")
            self.tkText.addText("ALREADY RUNNING")
            return

        print("STARTING " + self.procTitle + " IN CONSOLE")
        self.tkText.addText("STARTING " + self.procTitle + " IN CONSOLE")
        # self.tkText.clearText()
        # self.procHandler = subprocess.Popen('python.exe C:/smartcube/devel/testLidar_v1/mainNav.py', shell=True, creationflags = subprocess.CREATE_NEW_CONSOLE )
        if self.newConsole==True:
            newConsoleFlag=subprocess.CREATE_NEW_CONSOLE
        else:
            newConsoleFlag=0
        print ("USE SHELL", self.useShell)
        self.procHandler = subprocess.Popen(self.execCommand, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=self.useShell, creationflags=newConsoleFlag)
        # creationflags=subprocess.CREATE_NEW_CONSOLE)
        self.t1 = th.Thread(target=self.logOutput)
        self.t1.setDaemon(True)
        self.t1.start()

    def kill(self):

        import psutil
        if self.procHandler is None:
            self.tkText.addText("NO PROCESS WAS RUNNING")
            print ("NO PROCESS WAS RUNNING")
            return
        print("KILLING")
        parentPid = self.procHandler.pid
        print(parentPid)
        parent = psutil.Process(parentPid)
        for child in parent.children(recursive=True):  # or parent.children() for recursive=False
            child.kill()
        parent.kill()

        self.tkText.clearText()
        self.tkText.addText("PROCESS STOPPED!")
        # self.procHandler.terminate()
        self.procHandler = None


class camProc:
    procHandler=None
    t1=None

    def __init__(self, text):
        self.tkText = text

    def logOutput(self):
        while True:


            line=self.procHandler.stdout.read()
            if line:
                self.tkText.addText(line)
            time.sleep(0.010)

    def isRuninng(self):
        if self.procHandler is not None:
            if self.procHandler.is_alive():
                return True
        return False

    def start(self):
        if self.isRuninng():
            print ("ALREADY RUNNING")
            self.tkText.addText("ALREADY RUNNING")
            return

        print("STARTING WEBCAM IN CONSOLE")
        self.tkText.addText("STARTING WEBCAM IN CONSOLE")
        #self.tkText.clearText()
        #self.procHandler = subprocess.Popen('python.exe C:/smartcube/devel/testLidar_v1/mainNav.py', shell=True, creationflags = subprocess.CREATE_NEW_CONSOLE )
        self.procHandler = subprocess.Popen('python.exe C:/smartcube/devel/testLidar_v1/wrapCAm.py', shell=False)
                                            #creationflags=subprocess.CREATE_NEW_CONSOLE)
        self.t1=th.Thread(target=self.logOutput)
        self.t1.setDaemon(True)
        self.t1.start()

    def kill (self):
        print ("KILLING")
        import psutil
        parentPid=self.procHandler.pid
        print (parentPid)
        parent=psutil.Process(parentPid)
        for child in parent.children(recursive=True):  # or parent.children() for recursive=False
            child.kill()
        parent.kill()

        self.tkText.clearText()
        self.tkText.addText("PROCESS STOPPED!")
        #self.procHandler.terminate()
        self.procHandler=None


class navProc:
    procHandler=None
    t1=None

    def __init__(self, text):
        self.tkText = text

    def logOutput(self):
        while True:


            line=self.procHandler.stdout.read()
            if line:
                self.tkText.addText(line)
            time.sleep(0.010)

    def isRuninng(self):
        if self.procHandler is not None:
            if self.procHandler.is_alive():
                return True
        return False

    def start(self):
        if self.isRuninng():
            print ("ALREADY RUNNING")
            self.tkText.addText("ALREADY RUNNING")
            return

        print("STARTING IN A NEW CONSOLE")
        self.tkText.addText("STARTING IN A NEW CONSOLE")
        #self.tkText.clearText()
        #self.procHandler = subprocess.Popen('python.exe C:/smartcube/devel/testLidar_v1/mainNav.py', shell=True, creationflags = subprocess.CREATE_NEW_CONSOLE )
        self.procHandler = subprocess.Popen('python.exe C:/smartcube/devel/testLidar_v1/mainNav.py', shell=False,
                                            creationflags=subprocess.CREATE_NEW_CONSOLE)
        self.t1=th.Thread(target=self.logOutput)
        self.t1.setDaemon(True)
        self.t1.start()

    def kill (self):
        print ("KILLING")
        import psutil
        parentPid=self.procHandler.pid
        print (parentPid)
        parent=psutil.Process(parentPid)
        for child in parent.children(recursive=True):  # or parent.children() for recursive=False
            child.kill()
        parent.kill()

        self.tkText.clearText()
        self.tkText.addText("PROCESS STOPPED!")
        #self.procHandler.terminate()
        self.procHandler=None




class PythonProc:

    procHandler=None
    t1=None

    def __init__(self, text, scriptName, serviceStartFunction):
        self.tkText = text
        self.scriptName= scriptName
        self.pOut, self.pIn = multiprocessing.Pipe()
        self.serviceStartFunction=serviceStartFunction

    def logOutput(self, filename):

        while True:
            if self.pIn.poll(0.001):
                self.tkText.addText (self.pIn.recv())
    def isRuninng(self):
        if self.procHandler is not None:
            if self.procHandler.is_alive():
                return True
        return False
    def start(self):
        if self.isRuninng():
            print ("ALREADY RUNNING")
            return

        print("STARTING ")
        self.tkText.clearText()
        #joystickService.serviceStart(self.pOut, self.pIn, False)
        self.procHandler = multiprocessing.Process(target=self.serviceStartFunction, args=(self.pOut, self.pIn, False))
        self.procHandler.daemon=True
        self.procHandler.start()
        self.t1=th.Thread(target=self.logOutput, args=(self.scriptName,))
        self.t1.setDaemon(True)
        self.t1.start()

    def kill (self):
        print ("KILLING")
        self.tkText.clearText()
        self.procHandler.terminate()

class tkProcFrame:
    def __init__(self, root, serviceName):
        self.root = root
        self.serviceName = serviceName
        self.frame = Frame(root, relief="raised", height=10, width=150, border=2)
        #self.text=Text(self.frame, bg="black", fg="white", bd=2, width=100, height=20)
        self.text=ScrolledText(self.frame, bg="#202020", fg="#a0a0a0", height='5', width='45', wrap=WORD)
        self.label=Label(self.frame, text = self.serviceName)
        self.btnStart = Button(self.frame, text="START")
        self.btnStop = Button(self.frame, text="STOP")
        self.frame.pack(pady=4)
        self.label.pack()
        self.btnStart.pack(padx=5, side=LEFT)
        self.btnStop.pack(padx=5, side=LEFT)
        self.text.pack(padx=5)

    def addText(self, textToInsert):
        self.text.configure(state='normal')
        self.text.insert(END, textToInsert)
        self.text.yview(END)
        lineCount=int(self.text.index('end-1c').split('.')[0])
        if lineCount > 100:
            self.text.delete("1.0", "2.0")
        self.text.configure(state='disabled')
    def clearText(self):
        self.text.configure(state='normal')
        self.text.delete("1.0", "end")
        self.text.configure(state='disabled')
    def setStartFunc(self, startFunc):
        self.btnStart.configure(command=startFunc)

    def setStopFunc(self, stopFunc):
        self.btnStop.configure(command=stopFunc)


class tkProcNav():
    def __init__(self, root):
        self.root=root
        self.frame = Frame(self.root, relief="raised", height=20, width=150, border=2)
        self.text = ScrolledText(self.frame, bg="#202020", fg="#a0a0a0", height='15', width='65', wrap=WORD)
        self.label = Label(self.frame, text="MAIN NAV PROCESS")
        self.btnStart = Button(self.frame, text="START")
        self.btnStop = Button(self.frame, text="STOP")
        self.frame.pack(pady=4, padx=4)

        self.label.pack()
        self.btnStart.pack(padx=5, side=LEFT)
        self.btnStop.pack(padx=5, side=LEFT)

        self.text.pack(padx=5, pady=5)
    def addText(self, textToInsert):
        self.text.configure(state='normal')
        self.text.insert(END, textToInsert)
        self.text.yview(END)
        lineCount=int(self.text.index('end-1c').split('.')[0])
        if lineCount > 100:
            self.text.delete("1.0", "2.0")
        self.text.configure(state='disabled')
    def clearText(self):
        self.text.configure(state='normal')
        self.text.delete("1.0", "end")
        self.text.configure(state='disabled')
    def setStartFunc(self, startFunc):
        self.btnStart.configure(command=startFunc)

    def setStopFunc(self, stopFunc):
        self.btnStop.configure(command=stopFunc)

class tkProcCam():
    def __init__(self, root):
        self.root=root
        self.frame = Frame(self.root, relief="raised", height=20, width=150, border=2)
        self.text = ScrolledText(self.frame, bg="#202020", fg="#a0a0a0", height='1', width='65', wrap=WORD)
        self.label = Label(self.frame, text="CAM PROCESS")
        self.btnStart = Button(self.frame, text="START")
        self.btnStop = Button(self.frame, text="STOP")
        self.frame.pack(pady=4, padx=4)

        self.label.pack()
        self.btnStart.pack(padx=5, side=LEFT)
        self.btnStop.pack(padx=5, side=LEFT)

        self.text.pack(padx=5, pady=5)
    def addText(self, textToInsert):
        self.text.configure(state='normal')
        self.text.insert(END, textToInsert)
        self.text.yview(END)
        lineCount=int(self.text.index('end-1c').split('.')[0])
        if lineCount > 100:
            self.text.delete("1.0", "2.0")
        self.text.configure(state='disabled')
    def clearText(self):
        self.text.configure(state='normal')
        self.text.delete("1.0", "end")
        self.text.configure(state='disabled')
    def setStartFunc(self, startFunc):
        self.btnStart.configure(command=startFunc)

    def setStopFunc(self, stopFunc):
        self.btnStop.configure(command=stopFunc)


# HABRIA Q PONER UN INPUT FRAME PARA DEFINIR LA DISTANCIA DESDE EL REAL SENSE A LA CUAL QUEREMOS NAVEGAR
# O DESDE LA OTRA APLICACION.. PERO DEBERÍAMOS DAR FEEDBACK A QUÉ DISTANCIA ESTAMOS NAVEGANDO.

if __name__ == "__main__":

    root = Tk()
    root.geometry("1200x800")
    root.title("MANAGER")
    root.resizable(False, False)


    frameServices = Frame(root, height = 100)
    frameRight = Frame(root, height= 500)
    frameNav = Frame(frameRight, height=100)
    frameCam = Frame(frameRight, height=100)

    frameServices.pack(side=LEFT, anchor=NW, padx=(10,10), pady=(10,10))
    frameRight.pack(side=RIGHT, anchor=NE, padx=(10,10), pady=(10,10))
    #frameNav.pack (side=LEFT, anchor=NE, padx=(10,10), pady=(10,10))
    frameNav.pack(padx=(10, 10), pady=(10, 10))
    frameCam.pack(padx=(10,10), pady=(10,10))




    procFrame1 = tkProcFrame(frameServices, "Lidar Service")
    procFrame2 = tkProcFrame(frameServices, "Lidar Mins Service")
    procFrame3 = tkProcFrame(frameServices, "Realsense Service")
    procFrame4 = tkProcFrame(frameServices, "Realsense Mins Service")
    procFrame5 = tkProcFrame(frameServices, "Pixy Service")
    procFrame6 = tkProcFrame(frameServices, "Joystick Service")
    # migue crap
    # procFrame7 = tkProcFrame(frameServices, "Webcam Service")


    mainNavFrame = tkProcNav(frameNav)
    mainCamFrame = tkProcCam(frameCam)
    #text, execCommand, procTitle = ""
    procCmd = 'C:/Users/Usuario/AppData/Local/Programs/Python/Python39/python.exe -u C:/smartcube/devel/testLidar_v1/mainNav.py'
    z1=ExternalProc(text=mainNavFrame, execCommand=procCmd, procTitle="MAINNAV", newConsole=False)
    mainNavFrame.setStartFunc(z1.start)
    mainNavFrame.setStopFunc(z1.kill)

    procCmd = 'C:/Users/Usuario/AppData/Local/Programs/Python/Python39/python.exe -u C:/smartcube/devel/testLidar_v1/wrapCAm.py'
    #procCmd = 'python.exe C:/smartcube/devel/testLidar_v1/wrapCAm.py'

    z2=ExternalProc(text=mainCamFrame, execCommand=procCmd, procTitle="WRAPCAM", newConsole=False, useShell=True)
    mainCamFrame.setStartFunc(z2.start)
    mainCamFrame.setStopFunc(z2.kill)

    p1 = PythonProc(procFrame1, "lidarService", lidarServiceV2.serviceStart)
    procFrame1.setStartFunc(p1.start)
    procFrame1.setStopFunc(p1.kill)

    p2 = PythonProc(procFrame2, "lidarMinsService", lidarMinsService.serviceStart)
    procFrame2.setStartFunc(p2.start)
    procFrame2.setStopFunc(p2.kill)

    # PARA CONECTAR REALSENSE A LA COMP.. DESCOMENTAR ESTAS LINEAS

    # p3 = PythonProc(procFrame3, "realsenseService", realsenseService.serviceStart)
    # procFrame3.setStartFunc(p3.start)
    # procFrame3.setStopFunc(p3.kill)
    #
    # p4 = PythonProc(procFrame4, "realsenseMinsService", realsenseMinsService.serviceStart)
    # procFrame4.setStartFunc(p4.start)
    # procFrame4.setStopFunc(p4.kill)

    p5=PythonProc(procFrame5, "pixyService", pixyService.serviceStart)
    procFrame5.setStartFunc(p5.start)
    procFrame5.setStopFunc(p5.kill)

    p6=PythonProc(procFrame6, "joystickService", joystickService.serviceStart)
    procFrame6.setStartFunc(p6.start)
    procFrame6.setStopFunc(p6.kill)

    # migueCrap
    # p7 = PythonProc(procFrame7, "webCamService", wrapCam_proc.serviceStart())
    # procFrame7.setStartFunc(p7.start)
    # procFrame7.setStopFunc(p7.kill)





    if autoStart:
        # SERVICIOS SIN DEPENDENCIA
        p1.start()

        #p3.start()
        #p4.start()
        p5.start()
        p6.start()
        z2.start()

        # SERVICIOS CON DEPENDENCIA
        def delayedStart():
            startDelay=8
            procFrame2.addText("STARING IN " + str(startDelay))

            time.sleep(startDelay)
            p2.start()
            time.sleep(startDelay)
            z1.start()

            #time.sleep (5)
            #root.iconify()


        t=th.Thread(target=delayedStart)
        t.start()


    root.mainloop()


    p1.kill()
    p2.kill()
    # p3.kill()
    # p4.kill()
    p5.kill()
    p6.kill()
    z1.kill()
    z2.kill()


