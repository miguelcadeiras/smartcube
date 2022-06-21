from tkinter import *
from tkinter.scrolledtext import *
import multiprocessing
import joystickService
import lidarServiceV2
import lidarMinsService
import realsenseService
import realsenseMinsService
import pixyService
import time

import threading as th

autoStart=True

class externProc:

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


if __name__ == "__main__":
    root = Tk()
    root.geometry("1200x800")
    root.title("MANAGER")
    root.resizable(False, False)


    frameServices = Frame(root, height = 100)
    frameNav = Frame(root, height=200)
#frame1 = Frame(root, height = 20)
#frame2 = Frame(root, height = 20)
#text = Text(frame1, bg="black", fg="white", bd=2, width=100, height=20)
#text2 = Text(frame2, bg="black", fg="white", bd=2, width=100, height=20)

    frameServices.pack(side=LEFT, anchor=NW, padx=(10,10), pady=(10,10))
    frameNav.pack (side=RIGHT, anchor=NE, padx=(10,10), pady=(10,10))

    mainNavFrame=tkProcNav(frameNav)

    procFrame1 = tkProcFrame(frameServices, "Lidar Service")
    procFrame2 = tkProcFrame(frameServices, "Lidar Mins Service")
    procFrame3 = tkProcFrame(frameServices, "Realsense Service")
    procFrame4 = tkProcFrame(frameServices, "Realsense Mins Service")
    procFrame5 = tkProcFrame(frameServices, "Pixy Service")
    procFrame6 = tkProcFrame(frameServices, "Joystick Service")


    p1 = externProc(procFrame1, "lidarService", lidarServiceV2.serviceStart)
    procFrame1.setStartFunc(p1.start)
    procFrame1.setStopFunc(p1.kill)

    p2 = externProc(procFrame2, "lidarMinsService", lidarMinsService.serviceStart)
    procFrame2.setStartFunc(p2.start)
    procFrame2.setStopFunc(p2.kill)

    p3 = externProc(procFrame3, "realsenseService", realsenseService.serviceStart)
    procFrame3.setStartFunc(p3.start)
    procFrame3.setStopFunc(p3.kill)

    p4 = externProc(procFrame4, "realsenseMinsService", realsenseMinsService.serviceStart)
    procFrame4.setStartFunc(p4.start)
    procFrame4.setStopFunc(p4.kill)

    p5=externProc(procFrame5, "pixyService", pixyService.serviceStart)
    procFrame5.setStartFunc(p5.start)
    procFrame5.setStopFunc(p5.kill)

    p6=externProc(procFrame6, "joystickService", joystickService.serviceStart)
    procFrame6.setStartFunc(p6.start)
    procFrame6.setStopFunc(p6.kill)


    if autoStart:
        # SERVICIOS SIN DEPENDENCIA
        p1.start()
        p3.start()
        p5.start()
        p6.start()

        # SERVICIOS CON DEPENDENCIA
        def delayedStart():
            startDelay=8
            procFrame2.addText("STARING IN " + str(startDelay))
            procFrame4.addText("STARING IN " + str(startDelay))
            time.sleep(startDelay)
            p2.start()
            p4.start()
            time.sleep (5)
            root.iconify()


        t=th.Thread(target=delayedStart)
        t.start()






    root.mainloop()


    p1.kill()
    p2.kill()
    p3.kill()
    p4.kill()
    p5.kill()
    p6.kill()


