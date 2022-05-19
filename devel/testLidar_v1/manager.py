import os
from tkinter import *
from tkinter.scrolledtext import *
import subprocess
import os, signal
from subprocess import Popen, PIPE
import threading as th
import time

PYTHON_ENV="C:/Users/Usuario/AppData/Local/Programs/Python/Python39/python.exe"

class externProc:
    procHandler=0
    t1=0

    def __init__(self, text, scriptName):
        self.tkText = text
        self.scriptName= scriptName
    def logOutput(self, filename):
        print("STARTING ", filename )
        while self.procHandler.poll() is None:
            line=self.procHandler.stdout.readline()
            self.tkText.addText(line.decode())

    def start(self):
        self.tkText.clearText()
        self.procHandler = Popen([PYTHON_ENV, self.scriptName], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False)
        self.t1=th.Thread(target=self.logOutput, args=(self.scriptName,))
        #self.t1.setDaemon(True)
        self.t1.start()
    def kill (self):
        self.tkText.clearText()
        try:
            os.kill(self.procHandler.pid, signal.SIGTERM)
        except:
            self.tkText.addText("-- NO PROCESS RUNNING --")
        else:
            self.tkText.addText("-- PROCESS KILLED --")

class tkProcFrame:
    def __init__(self, root, serviceName):
        self.root = root
        self.serviceName = serviceName
        self.frame = Frame(root, relief="raised", height=20, width=150, border=2)
        #self.text=Text(self.frame, bg="black", fg="white", bd=2, width=100, height=20)
        self.text=ScrolledText(self.frame, bg="#202020", fg="#a0a0a0", height='10', width='45', wrap=WORD)
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

root = Tk()
root.geometry("800x800")
root.title("MANAGER")
#frame1 = Frame(root, height = 20)
#frame2 = Frame(root, height = 20)
#text = Text(frame1, bg="black", fg="white", bd=2, width=100, height=20)
#text2 = Text(frame2, bg="black", fg="white", bd=2, width=100, height=20)

procFrame1=tkProcFrame(root, "Realsense Service")
procFrame2=tkProcFrame(root, "Lidar Service")
procFrame3=tkProcFrame(root, "Lidar Mins Service")
procFrame4=tkProcFrame(root, "Nano Control")

p1=externProc(procFrame1, "realsenseService.py")
procFrame1.setStartFunc(p1.start)
procFrame1.setStopFunc(p1.kill)

p2=externProc(procFrame2, "lidarService.py")
procFrame2.setStartFunc(p2.start)
procFrame2.setStopFunc(p2.kill)

p3=externProc(procFrame3, "lidarMinsService.py")
procFrame3.setStartFunc(p3.start)
procFrame3.setStopFunc(p3.kill)

p4=externProc(procFrame4, "nanoControl.py")
procFrame4.setStartFunc(p4.start)
procFrame4.setStopFunc(p4.kill)




root.mainloop()
p1.kill()
p2.kill()
p3.kill()
p4.kill()

