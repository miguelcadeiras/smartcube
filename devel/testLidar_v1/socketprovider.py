import threading as th
from multiprocessing.connection import Listener
import time
import logdata

class socketProvider:
    tcp_port = 0
    address = None
    listener = None
    getData = None
    sendInterval = 0.1

    # def conHandler(self, conn, listener, lock):
    def conHandler(self, conn, listener):
        source = listener.last_accepted
        logdata.log('CONNECTED:', source)
        while True:
            if self.exit == True:
                break
            try:
                conn.send(self.getData())
            except:
                logdata.log("CANT SEND TO:", source)
                break
            time.sleep(self.sendInterval)
        logdata.log('DISCONNECTED:', source)
        conn.close()

    def listen(self, tcp_port):
        self.tcp_port = tcp_port
        self.address = ('localhost', self.tcp_port)  # family is deduced to be 'AF_INET'
        try:
            self.listener = Listener(self.address)
            logdata.log("Listening on:", self.tcp_port)
            while True:
                if self.exit == True:
                    break
                c = self.listener.accept()
                th.Thread(target=self.conHandler, args=(c, self.listener,), daemon=True).start()
            # th.Thread(target=self.conHandler, args=(c, self.listener,self.datalock,), daemon=True).start()

            # time.sleep(0.1)


        except KeyboardInterrupt:
            self.exit = True
            logdata.log("EXIT1")
            self.listener.close()

        except:
            self.exit = True
            logdata.log("ERROR: Socket", self.tcp_port)
            self.listener.close()

        logdata.log("Closing on:", self.tcp_port)
        self.listener.close()

    def start(self, tcp_port):
        self.exit = False
        self.thListener = th.Thread(target=self.listen, args=(tcp_port,))
        self.thListener.setDaemon(True)
        self.thListener.start()

    def stop(self):
        self.listener.close()
        self.exit = True
