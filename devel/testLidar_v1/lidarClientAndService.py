import time
import numpy as np

# Select the correct UART port automatically
TCP_PORT_CLIENT = 6000
TCP_PORT = 6001

minimos = []

# calcula los minimos
def calc_min(list_angulos,slices):
	arr = np.array(list_angulos[:-1])
	arr = arr.astype(int)
	arr[arr==-1] = 6000
	list_minimos = []
	for slice in slices:
		desde=slice[0]
		hasta=slice[1]
		m=min(arr[desde:hasta])
		list_minimos.append(m)
	return list_minimos


#Esta rutina lee el puerto
def run(lock):
	from multiprocessing.connection import Client
	# init()
	address = ('localhost', 6000)
	conn = Client(address)
	global minimos
	try:
		while True:
			list_angulos = [[0, 90], [270, 359]]
			lidarAngles = conn.recv()
			local_minimos=calc_min(lidarAngles, list_angulos)
			with lock:
				minimos=local_minimos
	finally:
		print ("CLIENT END!")



if __name__ == "__main__":
    import multiprocessing as mp
    import threading as th
    from multiprocessing.connection import Listener

    address = ('localhost', TCP_PORT)  # family is deduced to be 'AF_INET'
    listener = Listener(address)


    def conHandler(conn, listener, lock):
        print('connection accepted from', listener.last_accepted)
        global np_distance
        while True:
            with g_lock:
                try:
                    conn.send(minimos)
                except:
                    print ("CANT SEND!")
                    break;
            time.sleep(0.5)
        print ("DISCONNECTED")
        conn.close()
        print("THREAD END")



    # Global lock
    g_lock = th.Lock()
    print ("INZONE Service started")
    thSerialRead = th.Thread(target=run, args=(g_lock,))
    thSerialRead.setDaemon(True)
    thSerialRead.start()
    print ("Reading LIDAR on PORT", TCP_PORT_CLIENT)
    print ("Listening TCP on PORT", TCP_PORT)
    try:
        while True:
            c = listener.accept()
            th.Thread(target=conHandler, args=(c, listener,g_lock,)).start()
    except KeyboardInterrupt:
       print ("EXIT")
    listener.close()
    p.join()