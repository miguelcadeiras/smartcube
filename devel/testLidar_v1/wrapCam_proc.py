import threading

import cv2
import ftplib
# import getmac
import time
import threading as th
# print ("1")

webcam_on = True
# define a video capture object
cameraIndex = 0
vid = cv2.VideoCapture(cameraIndex,cv2.CAP_DSHOW)
frameCount=0
frame_upload_rate=150
# print ("2")


def make_320p(cap):
    cap.set(3, 320)
    cap.set(4, 240)

make_320p(vid)

mac = "webcam"
# print("mac: ",getmac.get_mac_address())
# print(getmac.getmac)
def upload_file(img_path):

    # print("uploadStart")
    try:
        session = ftplib.FTP('cubik.smartcubik.com', "smartcubik", "Chocolatada123!")
        file = open(img_path, 'rb')  # file to send
        session.storbinary("STOR %s" % img_path, file)  # send the file
        file.close()  # close file and FTP
        session.quit()
        # print("uploadFinished")
    except:
        # pass
        print("WrapCam.py Something went wrong")

def mainLoop():
    global webcam_on
    print("starting webcam")
    print( 'webcam_on flag:',webcam_on)
    frameCount = 0

    while (webcam_on):

        # Capture the video frame
        # by frame

        ret, frame = vid.read()
        if ret == False:
            print("NO CV2 FRAME")
            time.sleep(0.500)
            continue
        # Display the resulting frame
        cv2.imshow('frame', frame)
        frameCount +=1
        # Save file
        if frameCount > frame_upload_rate:
            print("sending to Upload")
            img_path = mac + ".jpg"
            cv2.imwrite(img_path, frame)
            x = threading.Thread(target=upload_file(img_path), args=(1,))
            x.start()
            frameCount = 0

        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()




lock = th.Lock()
t1 = th.Thread(target=mainLoop)

def serviceStart():
    global webcam_on
    webcam_on = True
    print("Press Start webcam")
    t1.start()

def stop():
    global webcam_on
    # lock.acquire()
    print("Press Stop webcam")
    webcam_on=False







# serviceStart()