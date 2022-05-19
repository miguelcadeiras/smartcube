import threading

import cv2
#import ftplib
# import getmac
import time
print ("1")
# define a video capture object
vid = cv2.VideoCapture(0)
print ("2")
def make_320p(cap):
    cap.set(3, 320)
    cap.set(4, 240)

make_320p(vid)

mac = "webcam"
# print("mac: ",getmac.get_mac_address())
# print(getmac.getmac)
def upload_file(img_path):
    try:
        session = ftplib.FTP('cubik.smartcubik.com', "smartcubik", "Chocolatada123!")
        file = open(img_path, 'rb')  # file to send
        session.storbinary("STOR %s" % img_path, file)  # send the file
        file.close()  # close file and FTP
        session.quit()
    except:
        pass
print ("START")
while (True):

    # Capture the video frame
    # by frame

    ret, frame = vid.read()
    print ("frame")
    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Save file
    # img_path = mac + ".jpg"
    # cv2.imwrite(img_path,frame)
    # upload_file(img_path)
    # print("tic")
    # time.sleep(1)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()