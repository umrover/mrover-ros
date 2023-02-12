import numpy as np 
import cv2 
from multiprocessing import Process

def receive(port = 5001):
    cr = 'udpsrc port='+str(port)+' "application/x-rtp, encoding-name=H264, payload=96" ! \
    rtph264depay ! \
    decodebin ! \
    videoconvert ! \
    autovideosink'
    cap_receive = cv2.VideoCapture(cr, cv2.CAP_GSTREAMER)

    if not cap_receive.isOpened():
        print('VideoCapture not opened')
        exit(0)

    while True:
        ret,frame = cap_receive.read()

        if not ret:
            print('empty frame')
            break

        cv2.imshow('receive', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break

    #cap_receive.release()
    
if __name__ == '__main__':
    r1 = Process(target=receive, args=(5001,))
    r2 = Process(target=receive, args=(5002,))
    r1.start()
    r2.start()
    r1.join()
    r2.join()

    cv2.destroyAllWindows()
