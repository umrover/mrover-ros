import numpy as np 
import cv2 
from multiprocessing import Process

def send(device = 0, host = '10.0.0.7', port = 5001, bitrate = 4000000, width = 1280, height = 720, fps = 30, colored = False):
    vc = 'v4l2src device=/dev/video' + str(device) + ' do-timestamp=true io-mode=2 ! \
    image/jpeg, width=1280, height=720 ! \
    jpegdec ! \
    videorate ! \
    video/x-raw,framerate='+str(fps)+'/1 ! \
    nvvidconv ! '
    if colored:
        vc += ' video/x-raw, format=BGRx ! '
    vc += 'videoconvert ! '
    if colored:
        vc += ' video/x-raw, format=BGR ! '
    vc += 'appsink'
    cap_send = cv2.VideoCapture(vc, cv2.CAP_GSTREAMER)
    
    vw = 'appsrc ! '
    if colored:
        vw += ' video/x-raw, format=BGR ! '
    vw += 'videoconvert ! '
    if colored:
        vw += ' video/x-raw, format=BGRx ! '

    vw += 'nvvidconv ! \
    video/x-raw(memory:NVMM), width='+str(width)+', height='+str(height)+' ! \
    nvv4l2h264enc bitrate=' + str(bitrate) + ' ! \
    h264parse ! \
    rtph264pay pt=96 config-interval=1 ! \
    udpsink host=' + str(host) + ' port=' + str(port)
    out_send = cv2.VideoWriter(vw ,cv2.CAP_GSTREAMER,0, 60, (1280,720), colored)

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened')
        exit(0) 

    while True:
        ret,frame = cap_send.read()

        if not ret:
            print('empty frame')
            break
        out_send.write(frame)

        #cv2.imshow('send', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
    print("stream machine broke")
    cap_send.release()
    out_send.release()

if __name__ == '__main__':
    s1 = Process(target=send, args=(0, '10.0.0.7', 5001, 3000000, 1280, 720, 30, True))
    s2 = Process(target=send, args=(2, '10.0.0.7', 5002, 3000000, 1280, 720, 30, True))
    s1.start()
    s2.start()
    s1.join()
    s2.join()

    cv2.destroyAllWindows()
