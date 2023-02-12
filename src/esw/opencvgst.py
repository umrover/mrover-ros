import numpy as np 
import cv2 
from multiprocessing import Process

def send():
    cap_send = cv2.VideoCapture('v4l2src device=/dev/video0 do-timestamp=true ! image/jpeg, width=1280, height=720 ! jpegdec ! videorate ! video/x-raw, framerate=30/1 ! nvvidconv ! appsink name=mysink', cv2.CAP_GSTREAMER)

    out_send = cv2.VideoWriter('appsrc name=mysource is-live=true do-timestamp=true format=3 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvv4l2h264enc bitrate=4000000 ! video/x-h264 !  rtph264pay config-interval=1 ! udpsink host=10.0.0.7 port=5001 auto-multicast=true' ,cv2.CAP_GSTREAMER,0, 30, (1280,720), True)

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened')
        exit(0)

    while True:
        ret,frame = cap_send.read()

        if not ret:
            print('empty frame')
            break

        out_send.write(frame)

        cv2.imshow('send', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break

    cap_send.release()
    out_send.release()

def receive():
    cap_receive = cv2.VideoCapture('udpsrc port=5001 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

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
    s = Process(target=send)
    r = Process(target=receive)
    s.start()
    r.start()
    s.join()
    r.join()

    cv2.destroyAllWindows()