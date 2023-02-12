import numpy as np 
import cv2 
from multiprocessing import Process

def send(device = 0, bitrate = 4000000, host = '10.0.0.7', port = 5001):
    #device = 5
    #bitrate = 4000000
    #host = '10.0.0.7'
    #port = 5001
    #vc = 'v4l2src device=/dev/video0 do-timestamp=true ! image/jpeg, width=1280, height=720 ! jpegdec ! videorate ! video/x-raw,framerate=30/1 ! nvvidconv ! appsink'
    vc = 'v4l2src device=/dev/video' + str(device) + ' do-timestamp=true io-mode=2 ! \
    image/jpeg, width=1280, height=720 ! \
    jpegdec ! \
    nvvidconv ! \
    video/x-raw, format=BGRx ! \
    videoconvert ! \
    video/x-raw, format=BGR ! \
    appsink'
    cap_send = cv2.VideoCapture(vc, cv2.CAP_GSTREAMER)
    #vw = 'appsrc ! video/x-raw, format=BGR ! queue ! nvvidconv ! video/x-raw(memory:NVMM) ! nvv4l2h264enc bitrate=4000000 ! video/x-h264 ! h264parse ! rtph264pay pt=96 config-interval=1 ! udpsink host=10.0.0.7 port=5001 auto-multicast=true '
    vw = 'appsrc ! \
    video/x-raw, format=BGR ! \
    queue ! \
    videoconvert ! \
    video/x-raw, format=BGRx ! \
    nvvidconv ! \
    nvv4l2h264enc bitrate=' + str(bitrate) + ' ! \
    video/x-h264, stream-format=byte-stream ! \
    h264parse ! \
    rtph264pay pt=96 config-interval=1 ! \
    udpsink host=' + str(host) + ' port=' + str(port)
    out_send = cv2.VideoWriter(vw ,cv2.CAP_GSTREAMER,0, 30, (1280,720), True)

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened')
        exit(0) 

    #i = 0
    while True:
        #await asyncio.wait_for(main_loop, 5)
        #i+=1
        ret,frame = cap_send.read()
        #print(frame)

        if not ret:
            print('empty frame')
            break
        #print(ret)
        #print(str(i))
        out_send.write(frame)

        #v2.imshow('send', frame)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
    print("stream machine broke")
    cap_send.release()
    out_send.release()

def receive():
    cr = 'udpsrc port=5001 \
    caps = "application/x-rtp, \
    media=(string)video, \
    clock-rate=(int)90000, \
    encoding-name=(string)H264, \
    payload=(int)96" ! \
    rtph264depay ! \
    decodebin ! \
    videoconvert ! \
    appsink'
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
    s1 = Process(target=send, args=(6, 4000000, '10.0.0.7', 5001))
    s2 = Process(target=send, args=(8, 4000000, '10.0.0.7', 5002))
    s1.start()
    s2.start()
    s1.join()
    s2.join()
    #r = Process(target=receive)
    #r.start()
    #r.join()

    cv2.destroyAllWindows()