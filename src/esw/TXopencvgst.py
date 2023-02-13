import numpy as np
import cv2
from multiprocessing import Process


def send(device=0, host='10.0.0.7', port=5001, bitrate=4000000, width=1280, height=720, fps=30, isColored=False):
    capstr = 'v4l2src device=/dev/video' + str(device) + ' do-timestamp=true io-mode=2 ! \
    image/jpeg, width=1280, height=720 ! \
    jpegdec ! \
    videorate ! \
    video/x-raw,\
    framerate='+str(fps)+'/1 ! \
    nvvidconv ! '
    if isColored:
        capstr += ' video/x-raw, format=BGRx ! '
    capstr += 'videoconvert ! '
    if isColored:
        capstr += ' video/x-raw, format=BGR ! '
    capstr += 'appsink'
    cap_send = cv2.VideoCapture(capstr, cv2.CAP_GSTREAMER)

    txstr = 'appsrc ! '
    if isColored:
        txstr += ' video/x-raw, format=BGR ! '
    vw += 'videoconvert ! '
    if isColored:
        txstr += ' video/x-raw, format=BGRx ! '
    txstr += 'nvvidconv ! \
    video/x-raw(memory:NVMM), \
    width='+str(width)+', \
    height='+str(height)+' ! \
    nvv4l2h264enc \
    bitrate='+str(bitrate)+' ! \
    h264parse ! \
    rtph264pay pt=96 config-interval=1 ! \
    udpsink host='+str(host)+' port='+str(port)
    fourcc = cv2.VideoWriter_fourcc('H', '2', '6', '4')
    out_send = cv2.VideoWriter(
        txstr, cv2.CAP_GSTREAMER, fourcc, 60, (1280, 720), isColored)
    # TODO: look into why changing resolution and fps on VideoWriter don't seem to work
    # TODO: try different codecs on VideoWriter

    print("Transmitting /dev/video"+str(device)+" to "+host+":"+str(port)+" with "+str(bitrate /
          1e6)+" Mbps target, "+str(fps)+" fps target, ("+str(width)+","+str(height)+") resolution")

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened for /dev/video' + str(device))
        exit(0)

    while True:
        # TODO: loop watchdog?
        if not cap_send.isOpened():
            print('Error opening the video file for /dev/video' +
                  str(device)+'. \nDevice disconnected?')
            exit(0)
        ret, frame = cap_send.read()
        if not ret:
            print('empty frame')
            break
        out_send.write(frame)

        #cv2.imshow('send', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print("stream machine broke")
    cap_send.release()
    out_send.release()


if __name__ == '__main__':
    s1 = Process(target=send, args=(
        0, '10.0.0.7', 5001, 3000000, 1280, 720, 30, True))
    s2 = Process(target=send, args=(
        2, '10.0.0.7', 5002, 3000000, 1280, 720, 30, True))
    s1.start()
    s2.start()
    s1.join()
    s2.join()

    cv2.destroyAllWindows()
