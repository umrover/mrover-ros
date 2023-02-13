# uses openCV with gstreamer support to capture video from v4l2 device and transmit over RTP

import numpy as np
import cv2
from multiprocessing import Process
from pynput import keyboard

def stream_manager(stream_process_list, id, cap_setting):
    if cap_setting:
        print("\nChanging /dev/video" + str(id) + " capture settings to " +str(cap_setting))
        stream_process_list[id].kill()
        stream_process_list[id] = Process(target=send, args=(
        id, '10.0.0.7', 5000+id, 1000000, 854, 480, 30, True))
        stream_process_list[id].start()
        stream_process_list[id].join()
    else:
        print('\nClosing /dev/video' + str(id) + ' stream')
        stream_process_list[id].kill()

def send(device=0, host='10.0.0.7', port=5001, bitrate=4000000, width=1280, height=720, fps=30, isColored=False):
    # Construct video capture pipeline string
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

    # openCV video capture from v4l2 device
    cap_send = cv2.VideoCapture(capstr, cv2.CAP_GSTREAMER)

    # Construct stream transmit pipeline string
    txstr = 'appsrc ! '
    if isColored:
        txstr += ' video/x-raw, format=BGR ! '
    txstr += 'videoconvert ! '
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

    # openCV stream transmit pipeline with RTP sink
    fourcc = cv2.VideoWriter_fourcc('H', '2', '6', '4')
    out_send = cv2.VideoWriter(
        txstr, cv2.CAP_GSTREAMER, fourcc, 60, (1280, 720), isColored)

    # TODO: look into why changing resolution and fps on VideoWriter doesn't seem to work
    # TODO: try different codecs on VideoWriter

    print("Transmitting /dev/video"+str(device)+" to "+host+":"+str(port)+" with "+str(bitrate /
          1e6)+" Mbps target, "+str(fps)+" fps target, ("+str(width)+","+str(height)+") resolution")

    if not cap_send.isOpened() or not out_send.isOpened():
        print('VideoCapture or VideoWriter not opened for /dev/video' + str(device))
        exit(0)

    # Transmit loop
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

        #key = cv2.waitKey(1) & 0xFF
        #print(key)

        # TODO: test functionality when two keys pressed at once
        

    print("stream machine broke")
    cap_send.release()
    out_send.release()

#keyboard keypress event handler
lk = ''
def on_press(key):
    global lk
    if key == keyboard.Key.esc:
        cl = 1
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    #print(k)
    if k in ['0', '1', '2', '3', '4', '5', '6']:
        print("Controlling /dev/video"+k)
    if lk in ['0', '1', '2', '3', '4', '5', '6']:
        if k in ['q']:
            stream_manager(s, int(lk), 0)
        if k in ['l']:
            stream_manager(s, int(lk), 1)
        if k in ['m']:
            stream_manager(s, int(lk), 2)
        if k in ['h']:
            stream_manager(s, int(lk), 3)
    lk = k

if __name__ == '__main__':
    s = [0] * 10
    s[0] = Process(target=send, args=(
        0, '10.0.0.7', 5000, 3000000, 1280, 720, 30, True))
    s[2] = Process(target=send, args=(
        2, '10.0.0.7', 5002, 3000000, 1280, 720, 30, True))
    cthread = keyboard.Listener(on_press=on_press, args=(s))
    s[0].start()
    s[2].start()
    cthread.start()
    print("[Keyboard Control Instructions]\n\
    Press a number to control dev/video[number]\n\
    Press L for low cap settings\n\
    Press M for medium cap settings\n\
    Press H for high cap settings\n\
    Press Q to close selected stream\n\n")
    # TODO: if print statement shows we might be able to use below space to make a loop to watch for termination of a process
    for i in range(len(r)):
        if s[i]:
            s[i].join()

    cthread.join()
    cv2.destroyAllWindows()
