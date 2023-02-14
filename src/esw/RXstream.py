#!/usr/bin/env python3
import os
from multiprocessing import Process
from pynput import keyboard


def stream_manager(stream_process_list, id):
    print("\nRestarting receiver listening on port 500" + str(id))
    if stream_process_list[id]:
        stream_process_list[id].kill()
    stream_process_list[id] = Process(target=receive, args=(5000+id, True))
    stream_process_list[id].start()


def receive(port=5001, fpsoverlay=False):

    # Construct stream receive pipeline string
    rxstr = 'gst-launch-1.0 udpsrc port='+str(port)+' ! \
        \"application/x-rtp, encoding-name=(string)H264, payload=96\" ! \
        rtph264depay ! \
        decodebin ! \
        videoconvert ! '
    if fpsoverlay:
        rxstr += 'fpsdisplaysink text-overlay=1 video-sink=autovideosink -v'
    else:
        rxstr += 'autovideosink'
    os.system(rxstr)


def on_press(key):  # keyboard keypress event handler
    global lk
    if key == keyboard.Key.esc:
        cl = 1
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    numlist = [str(n) for n in list(range(10))]
    if k in numlist:
        stream_manager(r, int(k))


if __name__ == '__main__':
    fol = True
    r = [0] * 10
    r[0] = Process(target=receive, args=(5000, fol))
    r[2] = Process(target=receive, args=(5002, fol))
    cthread = keyboard.Listener(on_press=on_press, args=(r))
    r[0].start()
    r[2].start()
    cthread.start()
    print("[KEYBOARD CONTROL INSTRUCTIONS]\n\
    Press a number to restart receiver listening on port "+str(5000)+"+[number].\n\n")

    for i in range(len(r)):
        if r[i]:
            r[i].join()
    cthread.join()
