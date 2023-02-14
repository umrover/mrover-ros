#!/usr/bin/env python3
import os
from multiprocessing import Process

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

if __name__ == '__main__':
    fol = True
    r = [0] * 10
    for i in range(10):
        r[i] = Process(target=receive, args=(5000+i, fol))    
        r[i].start()
    #r[2] = Process(target=receive, args=(5002, fol))
    #cthread = keyboard.Listener(on_press=on_press, args=(r))
    #cthread.start()
    print("[KEYBOARD CONTROL INSTRUCTIONS]\n\
    Press a number to restart receiver listening on port "+str(5000)+"+[number].\n\n")

    for i in range(len(r)):
        if r[i]:
            r[i].join()
    #cthread.join()
