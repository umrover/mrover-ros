#!/usr/bin/env python3
import os

from multiprocessing import Process

def receive(port = 5001, fpsoverlay = False):
    gst = 'gst-launch-1.0 udpsrc port='+str(port)+' ! \
        \"application/x-rtp, encoding-name=(string)H264, payload=96\" ! \
        rtph264depay ! \
        decodebin ! \
        videoconvert ! '
    if fpsoverlay:
        gst += 'fpsdisplaysink text-overlay=1 video-sink=autovideosink -v'
    else:
        gst += 'autovideosink'
    os.system(gst)
    
if __name__ == '__main__':
    r1 = Process(target=receive, args=(5001, True))
    r2 = Process(target=receive, args=(5002, True))
    r1.start()
    r2.start()
    r1.join()
    r2.join()

    cv2.destroyAllWindows()
