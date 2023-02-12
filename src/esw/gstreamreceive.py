#!/usr/bin/env python3
import os
'''
from mrover.msg import CameraCmd
from typing import List, Dict, Tuple

import threading

import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)

import jetson.utils
'''


def main():
    os.system("gst-launch-1.0 udpsrc port=5001 ! \"application/x-rtp, encoding-name=(string)H264, payload=96\" ! rtph264depay ! decodebin ! videoconvert ! autovideosink")
    # rospy.init_node("cameras")
    #streaming_manager = StreamingManager()
    # rospy.Service("change_cameras", ChangeCameras,
    #              streaming_manager.handle_change_cameras)
    # while not rospy.is_shutdown():
    #    streaming_manager.update_all_streams()


if __name__ == "__main__":
    main()
