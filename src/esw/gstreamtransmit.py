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
    os.system("gst-launch-1.0 v4l2src device=/dev/video0 do-timestamp=true ! image/jpeg, width=1280, height=720 ! jpegdec ! videorate ! video/x-raw, framerate=30/1 ! nvvidconv ! \"video/x-raw(memory: NVMM), width=1280, height=720\" ! nvv4l2h264enc bitrate=4000000 ! video/x-h264 ! rtph264pay config-interval=1 ! udpsink host=10.0.0.7 port=5001 auto-multicast=true")
    # rospy.init_node("cameras")
    #streaming_manager = StreamingManager()
    # rospy.Service("change_cameras", ChangeCameras,
    #              streaming_manager.handle_change_cameras)
    # while not rospy.is_shutdown():
    #    streaming_manager.update_all_streams()


if __name__ == "__main__":
    main()
