#!/usr/bin/env python3

from mrover.msg import CameraCmd
from typing import List, Dict, Tuple

import threading

import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)

primary_cameras = [CameraCmd(), CameraCmd(), CameraCmd(), CameraCmd()]
secondary_cameras = [CameraCmd(), CameraCmd(), CameraCmd(), CameraCmd()]

def handle_change_cameras(req: ChangeCamerasRequest) -> ChangeCamerasResponse:

    global primary_cameras, secondary_cameras

    if req.primary:
        primary_cameras = req.camera_cmds
    else:
        secondary_cameras = req.camera_cmds

    response = ChangeCamerasResponse(primary_cameras, secondary_cameras)

    return response

def main():
    rospy.init_node("cameras")
    rospy.Service("change_cameras", ChangeCameras, handle_change_cameras)
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
