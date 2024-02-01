#!/usr/bin/env python3
import os
import struct

import cv2
import numpy as np
import ros_numpy
import rospy
import sys

import cv2 as cv
from cv_bridge import CvBridge

from mrover.srv import CapturePanorama, CapturePanoramaRequest, CapturePanoramaResponse
from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs import point_cloud2


def capture_panorama(request: CapturePanoramaRequest) -> CapturePanoramaResponse:

    opencv_matrix = []
    def received_point_cloud(point: PointCloud2):
        rospy.loginfo("Received point cloud data")
        #Extract RGB field

        pc=ros_numpy.numpify(point)
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        shape = pc.shape + (3, )
        points = np.zeros(shape)
        points[..., 0] = pc['x']
        points[..., 1] = pc['y']
        points[..., 2] = pc['z']
        rgb = np.zeros(shape)
        rgb[..., 0] = pc['r']
        rgb[..., 1] = pc['g']
        rgb[..., 2] = pc['b']
        opencv_matrix.append(cv2.cvtColor((points * 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
        rospy.loginfo("Converted and Saved Image")
        rospy.loginfo("Total Images Saved" + str(len(opencv_matrix)))

    image_subscriber = rospy.Subscriber("/camera/left/points", PointCloud2,received_point_cloud, queue_size=1)

    #TODO, make motors spin around
    if(len(opencv_matrix) < 2):
        rospy.sleep(1)
    rospy.loginfo("Creating Panorama..." + str(len(opencv_matrix)))
    stitcher = cv2.Stitcher.create()
    (code,outPic) = stitcher.stitch(opencv_matrix)
    rospy.loginfo("Panorama Created" + str(len(opencv_matrix)))
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(outPic, encoding="bgr8")
    rospy.loginfo("Sending Response" + str(len(opencv_matrix)))
    return CapturePanoramaResponse(image_msg)

def main() -> int:
    rospy.init_node("panorama")
    panorama_service = rospy.Service(
        "capture_panorama",
        CapturePanorama,
        capture_panorama
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
