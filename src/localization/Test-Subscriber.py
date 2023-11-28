#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import tf2_ros
import math

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3
from util.SO3 import SO3


class Localization:
    pose: SE3

    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        # TODO
        rospy.Subscriber("gps/fix_gps_left", NavSatFix, self.gps_left_callback)
        rospy.Subscriber("gps/fix_gps_right", NavSatFix, self.gps_right_callback)

        # create a transform broadcaster for publishing to the TF tree
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # initialize pose to all zeros
        # self.pose = SE3()

    def gps_left_callback(self, msg: NavSatFix):
        print("GPS left callback")
        print(msg)

    def gps_right_callback(self, msg: NavSatFix):
        print("GPS right callback")
        print(msg)


def main():
    # initialize the node
    rospy.init_node("localization_test")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
