#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import EnableAuton
from mrover.srv import PublishEnableAuton
from util.course_publish_helpers import publish_waypoints


"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""


if __name__ == "__main__":
    rospy.init_node("debug_disable_auton")
    rospy.wait_for_service("enable_auton")
    try:
        publish_enable = rospy.ServiceProxy("enable_auton", PublishEnableAuton)
        msg = EnableAuton([], False)
        publish_enable(msg)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
