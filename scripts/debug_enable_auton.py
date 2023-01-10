#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import GPSWaypoint, WaypointType, EnableAuton
from util.SE3 import SE3
from util.course_service import EnableService
from mrover.srv import PublishEnableAuton

"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""

if __name__ == "__main__":
    rospy.init_node("debug_enable_auton")
    rospy.wait_for_service("enable_auton")
    try:
        publish_enable = rospy.ServiceProxy("enable_auton", PublishEnableAuton)
        waypoints = [
            GPSWaypoint(42.2, -83.7001, WaypointType(val=WaypointType.NO_SEARCH), 1),
            GPSWaypoint(42.2, -83.7002, WaypointType(val=WaypointType.NO_SEARCH), 1),
        ]
        msg = EnableAuton(waypoints, True)
        publish_enable(msg)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    rospy.spin()
