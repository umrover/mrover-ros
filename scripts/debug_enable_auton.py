#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import GPSWaypoint, WaypointType, EnableAuton
from util.SE3 import SE3
from util.course_service import EnableService

"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""

if __name__ == "__main__":
    rospy.init_node("debug_enable_auton")
    publish_enable = EnableService()
    waypoints = [
        GPSWaypoint(42.2, -83.7001, WaypointType(val=WaypointType.NO_SEARCH), 1),
        GPSWaypoint(42.2, -83.7002, WaypointType(val=WaypointType.NO_SEARCH), 1),
    ]
    msg = EnableAuton(waypoints, True)
    publish_enable(msg)

    rospy.spin()
