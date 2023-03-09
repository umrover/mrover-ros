#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import GPSWaypoint, WaypointType
from util.course_publish_helpers import publish_waypoints


"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""


if __name__ == "__main__":
    rospy.init_node("debug_enable_auton")
    publish_waypoints(
        [
            GPSWaypoint(42.2, -83.7001, WaypointType(val=WaypointType.NO_SEARCH), 1),
            GPSWaypoint(42.2, -83.7002, WaypointType(val=WaypointType.NO_SEARCH), 1),
        ]
    )
