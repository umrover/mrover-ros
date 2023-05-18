#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import Waypoint, WaypointType
from util.SE3 import SE3
from util.course_publish_helpers import convert_waypoint_to_gps, publish_waypoints


"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""


if __name__ == "__main__":
    rospy.init_node("debug_course_publisher")

    waypoints = [
        # ("course1", -2, -6, 0),
        # ("course1", -3, -3, -1),
        # ("course2", -5, -5, 0)
        (
            Waypoint(fiducial_id=0, tf_id="course0", type=WaypointType(val=WaypointType.NO_SEARCH)),
            # SE3(position=np.array([-844.37, 10351.56, 0])),
            SE3(position=np.array([-8, -8, 0])),
        ),
        # (Waypoint(fiducial_id=0, tf_id="course1"), SE3(position=np.array([-3, -3, -1]))),
        # (Waypoint(fiducial_id=0, tf_id="course2"), SE3(position=np.array([-5, -5, 0]))),
    ]

    publish_waypoints([convert_waypoint_to_gps(waypoint) for waypoint in waypoints])
