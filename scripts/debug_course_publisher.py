#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import Waypoint, WaypointType
from util.SE3 import SE3
from util.course_publish_helpers import convert_waypoint_to_gps, publish_waypoints

"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without tags and these will get published to nav
"""

if __name__ == "__main__":
    rospy.init_node("debug_course_publisher")

    publish_waypoints(
        [
            convert_waypoint_to_gps(waypoint)
            for waypoint in [
                # (
                #     Waypoint(tag_id=0, type=WaypointType(val=WaypointType.NO_SEARCH)),
                #     SE3(position=np.array([4, 4, 0])),
                # ),
                (
                    Waypoint(tag_id=10, type=WaypointType(val=WaypointType.POST)),
                    SE3(position=np.array([1, 1, 0])),
                ),
                # (
                #     Waypoint(tag_id=8, type=WaypointType(val=WaypointType.POST)),
                #     SE3(position=np.array([11, 30, 0])),
                # ),
            ]
        ]
    )
