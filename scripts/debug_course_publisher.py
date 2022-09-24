#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import Waypoint
from util.SE3 import SE3
from util.course_service import CourseService

"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""

if __name__ == "__main__":
    rospy.init_node("debug_course_publisher")
    publish_course = CourseService()

    waypoints = [
        (Waypoint(fiducial_id=0, tf_id="course0"), SE3(position=np.array([3, 3, 0]))),
        # (Waypoint(fiducial_id=0, tf_id="course1"), SE3(position=np.array([-3, -3, -1]))),
        # (Waypoint(fiducial_id=0, tf_id="course2"), SE3(position=np.array([-5, -5, 0]))),
    ]

    publish_course(waypoints)

    rospy.spin()
