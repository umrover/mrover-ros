#!/usr/bin/env python3

import unittest

import numpy as np

import rospy
from mrover.msg import Waypoint
from util.SE3 import SE3
from util.course_service import CourseService


class TestIntegration(unittest.TestCase):
    def test_integration(self):
        rospy.init_node("integration_tester")
        publish_course = CourseService("integration_course_service")

        waypoints = [
            (Waypoint(fiducial_id=0, tf_id="course0"), SE3(position=np.ndarray([-5.5, -5.5, 0.0]))),
        ]

        publish_course(waypoints)

        rospy.spin()


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "integration_test", TestIntegration)
