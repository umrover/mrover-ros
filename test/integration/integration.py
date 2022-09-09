#!/usr/bin/env python3
import sys
import unittest

import numpy as np

import rospy
import rostest
import tf2_ros
from mrover.msg import Waypoint
from util.SE3 import SE3
from util.course_service import CourseService


class TestIntegration(unittest.TestCase):
    def test_integration(self):
        rospy.logdebug("Integration Test Starting")

        rospy.sleep(5.0)

        rospy.init_node("integration_test")
        publish_course = CourseService()

        rospy.loginfo("Integration Test Ready")

        publish_course([
            (Waypoint(fiducial_id=0, tf_id="course0"), SE3(position=np.array([-5.5, -5.5, 0.0]))),
        ])

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        while not rospy.is_shutdown():
            rover_pose = SE3.from_tf_tree(tf_buffer, "odom", "base_link")
            rospy.loginfo(rover_pose.position)


if __name__ == "__main__":
    rostest.rosrun("mrover", "integration_test", TestIntegration, sys.argv)
