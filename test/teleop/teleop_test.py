#!/usr/bin/env python3

import unittest
import numpy as np

import rospy as ros
from control_msgs.msg import FollowJointTrajectoryFeedback
from teleop.jetson.arm_trajectory_server import euclidean_error, joint_error


class TestMotionPlan(unittest.TestCase):
    def test_euclidean_error_handling(self):
        feedback = FollowJointTrajectoryFeedback()
        feedback.desired.positions = [6, 6, 8, 9, 4]
        feedback.actual.positions = [4, 4, 6, 7, 2]
        feedback.error.positions = [2, 2, 2, 2, 2]

        self.assertFalse(euclidean_error(threshold=4.5, feedback=feedback)[0])
        self.assertTrue(euclidean_error(threshold=4.4, feedback=feedback)[0])

    def test_joint_error_handling(self):
        feedback = FollowJointTrajectoryFeedback()
        feedback.desired.positions = [5, 9, 7, 2, 6]
        feedback.actual.positions = [3, 8, 7, 2, 5]
        feedback.error.positions = [2, 1, 0, 0, 1]
        error_thresholds = [2, 2, 0, 1, 0]

        self.assertTrue(joint_error(error_thresholds, feedback)[0])
        error_thresholds[4] = 1
        self.assertFalse(joint_error(error_thresholds, feedback)[0])


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "basic_python_test", TestMotionPlan)
