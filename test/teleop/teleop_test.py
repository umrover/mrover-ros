#!/usr/bin/env python3

import unittest
import numpy as np

import rospy as ros
from control_msgs.msg import FollowJointTrajectoryFeedback
from teleop.jetson.arm_trajectory_server import euclidean_error, joint_error

class TestMotionPlan(unittest.TestCase):
    def test_euclidean_error_handling(self):
        feedback = FollowJointTrajectoryFeedback()
        feedback.error = [2, 2, 2, 2, 2]

        self.assertFalse(euclidean_error(threshold=4.5, feedback=feedback))
        self.assertTrue(euclidean_error(threshold=4.4, feedback=feedback))
    
    def test_joint_error_handling(self):
        feedback = FollowJointTrajectoryFeedback()
        feedback.error = [2, 1, 0, 0, 1]
        error_thresholds = [2, 2, 0, 1, 0]

        self.assertTrue(joint_error(error_thresholds, feedback))
        error_thresholds[4] = 1
        self.assertFalse(joint_error(error_thresholds, feedback))


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "basic_python_test", TestMotionPlan)
