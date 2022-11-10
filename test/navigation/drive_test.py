#!/usr/bin/env python3
import unittest
import numpy as np
from navigation.drive import get_drive_command
from util.SE3 import SE3
from util.SO3 import SO3
from math import pi
from util.np_utils import angle_to_rotate


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return np.array([qx, qy, qz, qw])


class TestDrive(unittest.TestCase):
    def test_drive_straight(self):
        pose = SE3.from_pos_quat([0, 0, 0], get_quaternion_from_euler(0.0, 0.0, 0.0))
        goal_pos = np.array([10.0, 0.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertEqual(cmd.angular.z, 0.0)
        self.assertEqual(cmd.linear.x, 1.0)

    def test_turn_right(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], get_quaternion_from_euler(0.0, 0.0, pi / 2))
        goal_pos = np.array([10.0, 0.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertLess(cmd.angular.z, 0.0)
        self.assertEqual(cmd.angular.z, -1.0)
        self.assertEqual(cmd.linear.x, 0.0)

    def test_turn_left(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], get_quaternion_from_euler(0.0, 0.0, -pi / 2))
        goal_pos = np.array([10.0, 0.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertGreater(cmd.angular.z, 0.0)
        self.assertEqual(cmd.angular.z, 1.0)
        self.assertEqual(cmd.linear.x, 0.0)

    def test_straight_angle(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], get_quaternion_from_euler(0.0, 0.0, pi / 4))
        goal_pos = np.array([10.0, 10.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertLess(abs(cmd.angular.z), 0.001)
        self.assertEqual(cmd.linear.x, 1.0)

    def test_drive_turn(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], get_quaternion_from_euler(0.0, 0.0, pi / 4))
        goal_pos = np.array([10.0, 10.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertLess(abs(cmd.angular.z), 0.001)
        self.assertEqual(cmd.linear.x, 1.0)

    def test_angle_to_rotate(self):
        v1 = np.array([1, 0, 0])
        v2 = np.array([1, 1, 0])
        self.assertAlmostEqual(angle_to_rotate(v1, v2), pi / 4)

        v1 = np.array([0, 1, 0])
        self.assertAlmostEqual(angle_to_rotate(v1, v2), -pi / 4)

        v1 = np.array([-1, -1, 0])
        self.assertAlmostEqual(angle_to_rotate(v1, v2), pi)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "basic_python_test", TestDrive)
