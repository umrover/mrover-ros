#!/usr/bin/env python3
import unittest
import numpy as np
from navigation.drive import get_drive_command
from util.SE3 import SE3
from util.SO3 import SO3
from math import pi
from util.np_utils import angle_to_rotate
from navigation.gate import GateTrajectory
from tf.transformations import quaternion_from_euler
import rospy


APPROACH_DISTANCE = 2.0


class TestDrive(unittest.TestCase):
    def test_drive_straight(self):
        pose = SE3.from_pos_quat([0, 0, 0], quaternion_from_euler(0.0, 0.0, 0.0))
        goal_pos = np.array([10.0, 0.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertEqual(cmd.angular.z, 0.0)
        self.assertEqual(cmd.linear.x, 1.0)

    def test_turn_right(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], quaternion_from_euler(0.0, 0.0, pi / 2))
        goal_pos = np.array([10.0, 0.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertLess(cmd.angular.z, 0.0)
        self.assertEqual(cmd.angular.z, -1.0)
        self.assertEqual(cmd.linear.x, 0.0)

    def test_turn_left(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], quaternion_from_euler(0.0, 0.0, -pi / 2))
        goal_pos = np.array([10.0, 0.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertGreater(cmd.angular.z, 0.0)
        self.assertEqual(cmd.angular.z, 1.0)
        self.assertEqual(cmd.linear.x, 0.0)

    def test_straight_angle(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], quaternion_from_euler(0.0, 0.0, pi / 4))
        goal_pos = np.array([10.0, 10.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertLess(abs(cmd.angular.z), 0.001)
        self.assertEqual(cmd.linear.x, 1.0)

    def test_drive_turn(self):
        # rover is facing along the y axis
        pose = SE3.from_pos_quat([0, 0, 0], quaternion_from_euler(0.0, 0.0, pi / 4))
        goal_pos = np.array([10.0, 10.0, 0.0])
        cmd, done = get_drive_command(goal_pos, pose, 2.0, 0.01)
        print(cmd)
        print(done)
        self.assertLess(abs(cmd.angular.z), 0.001)
        self.assertEqual(cmd.linear.x, 1.0)

    # def test_angle_to_rotate(self):
    #     v1 = np.array([1, 0, 0])
    #     v2 = np.array([1, 1, 0])
    #     self.assertAlmostEqual(angle_to_rotate(v1, v2), pi / 4)

    #     v1 = np.array([0, 1, 0])
    #     self.assertAlmostEqual(angle_to_rotate(v1, v2), -pi / 4)

    #     v1 = np.array([-1, -1, 0])
    #     self.assertAlmostEqual(angle_to_rotate(v1, v2), pi)
    def test_make_path(self):
        print("Hello2")
        post1 = np.array([1, 2, 0])
        post2 = np.array([-1, 2, 0])
        gate = (post1, post2)

        x = 1
        self.assertTrue(False)
        """
        #Test 1 (Shortest path should be selected)
        The path should contain the center, and the victory point
        """
        rover = np.array([0, 1, 0])
        self.traj = GateTrajectory.spider_gate_trajectory(APPROACH_DISTANCE, gate, rover)
        checkCoord = np.array([0, 2, 0], [0, 4, 0])
        self.assertTrue(self.traj.coordinates, checkCoord)

        """
        #Test 2 
        The path should contain, the closest approach point, the center, and the victory point
        """
        rover = np.array([1, -0.5, 0])
        self.traj = GateTrajectory.spider_gate_trajectory(APPROACH_DISTANCE, gate, rover)
        checkCoord = np.array([0, 0, 0], [0, 2, 0], [0, 4, 0])
        self.assertTrue(self.traj.coordinates, checkCoord)

        """
        #Test 3 
        The path should contain all the points. This should be the longest path.
        The path has the closest prep point, closest approach point, the center,
        and the victory point. 
        """
        rover = np.array([1, -3, 0])
        self.traj = GateTrajectory.spider_gate_trajectory(APPROACH_DISTANCE, gate, rover)
        rospy.loginfo("Test 3: ")
        checkCoord = np.array([1, -2, 0], [0, 0, 0], [0, 2, 0], [0, 4, 0])
        self.assertTrue(self.traj.coordinates, checkCoord)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "basic_python_test", TestDrive)
