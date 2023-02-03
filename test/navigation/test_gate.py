#!/usr/bin/env python3
import unittest
import numpy as np
from util.SE3 import SE3
from navigation.context import Gate
from navigation.gate import GateTrajectory
import rospy


APPROACH_DISTANCE = 2.0


class GateTest(unittest.TestCase):
    def test_make_path(self):
        print("Hello2")
        post1 = np.array([1, 2])
        post2 = np.array([-1, 2])
        gate = Gate(post1, post2)

        """
        #Test 1 (Shortest path should be selected)
        The path should contain the center, and the victory point
        """
        rover = np.array([0, 1, 0])
        self.traj = GateTrajectory.spider_gate_trajectory(APPROACH_DISTANCE, gate, rover)
        checkCoord = np.array([[0, 2, 0], [0, 4, 0]])
        self.assertTrue(np.allclose(self.traj.coordinates, checkCoord))

        """
        #Test 2 
        The path should contain, the closest approach point, the center, and the victory point
        """
        rover = np.array([1, -0.5, 0])
        self.traj = GateTrajectory.spider_gate_trajectory(APPROACH_DISTANCE, gate, rover)
        checkCoord = np.array([[0, 0, 0], [0, 2, 0], [0, 4, 0]])
        self.assertTrue(np.allclose(self.traj.coordinates, checkCoord))

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
        self.assertTrue(np.allclose(self.traj.coordinates, checkCoord))


if __name__ == "__main__":
    import rostest

    print("hello")
    rostest.rosrun("mrover", "Testing Gate", GateTest)
