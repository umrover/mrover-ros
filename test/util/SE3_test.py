#!/usr/bin/env python3
# NOTE: the above line is very important
# also make sure to chmod +x any python test files

"""
There are three ways to run this file
- We can launch it as part of the basic_integration-test
with rostest mrover basic_integration test
- We can run it as a unit test with ./python_test.py
- We can run a catkin test which runs all tests in the CMakeLists.txt
"""

import unittest
import numpy as np
import rospy
import tf2_ros

from util.SE3 import SE3


class TestSE3(unittest.TestCase):
    def test_init(self):

        # test default init values
        p1 = SE3()
        self.assertTrue(np.array_equal(p1.position, np.zeros(3)))
        self.assertTrue(np.array_equal(p1.rotation.quaternion, np.array([0, 0, 0, 1])))

        # test init values passed to constructor
        p2 = SE3(position=np.array([1, 2, 3]), rotation=np.array([0.247404, 0, 0, 0.9689124]))
        self.assertTrue(np.array_equal(p2.position, np.array([1, 2, 3])))
        self.assertTrue(np.array_equal(p2.rotation.quaternion, np.array([0.247404, 0, 0, 0.9689124])))

    def test_pos_distance_to(self):
        
        # distance between 2 "random" points
        p1 = SE3(position=np.array([1, 2, 3]))
        p2 = SE3(position=np.array([-4, 8, -7]))
        d1 = p1.pos_distance_to(p2)
        d2 = p2.pos_distance_to(p1)
        d3 = p1.pos_distance_to(p1)
        d4 = p2.pos_distance_to(p2)
        self.assertTrue(np.isclose(d1, 12.6886))
        self.assertTrue(np.isclose(d2, 12.6886))
        self.assertTrue(np.isclose(d3, 0))
        self.assertTrue(np.isclose(d4, 0))

        # distance between origin and a "random" point
        p1 = SE3()
        p2 = SE3(position=np.array([3, 4, 0]))
        d1 = p1.pos_distance_to(p2)
        d2 = p2.pos_distance_to(p1)
        self.assertTrue(np.isclose(d1, 5))
        self.assertTrue(np.isclose(d2, 5))

        # distance between two identical origin points
        p1 = SE3()
        p2 = SE3()
        d1 = p1.pos_distance_to(p2)
        d2 = p2.pos_distance_to(p1)
        self.assertTrue(np.isclose(d1, 0))
        self.assertTrue(np.isclose(d2, 0))
    
if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SE3_test", TestSE3)
