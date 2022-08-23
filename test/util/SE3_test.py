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

from util.SE3 import SE3

class TestSE3(unittest.TestCase):
    def test_pos_distance_to(self):
        p1 = SE3(position=np.array([1, 2, 3]))
        p2 = SE3(position=np.array([-4, 8, -7]))
        d1 = p1.pos_distance_to(p2)
        d2 = p2.pos_distance_to(p1)
        self.assertTrue(np.isclose(d1, 12.6886))
        self.assertTrue(np.isclose(d2, 12.6886))

        p1 = SE3()
        p2 = SE3(position=np.array([3, 4, 0]))
        d1 = p1.pos_distance_to(p2)
        d2 = p2.pos_distance_to(p1)
        self.assertTrue(np.isclose(d1, 5))
        self.assertTrue(np.isclose(d2, 5))

        p1 = SE3()
        p2 = SE3()
        d1 = p1.pos_distance_to(p2)
        d2 = p2.pos_distance_to(p1)
        self.assertTrue(np.isclose(d1, 0))
        self.assertTrue(np.isclose(d2, 0))


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SE3_test", TestSE3)
