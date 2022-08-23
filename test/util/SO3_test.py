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

from util.SO3 import SO3


class TestSO3(unittest.TestCase):
    def test_init(self):
        r1 = SO3()
        self.assertTrue(np.array_equal(r1.quaternion, np.array([0, 0, 0, 1])))
        
        r2 = SO3(quaternion=np.array([0, 1, 0, 0]))
        self.assertTrue(np.array_equal(r2.quaternion, np.array([0, 1, 0, 0])))
        
    def test_from_matrix(self):
        r1 = SO3.from_matrix(np.eye(3))
        self.assertTrue(np.array_equal(r1.quaternion, np.array([0, 0, 0, 1])))

        matrix = np.array([[-1, 0, 0],
                           [0, 1, 0],
                           [0, 0, -1]])
        r2 = SO3.from_matrix(matrix)
        self.assertTrue(np.array_equal(r2.quaternion, np.array([0, 1, 0, 0])))
        
        
    def test_rotation_matrix(self):
        ...
        
    def test_direction_vector(self):
        ...
        
    def test_rot_distance_to(self):
        ...


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SO3_test", TestSO3)
