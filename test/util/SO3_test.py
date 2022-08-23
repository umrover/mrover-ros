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
        self.assertTrue(np.allclose(r1.quaternion, np.array([0, 0, 0, 1])))

        r2 = SO3(quaternion=np.array([0, 1, 0, 0]))
        self.assertTrue(np.allclose(r2.quaternion, np.array([0, 1, 0, 0])))

    def test_from_matrix(self):
        r1 = SO3.from_matrix(np.eye(3))
        self.assertTrue(np.allclose(r1.quaternion, np.array([0, 0, 0, 1])))

        matrix = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        r2 = SO3.from_matrix(matrix)
        self.assertTrue(np.allclose(r2.quaternion, np.array([0, 1, 0, 0])))

    def test_rotation_matrix(self):
        r1 = SO3()
        self.assertTrue(np.allclose(r1.rotation_matrix(), np.eye(3)))

        r2 = SO3(quaternion=np.array([0.5, 0.5, 0.5, 0.5]))
        matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        self.assertTrue(np.allclose(r2.rotation_matrix(), matrix))

    def test_direction_vector(self):
        r1 = SO3()
        dir = np.array([1, 0, 0])
        self.assertTrue(np.allclose(r1.direction_vector(), dir))

        r2 = SO3(quaternion=np.array([0, 0, 0.3826834, 0.9238795]))
        dir = np.array([2 ** (1 / 2) / 2, 2 ** (1 / 2) / 2, 0])
        self.assertTrue(np.allclose(r2.direction_vector(), dir))

        r3 = SO3(quaternion=np.array([0.5, -0.5, 0.5, 0.5]))
        dir = np.array([0, 0, 1])
        self.assertTrue(np.allclose(r3.direction_vector(), dir))

    def test_rot_distance_to(self):
        r1 = SO3()
        r2 = SO3()
        self.assertTrue(np.isclose(r1.rot_distance_to(r1), 0))
        self.assertTrue(np.isclose(r2.rot_distance_to(r2), 0))
        self.assertTrue(np.isclose(r1.rot_distance_to(r2), 0))
        self.assertTrue(np.isclose(r2.rot_distance_to(r1), 0))
        
        r3 = SO3(quaternion=np.array([0, 0, 2**(1/2)/2, 2**(1/2)/2]))
        self.assertTrue(np.isclose(r3.rot_distance_to(r3), 0))
        self.assertTrue(np.isclose(r1.rot_distance_to(r3), np.pi/2))
        self.assertTrue(np.isclose(r3.rot_distance_to(r1), np.pi/2))

        r4 = SO3(quaternion=np.array([0, 0, -2**(1/2)/2, 2**(1/2)/2]))
        self.assertTrue(np.isclose(r4.rot_distance_to(r4), 0))
        self.assertTrue(np.isclose(r1.rot_distance_to(r4), np.pi/2))
        self.assertTrue(np.isclose(r4.rot_distance_to(r1), np.pi/2))
        self.assertTrue(np.isclose(r3.rot_distance_to(r4), np.pi))
        self.assertTrue(np.isclose(r4.rot_distance_to(r3), np.pi))


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SO3_test", TestSO3)
