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

        # test default init values
        r1 = SO3()
        self.assertTrue(np.allclose(r1.quaternion, np.array([0, 0, 0, 1])))

        # test init values passed to constructor
        r2 = SO3(quaternion=np.array([0, 1, 0, 0]))
        self.assertTrue(np.allclose(r2.quaternion, np.array([0, 1, 0, 0])))

    def test_from_matrix(self):

        # test identity matrix
        r1 = SO3.from_matrix(np.eye(3))
        self.assertTrue(np.allclose(r1.quaternion, np.array([0, 0, 0, 1])))

        # test "random" matrix
        matrix = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        r2 = SO3.from_matrix(matrix)
        self.assertTrue(np.allclose(r2.quaternion, np.array([0, 1, 0, 0])))

    def test_rotation_matrix(self):

        # test returning an identity matrix
        r1 = SO3()
        self.assertTrue(np.allclose(r1.rotation_matrix(), np.eye(3)))

        # test returning a "random" matrix
        r2 = SO3(quaternion=np.array([0.5, 0.5, 0.5, 0.5]))
        matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        self.assertTrue(np.allclose(r2.rotation_matrix(), matrix))

    def test_direction_vector(self):

        # test standard direction
        r1 = SO3()
        dir = np.array([1, 0, 0])
        self.assertTrue(np.allclose(r1.direction_vector(), dir))

        # test "random" direction
        r2 = SO3(quaternion=np.array([0, 0, 0.3826834, 0.9238795]))
        dir = np.array([2 ** (1 / 2) / 2, 2 ** (1 / 2) / 2, 0])
        self.assertTrue(np.allclose(r2.direction_vector(), dir))

        # test another "random" direction
        r3 = SO3(quaternion=np.array([0.5, -0.5, 0.5, 0.5]))
        dir = np.array([0, 0, 1])
        self.assertTrue(np.allclose(r3.direction_vector(), dir))

    def test_rot_distance_to(self):

        # make sure distances between the same SE3 and identical SE3s are the same, in both directions
        r1 = SO3()
        r2 = SO3()
        self.assertTrue(np.isclose(r1.rot_distance_to(r1), 0))
        self.assertTrue(np.isclose(r2.rot_distance_to(r2), 0))
        self.assertTrue(np.isclose(r1.rot_distance_to(r2), 0))
        self.assertTrue(np.isclose(r2.rot_distance_to(r1), 0))

        # test a rotation 90 degrees around the z axis
        r3 = SO3(quaternion=np.array([0, 0, 2 ** (1 / 2) / 2, 2 ** (1 / 2) / 2]))
        self.assertTrue(np.isclose(r3.rot_distance_to(r3), 0))
        self.assertTrue(np.isclose(r1.rot_distance_to(r3), np.pi / 2))
        self.assertTrue(np.isclose(r3.rot_distance_to(r1), np.pi / 2))

        # test a rotation -90 degrees around the z axis
        r4 = SO3(quaternion=np.array([0, 0, -(2 ** (1 / 2)) / 2, 2 ** (1 / 2) / 2]))
        self.assertTrue(np.isclose(r4.rot_distance_to(r4), 0))
        self.assertTrue(np.isclose(r1.rot_distance_to(r4), np.pi / 2))
        self.assertTrue(np.isclose(r4.rot_distance_to(r1), np.pi / 2))
        self.assertTrue(np.isclose(r3.rot_distance_to(r4), np.pi))
        self.assertTrue(np.isclose(r4.rot_distance_to(r3), np.pi))

        # test a rotation 270 degrees around the z axis
        r5 = SO3(quaternion=np.array([0, 0, (2 ** (1 / 2)) / 2, -(2 ** (1 / 2)) / 2]))
        self.assertTrue(np.isclose(r1.rot_distance_to(r5), np.pi / 2))
        self.assertTrue(np.isclose(r5.rot_distance_to(r1), np.pi / 2))

        # 45 degrees around the x axis, then 90 degrees around the y axis
        r6 = SO3(quaternion=np.array([0.2705981, 0.6532815, 0.2705981, 0.6532815]))
        self.assertTrue(np.isclose(r6.rot_distance_to(r4), 2.5935642935144156))
        self.assertTrue(np.isclose(r4.rot_distance_to(r6), 2.5935642935144156))

    def test_is_approx(self):

        # test two identical zero rotations in all directions and combinations
        r1 = SO3()
        r2 = SO3()
        self.assertTrue(r1.is_approx(r1))
        self.assertTrue(r2.is_approx(r2))
        self.assertTrue(r1.is_approx(r2))
        self.assertTrue(r2.is_approx(r1))

        # test two non zero rotations that are within the default tolerance
        r3 = SO3(quaternion=np.array([1, 2, 3, 4]))
        r4 = SO3(quaternion=np.array([1.000000008, 2, 3, 4]))
        self.assertTrue(r3.is_approx(r3))
        self.assertTrue(r4.is_approx(r4))
        self.assertTrue(r3.is_approx(r4))
        self.assertTrue(r4.is_approx(r3))
        self.assertFalse(r1.is_approx(r3))
        self.assertFalse(r3.is_approx(r1))
        self.assertFalse(r1.is_approx(r4))
        self.assertFalse(r4.is_approx(r1))

        # test two rotations that are within a tolerance of 1e-3, not within the default tolerance
        r5 = SO3(quaternion=np.array([1.0005, 2, 3, 4]))
        self.assertTrue(r3.is_approx(r5, tolerance=1e-3))
        self.assertTrue(r5.is_approx(r3, tolerance=1e-3))
        self.assertFalse(r3.is_approx(r5))
        self.assertFalse(r5.is_approx(r3))


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SO3_test", TestSO3)
