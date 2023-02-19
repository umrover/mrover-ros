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
from util.SO3 import SO3


class TestSE3(unittest.TestCase):
    def test_init(self):

        # test default init values
        p1 = SE3()
        self.assertTrue(np.array_equal(p1.position, np.zeros(3)))
        self.assertTrue(np.array_equal(p1.rotation.quaternion, np.array([0, 0, 0, 1])))

        # test init value passed to constructor
        p2 = SE3(
            position=np.array([1, 2, 3]),
            rotation=SO3(quaternion=np.array([1, 2, 3, 4])),
        )
        self.assertTrue(np.array_equal(p2.position, np.array([1, 2, 3])))
        self.assertTrue(np.array_equal(p2.rotation.quaternion, np.array([1, 2, 3, 4])))

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

    def test_is_approx(self):

        # test two identical zero SE3s in all directions and combinations
        r1 = SE3()
        r2 = SE3()
        self.assertTrue(r1.is_approx(r1))
        self.assertTrue(r2.is_approx(r2))
        self.assertTrue(r1.is_approx(r2))
        self.assertTrue(r2.is_approx(r1))

        # test two non zero SE3s that are within the default tolerance
        r3 = SE3.from_pos_quat(position=np.array([1, 2, 3]), quaternion=np.array([1, 2, 3, 4]))
        r4 = SE3.from_pos_quat(
            position=np.array([1.000000008, 2, 3]),
            quaternion=np.array([1.000000008, 2, 3, 4]),
        )
        self.assertTrue(r3.is_approx(r3))
        self.assertTrue(r4.is_approx(r4))
        self.assertTrue(r3.is_approx(r4))
        self.assertTrue(r4.is_approx(r3))
        self.assertFalse(r1.is_approx(r3))
        self.assertFalse(r3.is_approx(r1))
        self.assertFalse(r1.is_approx(r4))
        self.assertFalse(r4.is_approx(r1))

        # test two SE3s that are within a tolerance of 1e-3, not within the default tolerance
        r5 = SE3.from_pos_quat(position=np.array([1.0004, 2, 3]), quaternion=np.array([1.0005, 2, 3, 4]))
        self.assertTrue(r3.is_approx(r5, tolerance=1e-3))
        self.assertTrue(r5.is_approx(r3, tolerance=1e-3))
        self.assertFalse(r3.is_approx(r5))
        self.assertFalse(r5.is_approx(r3))

        # test two SE3s that have equal position but not rotation
        r6 = SE3.from_pos_quat(position=np.array([1, 2, 3]), quaternion=np.array([2, 2, 3, 4]))
        self.assertFalse(r6.rotation.is_approx(r3.rotation))
        self.assertFalse(r3.rotation.is_approx(r6.rotation))
        self.assertFalse(r3.is_approx(r6))
        self.assertFalse(r6.is_approx(r3))

        # test two SE3s that have equal rotation but not position
        r7 = SE3.from_pos_quat(position=np.array([7, 2, 3]), quaternion=np.array([1, 2, 3, 4]))
        self.assertTrue(r7.rotation.is_approx(r3.rotation))
        self.assertTrue(r3.rotation.is_approx(r7.rotation))
        self.assertFalse(r3.is_approx(r7))
        self.assertFalse(r7.is_approx(r3))

    def test_transform_matrix(self):
        # test that an SE3 from a transform matrix is the same as an SE3
        # from an equivalent quaternion and position
        r = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        t = np.array([1, 2, 3])
        m = np.eye(4)
        m[:3, :3] = r
        m[:3, 3] = t
        s1 = SE3.from_transform_matrix(m)

        q = np.array([0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2])
        s2 = SE3.from_pos_quat(position=t, quaternion=q)
        self.assertTrue(np.allclose(t, s1.position))
        self.assertTrue(np.allclose(r, s1.rotation.rotation_matrix()))
        self.assertTrue(np.allclose(m, s1.transform_matrix()))
        self.assertTrue(s1.is_approx(s2))
        self.assertTrue(np.allclose(s1.transform_matrix(), s2.transform_matrix()))


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SE3_test", TestSE3)
