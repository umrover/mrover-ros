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
    def test_from_tf_tree(self):

        # setup testing node and TF infrastructure
        rospy.init_node("SE3_tf_test")
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)

        # wait for TFs to become available
        rospy.sleep(1)

        # test TF from one frame to itself
        p1 = SE3.from_tf_tree(buffer, parent_frame="map", child_frame="map")
        position = np.zeros(3)
        quaternion = np.array([0, 0, 0, 1])
        self.assertTrue(np.allclose(p1.position, position))
        self.assertTrue(np.allclose(p1.rotation.quaternion, quaternion))

        # test TF that was directly published
        p2 = SE3.from_tf_tree(buffer, parent_frame="map", child_frame="odom")
        position = np.array([1, 0, 0])
        quaternion = np.array([0, 0, 0, 1])
        self.assertTrue(np.allclose(p2.position, position))
        self.assertTrue(np.allclose(p2.rotation.quaternion, quaternion))

        # test TF that was directly published
        p3 = SE3.from_tf_tree(buffer, parent_frame="odom", child_frame="base_link")
        position = np.array([0, 2, -3])
        quaternion = np.array([0, 0, 1, 0])
        self.assertTrue(np.allclose(p3.position, position))
        self.assertTrue(np.allclose(p3.rotation.quaternion, quaternion))

        # test combined TF from frames that were published separately
        p4 = SE3.from_tf_tree(buffer, parent_frame="map", child_frame="base_link")
        position = np.array([1, 2, -3])
        quaternion = np.array([0, 0, 1, 0])
        self.assertTrue(np.allclose(p4.position, position))
        self.assertTrue(np.allclose(p4.rotation.quaternion, quaternion))

    def test_publish_to_tf_tree_static(self):

        # setup testing node and TF infrastructure
        # use static TF publisher since we only publish once and it might take a while
        rospy.init_node("SE3_tf_test")
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        broadcaster = tf2_ros.StaticTransformBroadcaster()

        # wait for TFs to become available
        rospy.sleep(1)

        position1 = np.array([-1, 2, 3])
        q1 = np.array([0, 0, 1, 0])
        position2 = np.array([0, -1, 0])
        q2 = np.array([0, 0, 0, 1])
        p1 = SE3()
        p2 = SE3.from_pos_quat(position=position1, quaternion=q1)
        p3 = SE3.from_pos_quat(position=position2, quaternion=q2)

        # test zero TF
        p1.publish_to_tf_tree(broadcaster, parent_frame="frame_a", child_frame="frame_b")

        # test TFs with "random" numbers
        p2.publish_to_tf_tree(broadcaster, parent_frame="map1", child_frame="odom1")
        p3.publish_to_tf_tree(broadcaster, parent_frame="odom1", child_frame="base_link1")

        # wait for TFs to become available
        rospy.sleep(1)

        # make sure all published TFs are available
        new_p1 = SE3.from_tf_tree(buffer, parent_frame="frame_a", child_frame="frame_b")
        new_p2 = SE3.from_tf_tree(buffer, parent_frame="map1", child_frame="odom1")
        new_p3 = SE3.from_tf_tree(buffer, parent_frame="odom1", child_frame="base_link1")
        new_p4 = SE3.from_tf_tree(buffer, parent_frame="map1", child_frame="base_link1")
        self.assertTrue(np.allclose(new_p1.position, np.zeros(3)))
        self.assertTrue(np.allclose(new_p1.rotation.quaternion, q2))
        self.assertTrue(np.allclose(new_p2.position, position1))
        self.assertTrue(np.allclose(new_p2.rotation.quaternion, q1))
        self.assertTrue(np.allclose(new_p3.position, position2))
        self.assertTrue(np.allclose(new_p3.rotation.quaternion, q2))
        self.assertTrue(np.allclose(new_p4.position, np.array([-1, 3, 3])))
        self.assertTrue(np.allclose(new_p4.rotation.quaternion, np.array([0, 0, 1, 0])))

    # TODO: this would probably require a separate testing node to be running at the same time,
    #       just replacing the static TF publisher with a regular one causes the TFs not to be found
    def test_publish_to_tf_tree_regular(self):
        ...


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SE3_tf_test", TestSE3)
