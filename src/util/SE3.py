from __future__ import annotations
from typing import Optional

import numpy as np
import tf2_ros
import rospy

from ros_numpy import numpify
from tf.transformations import (quaternion_inverse, quaternion_matrix,
                                quaternion_multiply, rotation_from_matrix)
from .SO3 import SO3


class SE3:
    position: np.ndarray
    rotation: SO3

    def __init__(self, position: np.ndarray = None, rotation: np.ndarray = None):
        """
        Initialize an SE3 object

        :param position: optional numpy position vector [x, y, z], defaults to zero vector
        :param rotation: optional numpy quaternion vector [x, y, z, w], defaults to [0, 0, 0, 1]
        """
        if position is None:
            self.position = np.zeros(3)
        else:
            self.position = position

        if rotation is None:
            self.rotation = SO3()
        else:
            self.rotation = rotation

    @classmethod
    def from_tf_tree(cls, tf_buffer: tf2_ros.Buffer, parent_frame: str, child_frame: str) -> Optional[SE3]:
        """
        Ask the TF tree for a transform from parent_frame to child_frame,
        and return it as an SE3.

        :param tf_buffer: the tf buffer used to query the TF tree
        :param parent_frame: the parent frame of the desired transform
        :param child_frame: the child frame of the desired transform

        :returns: an SE3 containing the tranform from parent_frame to child_frame,
                  or None if the transform can't be found 
        """
        try:
            tf_msg = tf_buffer.lookup_transform(
                parent_frame, child_frame, rospy.Time()).transform

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        result = SE3()
        result.position = numpify(tf_msg.translation)
        result.rotation = SO3(numpify(tf_msg.rotation))
        return result

    def publish_to_tf_tree(self, parent_frame: str, child_frame: str):
        ...

    def pos_distance_to(self, p: SE3) -> float:
        """
        Get the euclidean distance from the position of this SE3 pose to the position of another SE3 pose

        :param p: another SE3 pose object
        :returns: euclidean distance between the two SE3 poses
        """
        return np.linalg.norm(p.position_vector() - self.position_vector())

    def rot_distance_to(self, p: SE3) -> float:
        """
        Get the rotational distance between this SE3 pose and another SE3 pose,
        defined as the angle of rotation from one quaternion to the other along the shortest arc.

        :param p: the other SE3 pose object
        :returns: the angle in radians between the orientations of the two poses
        """
        q1 = self.quaternion_array()
        q2 = p.quaternion_array()
        q_diff = quaternion_multiply(quaternion_inverse(q1), q2)
        rotation_matrix = quaternion_matrix(q_diff)
        angle, _, _ = rotation_from_matrix(rotation_matrix)
        return angle
