from __future__ import annotations

import numpy as np

from geometry_msgs.msg import Pose, Transform
from ros_numpy import numpify
from tf.transformations import (
    quaternion_inverse,
    quaternion_matrix,
    quaternion_multiply,
    rotation_from_matrix,
)
from .tf_utils import point_to_vector3, vector3_to_point


class SE3:
    position: np.ndarray
    rotation: SO3
    
    def __init__(self):
        self.position = np.zeros(3)
        self.rotation = SO3()

    @classmethod
    def from_tf(cls, tf: Transform) -> SE3:
        """
        Create an SE3 pose object from a Transform message.
        The translation of the Transform will become the position of the Pose,
        and the rotation of the Transform will become the orientation of the Pose.

        :param tf: the Transform message object
        :returns: the created SE3 pose object
        """
        point = vector3_to_point(tf.translation)
        orientation = tf.rotation
        return SE3(position=point, orientation=orientation)

    def to_tf(self) -> Transform:
        """
        Create a transform message from an SE3 pose object.
        The opposite of from_tf(), see its docs for details.

        :returns: the created Transform message object
        """
        translation = point_to_vector3(self.position)
        return Transform(translation=translation, rotation=self.orientation)

    def x_vector(self) -> np.ndarray:
        rotation_matrix = quaternion_matrix(self.quaternion_array())

        return rotation_matrix[0:3, 0]

    def pos_distance_to(self, p: SE3) -> float:
        """
        Get the euclidean distance from the position of this SE3 pose to the position of another SE3 pose

        :param p: another SE3 pose object
        :returns: euclidean distance between the two SE3 poses
        """
        return np.linalg.norm(p.position_vector() - self.position_vector())

    def position_vector(self) -> np.ndarray:
        """
        Get the position vector of the SE3 pose.

        :returns: a position vector [x, y, z]
        """
        return numpify(self.position)

    def quaternion_array(self) -> np.ndarray:
        return numpify(self.orientation)

    def rotation_matrix(self) -> np.ndarray:
        """
        Get the homogenous rotation matrix of the SE3 pose.

        :returns: a homogenous rotation matrix (4x4)
        """
        return quaternion_matrix(self.quaternion_array())

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
