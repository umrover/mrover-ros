from __future__ import annotations
from tf_utils import vector3_to_point, point_to_vector3
from ros_numpy import numpify
from gettext import translation
import numpy as np
from geometry_msgs.msg import Pose, Transform
from tf.transformations import quaternion_matrix, quaternion_inverse, \
    quaternion_multiply, rotation_from_matrix


class SE3(Pose):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @classmethod
    def from_tf(cls, tf: Transform) -> SE3:
        """
        Create an SE3 pose object from a Transform message.
        The translation of the Transform will become the position of the Pose,
        and the rotation of the Transform will become the orientation of the Pose.

        Args:
            tf: the Transform message object

        Returns:
            the created SE3 pose object
        """
        point = vector3_to_point(tf.translation)
        orientation = tf.rotation
        return SE3(position=point, orientation=orientation)

    def to_tf(self) -> Transform:
        """
        Create a transform message from an SE3 pose object.
        The opposite of from_tf(), see its docs for details.

        Returns:
            the created Transform message object
        """
        translation = point_to_vector3(self.position)
        return Transform(translation=translation, rotation=self.orientation)

    def x_vector(self) -> np.ndarray:
        """
        Get the unit direction vector of the SE3 pose's X axis.

        Returns:
            unit direction vector [x, y, z]
        """
        rotation_matrix = quaternion_matrix(self.quaternion_array())

        # Extract what the x-axis (forward) is with respect to our rover rotation
        return rotation_matrix[0:3, 0]

    def pos_distance_to(self, p: SE3) -> float:
        """
        Get the euclidean distance from the position of this SE3 pose 
        to the position of another SE3 pose.

        Args:
            p: another SE3 pose object

        Returns:
            euclidean distance between the two SE3 poses
        """
        return np.linalg.norm(p.position_vector() - self.position_vector())

    def position_vector(self) -> np.ndarray:
        """
        Get the position vector of the SE3 pose.

        Returns:
            a position vector [x, y, z]
        """
        return numpify(self.position)

    def quaternion_array(self) -> np.ndarray:
        """
        Get the quaternion array of the SE3 pose.

        Returns:
            a quaternion array [x, y, z, w]
        """
        return numpify(self.orientation)

    def rotation_matrix(self) -> np.ndarray:
        """
        Get the homogenous rotation matrix of the SE3 pose.

        Returns:
            a homogenous rotation matrix (4x4)
        """
        return quaternion_matrix(self.quaternion_array())

    def rot_distance_to(self, p: SE3) -> float:
        """
        Get the rotational distance between this SE3 pose and another SE3 pose,
        defined as the angle of rotation from one quaternion to the other along the shortest arc.

        Args:
            p: the other SE3 pose object

        Returns:
            the angle in radians between the orientations of the two poses
        """
        q1 = self.quaternion_array()
        q2 = p.quaternion_array()
        q_diff = quaternion_multiply(quaternion_inverse(q1), q2)
        rotation_matrix = quaternion_matrix(q_diff)
        angle, _, _ = rotation_from_matrix(rotation_matrix)
        return angle
