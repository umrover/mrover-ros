from __future__ import annotations
import numpy as np
from tf.transformations import (
    quaternion_inverse,
    quaternion_matrix,
    quaternion_from_matrix,
    quaternion_multiply,
    rotation_from_matrix,
)


class SO3:
    quaternion: np.ndarray

    def __init__(self, quaternion: np.ndarray = None):
        if quaternion is None:
            self.quaternion = np.array([0, 0, 0, 1])
        else:
            self.quaternion = quaternion

    @classmethod
    def from_matrix(cls, rotation_matrix: np.ndarray) -> SO3:
        """
        Create an SO3 object from a rotation matrix.

        :param rotation_matrix: the 3x3 rotation matrix
        :returns: the created SO3 object
        """
        homogenous = np.eye((4, 4))
        homogenous[:3, :3] = rotation_matrix
        return quaternion_from_matrix(homogenous)

    # TODO: is this method necessary?
    def quaternion_vector(self) -> np.ndarray:
        """
        Get the quaternion vector of the SO3.

        :returns: a quaternion vector [x, y, z, w]
        """
        return self.quaternion

    def rotation_matrix(self) -> np.ndarray:
        """
        Get the rotation matrix representation of the SO3.

        :returns: a 3x3 rotation matrix
        """
        return quaternion_matrix(self.quaternion)[:3, :3]

    def direction_vector(self) -> np.ndarray:
        """
        Get the unit forward direction vector of the SO3 (the x axis vector).

        :returns: unit direction vector [x, y, z]
        """
        return self.rotation_matrix()[:, 0]

    def rot_distance_to(self, r: SO3) -> float:
        """
        Get the rotational distance between this SO3 and another SO3,
        defined as the angle of rotation from one SO3 to the other along the shortest arc.

        :param p: the other SO3
        :returns: the angle in radians between the orientations of the two SO3s
        """
        q1 = self.quaternion_vector()
        q2 = r.quaternion_vector()
        q_diff = quaternion_multiply(quaternion_inverse(q1), q2)
        rotation_matrix = quaternion_matrix(q_diff)
        angle, _, _ = rotation_from_matrix(rotation_matrix)
        return angle
