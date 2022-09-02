from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from tf.transformations import (
    quaternion_inverse,
    quaternion_matrix,
    quaternion_from_matrix,
    quaternion_multiply,
)


@dataclass(frozen=True)
class SO3:
    """
    An SO3 object represents a rotation in 3 dimensions,
    AKA a member of the Special Orthogonal group in 3 dimensions (SO3).

    NOTE: when passing an already existing numpy array to the constructor as the `quaternion` argument,
          make sure to call `.copy()` on it in order to avoid transferring ownership of the array.

          For example:
          >>> arr = np.array([1, 2, 3, 4])
          >>> r = SO3(quaternion=arr.copy())

    """

    quaternion: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 1]))

    @classmethod
    def from_matrix(cls, rotation_matrix: np.ndarray) -> SO3:
        """
        Create an SO3 object from a rotation matrix.

        :param rotation_matrix: the 3x3 rotation matrix
        :returns: the created SO3 object
        """
        homogenous = np.eye(4)
        homogenous[:3, :3] = rotation_matrix
        return SO3(quaternion_from_matrix(homogenous))

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

        :param r: the other SO3
        :returns: the angle in radians between the orientations of the two SO3s
        """
        q1 = self.quaternion
        q2 = r.quaternion
        q_diff = quaternion_multiply(quaternion_inverse(q1), q2)
        angle = 2 * np.arccos(q_diff[3])
        if angle > np.pi:
            angle -= np.pi
        return angle

    def is_approx(self, r: SO3, tolerance=1e-8) -> bool:
        """
        Check if two SO3s are approximately equal within a tolerance by checking that each
        element of the quaternion vector is approximately equal.

        :param r: another SO3
        :param tolerance: the tolerance for comparing each number, if the difference
                          between each number is less than or equal to this tolerance,
                          they will be considered equal
        :returns: True if the two SO3s are approximately equal, False otherwise
        """
        return np.allclose(self.quaternion, r.quaternion, atol=tolerance)
