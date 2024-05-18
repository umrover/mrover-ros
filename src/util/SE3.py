from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from std_msgs.msg import Time
from .SO3 import SO3
from .np_utils import numpify


@dataclass(frozen=True)
class SE3:
    """
    An SE3 object represents a pose in 3 dimensions,
    AKA a member of the Special Euclidean group in 3 dimensions (SE3).
    This consists of a 3D position and a 3D rotation (represented as an SO3).

    NOTE: when passing an already existing numpy array to the constructor as the `position` argument,
          make sure to call `.copy()` on it in order to avoid transferring ownership of the array.

          For example:
          >>> arr = np.array([1, 2, 3])
          >>> p = SE3(position=arr.copy())

    """

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: SO3 = SO3()

    @classmethod
    def from_transform_matrix(cls, m: np.ndarray):
        """
        Initialize an SE3 object using a homogenous transform matrix.

        :param m: 4x4 homogenous transform matrix
        """
        position = m[:3, 3]
        rotation = SO3.from_matrix(m[:3, :3])
        return SE3(position, rotation)

    @classmethod
    def from_pos_quat(cls, position: np.ndarray, quaternion: np.ndarray):
        """
        Initialize an SE3 object using a position vector to specify position
        and a quaternion vector to specify rotation.

        :param position: position vector [x, y, z], defaults to zero vector
        :param quaternion: quaternion vector [x, y, z, w], defaults to [0, 0, 0, 1]
        """
        return SE3(position, SO3(quaternion))

    @classmethod
    def from_tf_tree(cls, tf_buffer: tf2_ros.Buffer, parent_frame: str, child_frame: str) -> SE3:
        """
        Ask the TF tree for a transform from parent_frame to child_frame,
        and return it as an SE3.

        :param tf_buffer: the tf buffer used to query the TF tree
        :param parent_frame: the parent frame of the desired transform
        :param child_frame: the child frame of the desired transform

        :raises tf2_ros.LookupException: if one or both of the requested frames don't exist in the TF tree
        :raises tf2_ros.ConnectivityException: if no connection can be found between the two requested frames
        :raises tf2_ros.ExtrapolationException: if the transform would've required extrapolation
                                                (forward or backward in time) beyond current limits

        :returns: an SE3 containing the transform from parent_frame to child_frame
        """
        tf_msg = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time()).transform
        result = SE3(position=numpify(tf_msg.translation), rotation=SO3(numpify(tf_msg.rotation)))
        return result

    @classmethod
    def from_tf_time(cls, tf_buffer: tf2_ros.Buffer, parent_frame: str, child_frame: str) -> Tuple[SE3, Time]:
        """
        Ask the TF tree for the time stamp of a frame and return it in a tuple with an
        SE3 that holds the transform from the parent to the child_frame
        """
        tf_lookup = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time())
        tf_msg = tf_lookup.transform
        result = SE3(position=numpify(tf_msg.translation), rotation=SO3(numpify(tf_msg.rotation)))
        time_stamp = tf_lookup.header.stamp
        return (result, time_stamp)

    def publish_to_tf_tree(
        self,
        tf_broadcaster: tf2_ros.TransformBroadcaster | tf2_ros.StaticTransformBroadcaster,
        parent_frame: str,
        child_frame: str,
    ):
        """
        Publish the SE3 to the TF tree as a transform from parent_frame to child_frame.
        Transform can be published as either a regular transform, which will expire after a short delay,
        or a static transform, which will not expire. This will be decided by the type of
        transform broadcaster passed to the function.

        :param tf_broadcaster: the TF broadcaster used to publish to the TF tree
        :param parent_frame: the parent frame of the transform to be published
        :param child_frame: the child frame of the transform to be published
        """
        tf = TransformStamped()
        tf.transform.translation = Vector3(*self.position)
        tf.transform.rotation = Quaternion(*self.rotation.quaternion)
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf_broadcaster.sendTransform(tf)

    def pos_distance_to(self, p: SE3) -> float:
        """
        Get the euclidean distance from the position of this SE3 to the position of another SE3

        :param p: another SE3
        :returns: euclidean distance between the two SE3s
        """
        return np.linalg.norm(p.position - self.position)  # type: ignore

    def is_approx(self, p: SE3, tolerance=1e-8) -> bool:
        """
        Check if two SE3s are approximately equal within a tolerance by checking that each
        position vector is approximately equal and that each rotation is approximately equal.

        :param p: another SE3
        :param tolerance: the tolerance for comparing each number, if the difference
                          between each number is less than or equal to this tolerance,
                          they will be considered equal
        :returns: True if the two SE3s are approximately equal, False otherwise
        """
        return np.allclose(self.position, p.position, atol=tolerance) and self.rotation.is_approx(p.rotation, tolerance)

    def transform_matrix(self) -> np.ndarray:
        """
        Get the homogenous transform matrix representing this SE3.

        :returns: 4x4 numpy array containing homogenous transform matrix
        """
        m = np.identity(4)
        m[:3, :3] = self.rotation.rotation_matrix()
        m[:3, 3] = self.position
        return m
