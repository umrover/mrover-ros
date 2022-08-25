from __future__ import annotations
from typing import Optional, Union
from dataclasses import dataclass

import numpy as np
import tf2_ros
import rospy

from ros_numpy import numpify
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from .SO3 import SO3


@dataclass
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
            self.position = position.copy()

        if rotation is None:
            self.rotation = SO3()
        else:
            self.rotation = SO3(rotation)

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

        :returns: an SE3 containing the tranform from parent_frame to child_frame
        """
        tf_msg = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time()).transform
        result = SE3()
        result.position = numpify(tf_msg.translation)
        result.rotation = SO3(numpify(tf_msg.rotation))
        return result

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
        return np.linalg.norm(p.position - self.position)

    def is_approx(self, p: SE3, tolerance=1e-8) -> bool:
        """
        Check if two SE3s are approximately equal within a tolerance by checking that each
        position vector is approximately equal and that each rotation is approximately equal.

        :param p: another SE3
        :returns: True if the two SE3s are approximately equal, False otherwise
        """
        return np.allclose(self.position, p.position, atol=tolerance) and self.rotation.is_approx(p.rotation, tolerance)

    def __eq__(self, other: object) -> bool:
        """
        Override of the equals operator to determine if two SE3s are approximately equal,
        meaning each element of the position and quaternion are equal within a tolerance of 1e-8.

        :param other: another object to check equality with
        :returns: True if the two objects are approximately equal, False otherwise
        """
        if isinstance(other, SE3):
            return self.is_approx(other)
        return False
