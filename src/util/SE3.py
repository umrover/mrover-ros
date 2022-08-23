from __future__ import annotations
from typing import Optional

import numpy as np
import tf2_ros
import rospy

from ros_numpy import numpify
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
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
            tf_msg = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time()).transform

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        result = SE3()
        result.position = numpify(tf_msg.translation)
        result.rotation = SO3(numpify(tf_msg.rotation))
        return result

    def publish_to_tf_tree(self, tf_broadcaster: tf2_ros.TransformBroadcaster, parent_frame: str, child_frame: str):
        """
        Publish the SE3 to the TF tree as a transform from parent_frame to child_frame.

        :param tf_broadcaster: the TF broadcaster used to publish to the TF tree
        :param parent_frame: the parent frame of the transform to be published
        :param child_frame: the child frame of the transform to be published
        """
        tf = TransformStamped()
        tf.transform.translation = Vector3(*self.position)
        tf.transform.rotation = Quaternion(*self.rotation.quaternion_vector())
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
