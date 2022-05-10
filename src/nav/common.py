from abc import ABC
from typing import Tuple

import numpy as np
import rospy
import smach
import tf
from geometry_msgs.msg import TwistStamped

LATEST_TIME = rospy.Time(0)


# TODO: rename?
class Context:
    vel_cmd_publisher: rospy.Publisher
    tf_listener: tf.TransformListener

    def __init__(self):
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()


class BaseState(smach.State, ABC):
    navigation: Context

    def transform(self, frame: str) -> Tuple[np.ndarray, np.ndarray]:
        """Retrieve position and rotation of frame in tf tree. Relative to the point where we linearized.
        :param: frame: Name of desired frame
        :return: position, rotation which are both numpy arrays
        """
        try:
            trans, rot = self.navigation.tf_listener.lookupTransform(frame, '/<source>', LATEST_TIME)
            return np.array(trans), np.array(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def __init__(self, navigation: Context, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigation = navigation


class DoneState(BaseState):
    def __init__(self, navigation: Context):
        super().__init__(
            navigation,
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'done'
