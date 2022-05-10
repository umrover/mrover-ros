from abc import ABC
from typing import Tuple, Optional

import numpy as np

import rospy
import smach
import tf
from geometry_msgs.msg import Twist

LATEST_TIME = rospy.Time(0)


# TODO: rename?
class Context:
    is_shutdown: bool
    vel_cmd_publisher: rospy.Publisher
    tf_listener: tf.TransformListener

    def __init__(self):
        self.is_shutdown = False
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()


class BaseState(smach.State, ABC):
    """
    Custom base state which handles termination cleanly via smach preemption.
    """
    context: Context

    def __init__(self, context: Context, *args, **kwargs):
        kwargs['outcomes'].append('terminated')
        super().__init__(*args, **kwargs)
        self.context = context

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'terminated'
        return self.evaluate(ud)

    def evaluate(self, ud: smach.UserData) -> str:
        pass

    def transform(self, frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Retrieve position and rotation of frame in tf tree. Relative to the point where we linearized
        :param frame:   Name of desired frame
        :return:        position (vector3), rotation (quaternion) which are both numpy arrays
        """
        try:
            pos, rot = self.context.tf_listener.lookupTransform(frame, 'base_link', LATEST_TIME)
            return np.array(pos), np.array(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None


class DoneState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
        )

    def evaluate(self, ud):
        return 'done'
