from abc import ABC
from typing import Tuple

import numpy as np

import rospy
import smach
import tf
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker


# TODO: rename?
class Context:
    is_shutdown: bool
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    tf_listener: tf.TransformListener

    def __init__(self):
        self.is_shutdown = False
        self.vel_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher('/navigation', Marker, queue_size=1)
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
        """
        Override execute method to add logic for early termination.
        Base classes should override evaluate instead of this!
        """
        if self.preempt_requested():
            self.service_preempt()
            return 'terminated'
        return self.evaluate(ud)

    def evaluate(self, ud: smach.UserData) -> str:
        """Override me instead of execute!"""
        pass

    def transform(self, frame: str, parent_frame: str = 'world', time: rospy.Time = rospy.Time(0)) \
            -> Tuple[np.ndarray, np.ndarray]:
        """Retrieve position and rotation of frame in tf tree. Relative to the point where we linearized.
        :param: frame: Name of desired frame
        :return: position, rotation which are both numpy arrays
        """
        pos, rot = self.context.tf_listener.lookupTransform(parent_frame, frame, time)
        return np.array(pos), np.array(rot)


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
