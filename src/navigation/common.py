from abc import ABC

import rospy
import smach
import tf
from geometry_msgs.msg import Twist


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
