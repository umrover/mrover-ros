from abc import ABC
from typing import Tuple

import numpy as np

import rospy
import smach
import tf2_ros
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker


# TODO: rename?
class Context:
    is_shutdown: bool
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener

    def __init__(self):
        self.is_shutdown = False
        self.vel_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher('/nav_vis', Marker)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


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

    def rover_transform(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.transform('base_link')

    def transform(self, frame: str, parent_frame: str = 'odom') -> Tuple[np.ndarray, np.ndarray]:
        """Retrieve position and rotation of frame in tf tree. Relative to the point where we linearized.
        :param: frame: Name of desired frame
        :return: position, rotation which are both numpy arrays
        """
        stamped_transform = self.context.tf_buffer.lookup_transform(parent_frame, frame, rospy.Time(0))
        p = stamped_transform.transform.translation
        r = stamped_transform.transform.rotation
        return np.array([p.x, p.y, p.z]), np.array([r.x, r.y, r.z, r.w])


class DoneState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
        )

    def evaluate(self, ud):
        cmd_vel = Twist()
        self.context.vel_cmd_publisher.publish(cmd_vel)
        return 'done'
