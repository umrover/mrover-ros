from abc import ABC
from typing import List

import rospy
import smach
from context import Context
from geometry_msgs.msg import Twist

from util.SE3 import SE3


class BaseState(smach.State, ABC):
    """
    Custom base state which handles termination cleanly via smach preemption.
    """
    context: Context

    def __init__(self, context: Context, outcomes: List[str], input_keys: List[str], output_keys: List[str]):
        super().__init__(
            outcomes + ['terminated'],
            input_keys + ['waypoint_index'],
            output_keys + ['waypoint_index']
        )
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

    def rover_pose(self) -> SE3:
        return self.transform('base_link')

    def transform(self, frame: str, parent_frame: str = 'odom') -> SE3:
        """
        :param frame:
        :param parent_frame:
        :return:
        """
        # TODO: use SE3 function to lookup
        stamped_transform = self.context.tf_buffer.lookup_transform(parent_frame, frame, rospy.Time(0))
        return SE3.from_tf(stamped_transform.transform)


class DoneState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['done', 'waypoint_traverse'],
            input_keys=[],
            output_keys=[]
        )

    def evaluate(self, ud):
        if self.context.course and ud.waypoint_index != len(self.context.course.waypoints):
            return 'waypoint_traverse'

        cmd_vel = Twist()
        self.context.vel_cmd_publisher.publish(cmd_vel)
        return 'done'
