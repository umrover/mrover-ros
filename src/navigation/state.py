from abc import ABC
from typing import List

import smach
from context import Context
from aenum import Enum, NoAlias
from geometry_msgs.msg import Twist


class BaseState(smach.State, ABC):
    """
    Custom base state which handles termination cleanly via smach preemption.
    """

    context: Context

    def __init__(
        self,
        context: Context,
        add_outcomes: List[str] = None,
        add_input_keys: List[str] = None,
        add_output_keys: List[str] = None,
    ):
        add_outcomes = add_outcomes or []
        add_input_keys = add_input_keys or []
        add_output_keys = add_output_keys or []
        super().__init__(
            add_outcomes + ["terminated"],
            add_input_keys + ["waypoint_index"],
            add_output_keys + ["waypoint_index"],
        )
        self.context = context

    def execute(self, ud):
        """
        Override execute method to add logic for early termination.
        Base classes should override evaluate instead of this!
        :param ud:  State machine user data
        :return:    Next state, 'terminated' if we want to quit early
        """
        if self.preempt_requested():
            self.service_preempt()
            return "terminated"
        return self.evaluate(ud)

    def evaluate(self, ud: smach.UserData) -> str:
        """Override me instead of execute!"""
        pass


class DoneStateTransitions(Enum):
    _settings_ = NoAlias

    idle = "DoneState"
    begin_course = "WaypointState"


class DoneState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=[transition.name for transition in DoneStateTransitions],  # type: ignore
        )

    def evaluate(self, ud):
        # Check if we have a course to traverse
        if self.context.course and (not self.context.course.is_complete()):
            return DoneStateTransitions.begin_course.name  # type: ignore

        # Stop rover
        cmd_vel = Twist()
        self.context.rover.send_drive_command(cmd_vel)
        return DoneStateTransitions.idle.name  # type: ignore
