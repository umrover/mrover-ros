from typing import List

import numpy as np
import rospy

import tf2_ros
from context import Context, Environment
from aenum import Enum, NoAlias
from state import BaseState
from util.ros_utils import get_rosparam


class WaypointStateTransitions(Enum):
    _settings_ = NoAlias

    continue_waypoint_traverse = "WaypointState"
    search_at_waypoint = "SearchState"
    no_waypoint = "DoneState"
    find_approach_post = "ApproachPostState"
    go_to_gate = "GateTraverseState"


class WaypointState(BaseState):
    STOP_THRESH = get_rosparam("waypoint/stop_thresh", 0.5)
    DRIVE_FWD_THRESH = get_rosparam("waypoint/drive_fwd_thresh", 0.34)  # 20 degrees
    NO_FIDUCIAL = get_rosparam("waypoint/no_fiducial", -1)

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
            context,
            add_outcomes + [transition.name for transition in WaypointStateTransitions],  # type: ignore
            add_input_keys,
            add_output_keys,
        )

    def rover_forward(self) -> np.ndarray:
        return self.context.rover.get_pose().rotation.direction_vector()

    def evaluate(self, ud) -> str:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a fiducial id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param ud:  State machine user data
        :return:    Next state
        """
        current_waypoint = self.context.course.current_waypoint()
        if current_waypoint is None:
            return WaypointStateTransitions.no_waypoint.name  # type: ignore

        # Go into either gate or search if we see them early (and are looking)
        if self.context.course.look_for_gate():
            if self.context.env.current_gate() is not None:
                return WaypointStateTransitions.go_to_gate.name  # type: ignore
        if self.context.course.look_for_post():
            if self.context.env.current_fid_pos() is not None:
                return WaypointStateTransitions.find_approach_post.name  # type: ignore

        # Attempt to find the waypoint in the TF tree and drive to it
        try:
            waypoint_pos = self.context.course.current_waypoint_pose().position
            cmd_vel, arrived = self.context.rover.driver.command(
                waypoint_pos,
                self.context.rover.get_pose(),
                self.STOP_THRESH,
                self.DRIVE_FWD_THRESH,
            )
            if arrived:
                if not self.context.course.look_for_gate() and not self.context.course.look_for_post():
                    # We finished a regular waypoint, go onto the next one
                    self.context.course.increment_waypoint()
                else:
                    # We finished a waypoint associated with a fiducial id, but we have not seen it yet.
                    return WaypointStateTransitions.search_at_waypoint.name  # type: ignore
            self.context.rover.send_drive_command(cmd_vel)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass

        return WaypointStateTransitions.continue_waypoint_traverse.name  # type: ignore
