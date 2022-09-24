from typing import List

import numpy as np

import tf2_ros
from context import Context, Environment
from drive import get_drive_command
from state import BaseState

STOP_THRESH = 0.5
DRIVE_FWD_THRESH = 0.95
NO_FIDUCIAL = -1


class WaypointState(BaseState):
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
            add_outcomes + ["waypoint_traverse", "single_fiducial", "search", "done"],
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
            return "done"

        # Go into the single fiducial state if we see it early
        if current_waypoint.fiducial_id != Environment.NO_FIDUCIAL and self.context.env.current_fid_pos() is not None:
            return "single_fiducial"

        # Attempt to find the waypoint in the TF tree and drive to it
        try:
            waypoint_pos = self.context.course.current_waypoint_pose().position
            cmd_vel, arrived = get_drive_command(
                waypoint_pos,
                self.context.rover.get_pose(),
                STOP_THRESH,
                DRIVE_FWD_THRESH,
            )
            if arrived:
                if current_waypoint.fiducial_id == NO_FIDUCIAL:
                    # We finished a regular waypoint, go onto the next one
                    self.context.course.increment_waypoint()
                else:
                    # We finished a waypoint associated with a fiducial id, but we have not seen it yet.
                    return "search"
            self.context.rover.send_drive_command(cmd_vel)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass

        return "waypoint_traverse"
