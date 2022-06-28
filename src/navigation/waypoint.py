from typing import List, Optional

import numpy as np

import tf2_ros
from context import Context
from drive import get_drive_command
from mrover.msg import Waypoint
from state import BaseState
from util import SE3

STOP_THRESH = 0.5
DRIVE_FWD_THRESH = 0.95
NO_FIDUCIAL = -1


class WaypointState(BaseState):
    def __init__(self, context: Context,
                 add_outcomes: List[str] = None,
                 add_input_keys: List[str] = None,
                 add_output_keys: List[str] = None):
        add_outcomes = add_outcomes or []
        add_input_keys = add_input_keys or []
        add_output_keys = add_output_keys or []
        super().__init__(
            context,
            add_outcomes + ['waypoint_traverse', 'single_fiducial', 'done'],
            add_input_keys, add_output_keys
        )

    def waypoint_pose(self, wp_idx: int) -> SE3:
        return self.context.get_transform(self.context.course.waypoints[wp_idx].tf_id)

    def rover_forward(self) -> np.ndarray:
        return self.context.get_rover_pose().x_vector()

    def get_fid_pos(self, fid_id: int) -> Optional[np.ndarray]:
        try:
            fid_pose = self.context.get_transform(f'fiducial{fid_id}')
            return fid_pose.position_vector()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def current_waypoint(self, ud) -> Optional[Waypoint]:
        """
        :param ud:  State machine user data
        :return:    Next waypoint to reach if we have an active course
        """
        if self.context.course is None or ud.waypoint_index >= len(self.context.course.waypoints):
            return None
        return self.context.course.waypoints[ud.waypoint_index]

    def current_fid_pos(self, ud) -> Optional[np.ndarray]:
        current_waypoint = self.current_waypoint(ud)
        if current_waypoint is None or current_waypoint.fiducial_id == NO_FIDUCIAL:
            return None

        return self.get_fid_pos(current_waypoint.fiducial_id)

    def evaluate(self, ud) -> str:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a fiducial id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param ud:  State machine user data
        :return:    Next state
        """
        current_waypoint = self.current_waypoint(ud)
        if current_waypoint is None:
            return 'done'

        # Go into the single fiducial state if we see it early
        if current_waypoint.fiducial_id != NO_FIDUCIAL and self.current_fid_pos(ud) is not None:
            return 'single_fiducial'

        try:
            waypoint_pos = self.waypoint_pose(ud.waypoint_index).position_vector()
            cmd_vel, arrived = get_drive_command(waypoint_pos, self.context.get_rover_pose(),
                                                 STOP_THRESH, DRIVE_FWD_THRESH)
            if arrived:
                if current_waypoint.fiducial_id == NO_FIDUCIAL:
                    # We finished a regular waypoint, go onto the next one
                    ud.waypoint_index += 1
                else:
                    # We finished a waypoint associated with a fiducial id, but we have not seen it yet.
                    return 'single_fiducial'
            self.context.drive_command(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint_traverse'
