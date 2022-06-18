from typing import List, Optional

import numpy as np
import tf2_ros
from context import Context
from drive import get_drive_command
from mrover.msg import Waypoint
from state import BaseState

from util import SE3

DRIVE_FWD_THRESH = 0.95
NO_FIDUCIAL = -1


class BaseWaypointState(BaseState):
    def __init__(self, context: Context,
                 outcomes: List[str], input_keys: List[str], output_keys: List[str]):
        super().__init__(
            context,
            outcomes + ['waypoint_traverse', 'single_fiducial', 'done'],
            input_keys, output_keys
        )

    def waypoint_pose(self, wp_idx: int) -> SE3:
        return self.transform(self.context.course.waypoints[wp_idx].tf_id)

    def rover_forward(self) -> np.ndarray:
        return self.rover_pose().x_vector()

    def get_fid_pos(self, fid_id: int) -> Optional[np.ndarray]:
        for fid in self.context.fiducial_transforms.transforms:
            if fid.fiducial_id == fid_id:
                t = fid.transform.translation
                return np.array([t.x, t.y, t.z])
        return None

    def current_waypoint(self, ud) -> Optional[Waypoint]:
        if self.context.course is None or ud.waypoint_index >= len(self.context.course.waypoints):
            return None
        return self.context.course.waypoints[ud.waypoint_index]

    def current_fid_pos(self, ud) -> Optional[np.ndarray]:
        current_waypoint = self.current_waypoint(ud)
        if current_waypoint is None or current_waypoint.fiducial_id == NO_FIDUCIAL:
            return None

        return self.get_fid_pos(current_waypoint.fiducial_id)

    def evaluate(self, ud) -> str:
        current_waypoint = self.current_waypoint(ud)
        if current_waypoint is None:
            return 'done'

        if current_waypoint.fiducial_id != NO_FIDUCIAL and self.current_fid_pos(ud) is not None:
            return 'single_fiducial'

        try:
            waypoint_pos = self.waypoint_pose(ud.waypoint_index).position_vector()
            cmd_vel, arrived = get_drive_command(waypoint_pos, self.rover_pose(), 0.5, DRIVE_FWD_THRESH)
            if arrived:
                if current_waypoint.fiducial_id == NO_FIDUCIAL:
                    ud.waypoint_index += 1
                else:
                    return 'single_fiducial'
            else:
                self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint_traverse'


class WaypointState(BaseWaypointState):
    def __init__(self, context: Context):
        super().__init__(context, [], [], [])
