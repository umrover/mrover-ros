import numpy as np
import tf2_ros
from common import BaseState, Context
from drive import get_drive_command

from util import SE3

DRIVE_FWD_THRESH = 0.95


class WaypointState(BaseState):
    def __init__(self, context: Context, *args, **kwargs):
        kwargs['outcomes'].extend(['waypoint_traverse', 'waypoint_done'])
        kwargs['input_keys'].extend(['waypoint_index', 'waypoints'])
        kwargs['waypoint_index'].extend(['waypoint_index'])
        super().__init__(context, *args, **kwargs)

    def waypoint_pose(self, ud, wp_idx: int) -> SE3:
        return self.transform(ud.waypoints[wp_idx])

    def rover_forward(self) -> np.ndarray:
        return self.rover_pose().x_vector()

    def evaluate(self, ud):
        if ud.waypoint_index >= len(ud.waypoints):
            return 'waypoint_done'
        try:
            course_pos = self.waypoint_pose(ud, ud.waypoint_index).position_vector()
            cmd_vel, is_done = get_drive_command(course_pos, self.rover_pose(), 0.5, DRIVE_FWD_THRESH)
            if is_done:
                ud.waypoint_index += 1
            else:
                self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint_traverse'
