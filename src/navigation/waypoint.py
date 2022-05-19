from typing import Tuple

import numpy as np
import tf2_ros
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix
from drive import get_drive_command

from common import Context, BaseState

DRIVE_FWD_THRESH = 0.95

class WaypointState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['waypoint_traverse', 'waypoint_done'],
            input_keys=['waypoint_index', 'waypoints'],
            output_keys=['waypoint_index']
        )

    def waypoint_transform(self, ud, wp_idx: int) -> Tuple[np.ndarray, np.ndarray]:
        return self.transform(ud.waypoints[wp_idx])

    def rover_forward(self) -> np.ndarray:
        _, rover_rot = self.rover_transform()
        # Extract what the x-axis (forward) is with respect to our rover rotation
        return quaternion_matrix(rover_rot)[0:3, 0]

    def evaluate(self, ud):
        if ud.waypoint_index >= len(ud.waypoints):
            return 'waypoint_done'
        try:
            course_pos, course_rot = self.waypoint_transform(ud, ud.waypoint_index)
            rover_pos, _ = self.rover_transform()
            cmd_vel, is_done = get_drive_command(course_pos, rover_pos, self.rover_forward(), 0.5, DRIVE_FWD_THRESH)
            if is_done:
                ud.waypoint_index += 1
            else:
                self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint_traverse'
