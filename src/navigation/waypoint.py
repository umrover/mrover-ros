from typing import Tuple

import numpy as np
import tf2_ros
from common import BaseState, Context
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix

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
            # Get vector from rover to waypoint
            target_dir = course_pos - rover_pos
            target_dist = np.linalg.norm(target_dir)
            if target_dist == 0:
                target_dist = np.finfo(float).eps
            # Normalize direction
            target_dir /= target_dist
            rover_dir = self.rover_forward()
            # Both vectors are unit vectors so the dot product magnitude is 0-1
            # 0 alignment is perpendicular, 1 is parallel (fully aligned)
            alignment = np.dot(target_dir, rover_dir)

            if target_dist < 0.5:
                ud.waypoint_index += 1
            else:
                cmd_vel = Twist()
                if alignment > DRIVE_FWD_THRESH:
                    # We are pretty aligned so we can drive straight
                    error = target_dist
                    cmd_vel.linear.x = np.clip(error, 0.0, 1.0)
                # Determine the sign of our effort by seeing if we are to the left or to the right of the target
                # This is done by dotting rover_dir and target_dir rotated 90 degrees ccw
                perp_alignment = rover_dir[0] * -target_dir[1] + rover_dir[1] * target_dir[0]
                sign = -np.sign(perp_alignment)
                # 1 is target alignment
                error = 1.0 - alignment
                cmd_vel.angular.z = np.clip(error * 100.0 * sign, -1.0, 1.0)
                self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint_traverse'
