from typing import Tuple

import numpy as np

import rospy
import tf2_ros
from common import Context, BaseState
from geometry_msgs.msg import Twist, Point
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker

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
                rover_dir_90ccw = rover_dir[0] * -target_dir[1] + rover_dir[1] * target_dir[0]
                sign = -np.sign(rover_dir_90ccw)
                # 1 is target alignment
                error = 1.0 - alignment
                cmd_vel.angular.z = np.clip(error * 100.0 * sign, -1.0, 1.0)
                self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint_traverse'

    def send_debug_arrow(self, rot):
        # TODO: not working
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.action = Marker.ADD
        marker.type = Marker.ARROW
        marker.pose.orientation.x = rot[0]
        marker.pose.orientation.y = rot[1]
        marker.pose.orientation.z = rot[2]
        marker.pose.orientation.w = rot[3]
        marker.scale = 2.0
        marker.color.r = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [Point(0, 0, 0), Point(2, 0, 0)]
        self.context.vis_publisher.publish(marker)
