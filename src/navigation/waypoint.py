import numpy as np

import rospy
import tf2_ros
from common import Context, BaseState
from geometry_msgs.msg import Twist, Point
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker

COURSE_FRAME = 'course'
ROVER_FRAME = 'rover'


def normalized(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        norm = np.finfo(v.dtype).eps
    return v / norm


class WaypointState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['waypoint', 'waypoint_done'],
            input_keys=[],
            output_keys=[]
        )

    def evaluate(self, ud):
        try:
            course_pos, course_rot = self.transform(COURSE_FRAME)
            rover_pos, rover_rot = self.transform(ROVER_FRAME)
            target_dir = normalized(course_pos - rover_pos)
            rover_dir = quaternion_matrix(rover_rot)[0:3, 0]
            alignment = np.dot(target_dir, rover_dir)

            self.send_debug_arrow(rover_rot)

            rover_dir_90ccw = rover_dir[0] * -target_dir[1] + rover_dir[1] * target_dir[0]
            error = 1.0 - alignment
            # print('course_pos:', course_pos)
            # print('rover_pos:', rover_pos)
            # print('target_dir:', target_dir)
            # print('rover_dir:', rover_dir)
            print('alignment:', alignment)
            # print('error:', error)

            cmd_vel = Twist()
            sign = -np.sign(rover_dir_90ccw)
            cmd_vel.angular.z = np.clip(error * 50.0 * sign, -1.0, 1.0)
            self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint'

    def send_debug_arrow(self, rot):
        marker = Marker()
        marker.id = 0
        marker.action = Marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.x = rot[0]
        marker.pose.orientation.y = rot[1]
        marker.pose.orientation.z = rot[2]
        marker.pose.orientation.w = rot[3]
        marker.scale = 2.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.type = Marker.ARROW
        marker.points = [Point(0, 0, 0), Point(2, 0, 0)]
        self.context.vis_publisher.publish(marker)
