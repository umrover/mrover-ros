import numpy as np

import rospy
import tf
from common import Context, BaseState
from geometry_msgs.msg import Twist, Point
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker

COURSE_FRAME = 'course'
ROVER_FRAME = 'rover'


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
            target_dir = course_pos - rover_pos
            target_dir /= np.linalg.norm(target_dir)
            rover_dir = quaternion_matrix(rover_rot)[0:3, 0]
            alignment = np.dot(target_dir, rover_dir)

            self.send_debug_arrow(rover_rot)

            rover_dir_rotated = rover_dir[0] * -target_dir[1] + rover_dir[1] * target_dir[0]
            print(course_pos, rover_pos, target_dir, rover_dir, alignment, np.sign(rover_dir_rotated))
            error = 1.0 - alignment * np.sign(rover_dir_rotated)

            cmd_vel = Twist()
            cmd_vel.angular.z = error * 2.0
            self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint'

    def send_debug_arrow(self, rot):
        marker = Marker()
        marker.id = 0
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
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
