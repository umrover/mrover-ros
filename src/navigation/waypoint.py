import math

import numpy as np

import rospy
import tf
from common import Context, BaseState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


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
            pos, rot = self.context.tf_listener.lookupTransform('course', 'rover', rospy.Time(0))
            rot_euler = euler_from_quaternion(rot)
            # angle = np.arctan2(pos[0], pos[1])
            # print(pos)
            # print(np.rad2deg(angle))
            # print(rot_euler)
            print(np.rad2deg(rot_euler[2]))
            cmd_vel = Twist()
            cmd_vel.angular.z = rot_euler[2] / math.tau / 2
            self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'waypoint'
