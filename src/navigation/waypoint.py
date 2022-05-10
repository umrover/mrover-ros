import rospy
from common import Context, BaseState
from geometry_msgs.msg import Twist


class WaypointState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['waypoint', 'waypoint_done'],
            input_keys=[],
            output_keys=[]
        )

    def evaluate(self, ud):
        next_waypoint = self.transform('course')
        if next_waypoint:
            cmd_vel = Twist()
            cmd_vel.angular.z = -1.0
            self.context.vel_cmd_publisher.publish(cmd_vel)
        return 'waypoint'
