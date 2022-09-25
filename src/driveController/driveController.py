from ast import JoinedStr
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

WHEEL_RADIUS = 0.13
WHEEL_BASE = 0.86

LEFT_CHANNEL = "drive_cmd/wheels/left"
RIGHT_CHANNEL = "drive_cmd/wheels/right"


class DriveController:
    def __init__(self):
        self.left_publisher = rospy.Publisher(LEFT_CHANNEL, JointState, queue_size=1)
        self.right_publisher = rospy.Publisher(RIGHT_CHANNEL, JointState, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.process_twist_message)

    def process_twist_message(self, control: Twist):
        forward = control.linear.x
        turn = control.angular.z
        turn_difference = turn * WHEEL_BASE / 2
        left_velocity = forward + turn_difference
        right_velocity = forward - turn_difference
        left_angular, right_angular = left_velocity / WHEEL_RADIUS, right_velocity / WHEEL_BASE
        leftMsg : JointState = JointState()
        rightMsg : JointState = JointState()
        leftMsg.velocity = left_angular
        rightMsg.velocity = right_angular
        #TODO: I'm not sure what the units of velocity are
        self.right_publisher.publish(rightMsg)
        self.left_publisher.publish(leftMsg)

def main():
    rospy.init_node("drive_controller")
    driver = DriveController()
    rospy.spin()