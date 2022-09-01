#!/usr/bin/env python3
# Node for teleop-related callback functions

from math import copysign
from enum import IntEnum
import rospy as ros
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from mrover.msg import Chassis, RAOpenLoopCmd, HandCmd


def quadratic(val):
    return copysign(val**2, val)


def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold) / (1 - threshold)

    return copysign(temp_mag, magnitude)


class Drive:
    class LogitechAxes(IntEnum):
        left_right = 0
        forward_back = 1
        twist = 2
        dampen = 3
        pan = 4
        tilt = 5

    def __init__(self):
        # Constants for diff drive
        self.TRACK_RADIUS = 10.0  # meter
        self.WHEEL_RADIUS = 0.5  # meter
        self.drive_vel_pub = ros.Publisher("/drive_cmd_wheels", Chassis, queue_size=100)

    # TODO: Reimplement Dampen Switch
    def teleop_drive_callback(self, msg):

        # teleop_twist_joy angular message is maxed at 0.5 regardless of
        # turbo mode, multiply by 2 to make range [-1,1]
        v = msg.linear.x
        omega = msg.angular.z * 2

        # Transform into L & R wheel angular velocities using diff drive kinematics
        omega_l = (v - omega * self.TRACK_RADIUS / 2.0) / self.WHEEL_RADIUS
        omega_r = (v + omega * self.TRACK_RADIUS / 2.0) / self.WHEEL_RADIUS

        command = Chassis()
        command.omega_l = omega_l
        command.omega_r = omega_r

        self.drive_vel_pub.publish(command)


class ArmControl:
    class XboxMappings(IntEnum):
        left_js_x = 0
        left_js_y = 1
        left_trigger = 6
        right_trigger = 7
        right_js_x = 2
        right_js_y = 3
        right_bumper = 5
        left_bumper = 4
        d_pad_up = 12
        d_pad_down = 13
        d_pad_right = 14
        d_pad_left = 15
        a = 0
        b = 1
        x = 2
        y = 3

    def __init__(self):
        # RA Joint Publishers
        self.joint_a_pub = ros.Publisher("/ra/open_loop/joint_a", JointState, queue_size=100)
        self.joint_b_pub = ros.Publisher("/ra/open_loop/joint_b", JointState, queue_size=100)
        self.joint_c_pub = ros.Publisher("/ra/open_loop/joint_c", JointState, queue_size=100)
        self.joint_d_pub = ros.Publisher("/ra/open_loop/joint_d", JointState, queue_size=100)
        self.joint_e_pub = ros.Publisher("/ra/open_loop/joint_e", JointState, queue_size=100)
        self.joint_f_pub = ros.Publisher("/ra/open_loop/joint_f", JointState, queue_size=100)

        # Hand Publishers
        self.finger_pub = ros.Publisher("/hand/open_loop/finger", JointState, queue_size=100)
        self.grip_pub = ros.Publisher("/hand/open_loop/grip", JointState, queue_size=100)

    def ra_control_callback(self, msg):

        print(self.XboxMappings.left_js_x)

        joint_a = JointState()
        joint_a.velocity.append(quadratic(deadzone(msg.axes[self.XboxMappings.left_js_x], 0.15)))
        joint_b = JointState()
        joint_b.velocity.append(quadratic(-deadzone(msg.axes[self.XboxMappings.left_js_y], 0.15)))
        joint_c = JointState()
        joint_c.velocity.append(quadratic(-deadzone(msg.axes[self.XboxMappings.right_js_y], 0.15)))
        joint_d = JointState()
        joint_d.velocity.append(quadratic(deadzone(msg.axes[self.XboxMappings.right_js_x], 0.15)))
        joint_e = JointState()
        joint_e.velocity.append(
            quadratic(msg.buttons[self.XboxMappings.right_trigger] - msg.buttons[self.XboxMappings.left_trigger])
        )
        joint_f = JointState()
        joint_f.velocity.append(msg.axes[self.XboxMappings.right_bumper] - msg.axes[self.XboxMappings.left_bumper])

        self.joint_a_pub.publish(joint_a)
        self.joint_b_pub.publish(joint_b)
        self.joint_c_pub.publish(joint_c)
        self.joint_d_pub.publish(joint_d)
        self.joint_e_pub.publish(joint_e)
        self.joint_f_pub.publish(joint_f)

        hand_finger = JointState()
        hand_finger.velocity.append(msg.buttons[self.XboxMappings.y] - msg.buttons[self.XboxMappings.a])
        hand_grip = JointState()
        hand_grip.velocity.append(msg.buttons[self.XboxMappings.b] - msg.buttons[self.XboxMappings.x])

        self.finger_pub.publish(hand_finger)
        self.grip_pub.publish(hand_grip)


def main():
    arm = ArmControl()
    drive = Drive()

    ros.init_node("teleop")

    ros.Subscriber("/drive_cmd_twist", Twist, drive.teleop_drive_callback)
    ros.Subscriber("/xbox/ra_control", Joy, arm.ra_control_callback)

    ros.spin()


main()
