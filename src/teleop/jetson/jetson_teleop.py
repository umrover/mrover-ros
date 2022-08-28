#!/usr/bin/env python3
# Node for teleop-related callback functions

from math import copysign
from enum import Enum
import rospy as ros
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mrover.msg import DriveVelCmd, RAOpenLoopCmd, HandCmd


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
    class LogitechAxes(Enum):
        left_right = 0
        forward_back = 1
        twist = 2
        dampen = 3
        pan = 4
        tilt = 5

    # TODO: Reimplement Dampen Switch
    def teleop_drive_callback(self, msg):

        # teleop_twist_joy message is maxed at 0.5, multiply by 2 to make range [-1,1]
        linear = msg.linear.x * 2
        angular = msg.angular.z * 2

        # Convert arcade drive to tank drive
        # Inversions may need to be done for logitech joystick
        angular_op = (angular / 2) / (abs(linear) + 0.5)
        vel_left = linear + angular_op
        vel_right = linear - angular_op

        # Scale to be within [-1, 1], if necessary
        if abs(vel_left) > 1 or abs(vel_right) > 1:
            if abs(vel_left) > abs(vel_right):
                vel_right /= abs(vel_left)
                vel_left /= abs(vel_left)
            else:
                vel_left /= abs(vel_right)
                vel_right /= abs(vel_right)

        command = DriveVelCmd()
        command.left = vel_left
        command.right = vel_right

        drive_vel_pub = ros.Publisher("/drive_vel_cmd", DriveVelCmd, queue_size=100)
        drive_vel_pub.publish(command)


class ArmControl:

    # Leaving in slow mode for now for testing
    slow_mode = True

    class XboxMappings(Enum):
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

    def ra_control_callback(self, msg):

        print(self.XboxMappings.left_js_x)
        motor_speeds = [
            quadratic(deadzone(msg.axes[self.XboxMappings.left_js_x.value], 0.15)),
            quadratic(-deadzone(msg.axes[self.XboxMappings.left_js_y.value], 0.15)),
            quadratic(-deadzone(msg.axes[self.XboxMappings.right_js_y.value], 0.15)),
            quadratic(deadzone(msg.axes[self.XboxMappings.right_js_x.value], 0.15)),
            quadratic(
                msg.buttons[self.XboxMappings.right_trigger.value] - msg.buttons[self.XboxMappings.left_trigger.value]
            ),
            (msg.axes[self.XboxMappings.right_bumper.value] - msg.axes[self.XboxMappings.left_bumper.value]),
        ]

        if self.slow_mode:
            # slow down joints a, c, e, and f
            motor_speeds[0] *= 0.5
            motor_speeds[2] *= 0.5
            motor_speeds[4] *= 0.5
            motor_speeds[5] *= 0.5

        openloop_msg = RAOpenLoopCmd()
        openloop_msg.throttle = motor_speeds

        ra_open_loop_pub = ros.Publisher("/ra_open_loop_cmd", RAOpenLoopCmd, queue_size=100)
        ra_open_loop_pub.publish(openloop_msg)

        hand_msg = HandCmd()
        hand_msg.finger = msg.buttons[self.XboxMappings.y.value] - msg.buttons[self.XboxMappings.a.value]
        hand_msg.grip = msg.buttons[self.XboxMappings.b.value] - msg.buttons[self.XboxMappings.x.value]

        hand_open_loop_pub = ros.Publisher("/hand_open_loop_cmd", HandCmd, queue_size=100)
        hand_open_loop_pub.publish(hand_msg)


def main():
    arm = ArmControl()
    drive = Drive()

    ros.init_node("teleop")

    ros.Subscriber("/drive_cmd_twist", Twist, drive.teleop_drive_callback)
    ros.Subscriber("/xbox_ra_control", Joy, arm.ra_control_callback)

    ros.spin()


main()
