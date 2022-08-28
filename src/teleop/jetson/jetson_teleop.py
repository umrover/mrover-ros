#!/usr/bin/env python3
# Node for teleop-related callback functions

from math import copysign
from enum import Enum
import rospy as ros
from sensor_msgs.msg import Joy, JointState
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

        # teleop_twist_joy angular message is maxed at 0.5 regardless of
        # turbo mode, multiply by 2 to make range [-1,1]
        linear = msg.linear.x
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

        joint_a = JointState()
        joint_a.velocity.append(quadratic(deadzone(msg.axes[self.XboxMappings.left_js_x.value], 0.15)))
        joint_b = JointState()
        joint_b.velocity.append(quadratic(-deadzone(msg.axes[self.XboxMappings.left_js_y.value], 0.15)))
        joint_c = JointState()
        joint_c.velocity.append(quadratic(-deadzone(msg.axes[self.XboxMappings.right_js_y.value], 0.15)))
        joint_d = JointState()
        joint_d.velocity.append(quadratic(deadzone(msg.axes[self.XboxMappings.right_js_x.value], 0.15)))
        joint_e = JointState()
        joint_e.velocity.append(quadratic(
                    msg.buttons[self.XboxMappings.right_trigger.value] - msg.buttons[self.XboxMappings.left_trigger.value]
                    ))
        joint_f = JointState()
        joint_f.velocity.append(msg.axes[self.XboxMappings.right_bumper.value] - msg.axes[self.XboxMappings.left_bumper.value])

        joint_a_pub = ros.Publisher("/ra/open_loop/joint_a", JointState, queue_size=100)
        joint_b_pub = ros.Publisher("/ra/open_loop/joint_b", JointState, queue_size=100)
        joint_c_pub = ros.Publisher("/ra/open_loop/joint_c", JointState, queue_size=100)
        joint_d_pub = ros.Publisher("/ra/open_loop/joint_d", JointState, queue_size=100)
        joint_e_pub = ros.Publisher("/ra/open_loop/joint_e", JointState, queue_size=100)
        joint_f_pub = ros.Publisher("/ra/open_loop/joint_f", JointState, queue_size=100)

        joint_a_pub.publish(joint_a)
        joint_b_pub.publish(joint_b)
        joint_c_pub.publish(joint_c)
        joint_d_pub.publish(joint_d)
        joint_e_pub.publish(joint_e)
        joint_f_pub.publish(joint_f)

        hand_finger = JointState()
        hand_finger.velocity.append(msg.buttons[self.XboxMappings.y.value] - msg.buttons[self.XboxMappings.a.value])
        hand_grip = JointState()
        hand_grip.velocity.append(msg.buttons[self.XboxMappings.b.value] - msg.buttons[self.XboxMappings.x.value])

        finger_pub = ros.Publisher("/hand/open_loop/finger", JointState, queue_size=100)
        grip_pub = ros.Publisher("/hand/open_loop/grip", JointState, queue_size=100)

        finger_pub.publish(hand_finger)
        grip_pub.publish(hand_grip)


def main():
    arm = ArmControl()
    drive = Drive()

    ros.init_node("teleop")

    ros.Subscriber("/drive_cmd_twist", Twist, drive.teleop_drive_callback)
    ros.Subscriber("/xbox/ra_control", Joy, arm.ra_control_callback)

    ros.spin()


main()
