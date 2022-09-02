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
    def __init__(self, joystick_mappings, drive_config, track_radius, wheel_radius):
        self.joystick_mappings = joystick_mappings
        self.drive_config = drive_config

        # Constants for diff drive
        self.TRACK_RADIUS = track_radius  # meter
        self.WHEEL_RADIUS = wheel_radius  # meter
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
    def __init__(self, xbox_mappings, ra_config):
        self.xbox_mappings = xbox_mappings
        self.ra_config = ra_config

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

        joint_a = JointState()
        joint_a.velocity.append(
            self.ra_config["joint_a"]["multiplier"]
            * quadratic(deadzone(msg.axes[self.xbox_mappings["left_js_x"]], 0.15))
        )
        joint_b = JointState()
        joint_b.velocity.append(
            self.ra_config["joint_b"]["multiplier"]
            * quadratic(-deadzone(msg.axes[self.xbox_mappings["left_js_y"]], 0.15))
        )
        joint_c = JointState()
        joint_c.velocity.append(
            self.ra_config["joint_c"]["multiplier"]
            * quadratic(-deadzone(msg.axes[self.xbox_mappings["right_js_y"]], 0.15))
        )
        joint_d = JointState()
        joint_d.velocity.append(
            self.ra_config["joint_d"]["multiplier"]
            * quadratic(deadzone(msg.axes[self.xbox_mappings["right_js_x"]], 0.15))
        )
        joint_e = JointState()
        joint_e.velocity.append(
            self.ra_config["joint_e"]["multiplier"]
            * quadratic(
                msg.buttons[self.xbox_mappings["right_trigger"]] - msg.buttons[self.xbox_mappings["left_trigger"]]
            )
        )
        joint_f = JointState()
        joint_f.velocity.append(
            self.ra_config["joint_f"]["multiplier"]
            * (msg.axes[self.xbox_mappings["right_bumper"]] - msg.axes[self.xbox_mappings["left_bumper"]])
        )

        self.joint_a_pub.publish(joint_a)
        self.joint_b_pub.publish(joint_b)
        self.joint_c_pub.publish(joint_c)
        self.joint_d_pub.publish(joint_d)
        self.joint_e_pub.publish(joint_e)
        self.joint_f_pub.publish(joint_f)

        hand_finger = JointState()
        hand_finger.velocity.append(
            self.ra_config["finger"]["multiplier"]
            * (msg.buttons[self.xbox_mappings["y"]] - msg.buttons[self.xbox_mappings["a"]])
        )
        hand_grip = JointState()
        hand_grip.velocity.append(
            self.ra_config["grip"]["multiplier"]
            * (msg.buttons[self.xbox_mappings["b"]] - msg.buttons[self.xbox_mappings["x"]])
        )

        self.finger_pub.publish(hand_finger)
        self.grip_pub.publish(hand_grip)


def main():
    ros.init_node("teleop")
    xbox = ros.get_param("/teleop/xbox_mappings")
    joystick = ros.get_param("/teleop/xbox_mappings")
    ra_config = ros.get_param("/teleop/ra_controls")
    drive_config = ros.get_param("/teleop/drive_controls")
    track_radius = ros.get_param("/teleop/constants/track_radius")
    wheel_radius = ros.get_param("/teleop/constants/wheel_radius")

    arm = ArmControl(xbox_mappings=xbox, ra_config=ra_config)
    drive = Drive(
        joystick_mappings=joystick, drive_config=drive_config, track_radius=track_radius, wheel_radius=wheel_radius
    )

    ros.Subscriber("/drive_cmd_twist", Twist, drive.teleop_drive_callback)
    ros.Subscriber("/xbox/ra_control", Joy, arm.ra_control_callback)

    ros.spin()


main()
