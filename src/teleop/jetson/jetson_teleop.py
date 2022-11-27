#!/usr/bin/env python3
# Node for teleop-related callback functions

import math
from math import copysign
import typing
from enum import IntEnum
import rospy as ros
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist


def quadratic(val):
    return copysign(val**2, val)


# If below threshold, make output zero
def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold) / (1 - threshold)

    return copysign(temp_mag, magnitude)


class Drive:
    def __init__(self):
        self.joystick_mappings = ros.get_param("teleop/joystick_mappings")
        self.drive_config = ros.get_param("teleop/drive_controls")

        # Constants for diff drive
        self.max_wheel_speed = ros.get_param("rover/max_speed")
        wheel_radius = ros.get_param("wheel/radius")
        self.max_angular_speed = self.max_wheel_speed / wheel_radius
        self.twist_pub = ros.Publisher("/cmd_vel", Twist, queue_size=100)

    def teleop_drive_callback(self, msg):
        joints: typing.Dict[str, JointState] = {}

        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg.axes[self.joystick_mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(
            msg.axes[self.joystick_mappings["forward_back"]] * self.drive_config["forward_back"]["multiplier"], 0.05
        )

        # Convert from [0,1] to [0, max_wheel_speed] and apply dampen
        linear *= self.max_wheel_speed * dampen

        angular = deadzone(
            msg.axes[self.joystick_mappings["left_right"]] * self.drive_config["left_right"]["multiplier"]
            + msg.axes[self.joystick_mappings["twist"]] * self.drive_config["twist"]["multiplier"],
            0.05,
        )

        # Same as linear but for angular speed
        angular *= self.max_angular_speed * dampen
        # Clamp if both twist and left_right are used at the same time
        if abs(angular) > self.max_angular_speed:
            angular = copysign(self.max_angular_speed, angular)
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        self.twist_pub.publish(twist_msg)


class ArmControl:
    def __init__(self):
        self.xbox_mappings = ros.get_param("teleop/xbox_mappings")
        self.ra_config = ros.get_param("teleop/ra_controls")

        self.ra_cmd_pub = ros.Publisher("ra_cmd", JointState, queue_size=100)

        self.ra_names = [
            "ra_joint_a",
            "ra_joint_b",
            "ra_joint_c",
            "ra_joint_d",
            "ra_joint_e",
            "ra_joint_f",
            "ra_finger",
            "ra_gripper",
        ]
        self.ra_cmd = JointState(
            name=[name for name in self.ra_names],
            position=[math.nan for i in range(len(self.ra_names))],
            velocity=[0.0 for i in range(len(self.ra_names))],
            effort=[math.nan for i in range(len(self.ra_names))],
        )

    def ra_control_callback(self, msg):
        self.ra_cmd[self.ra_names.index("joint_a")] = self.ra_config["joint_a"]["multiplier"] * quadratic(
            deadzone(msg.axes[self.xbox_mappings["left_js_x"]], 0.15)
        )
        self.ra_cmd[self.ra_names.index("joint_b")] = self.ra_config["joint_b"]["multiplier"] * quadratic(
            -deadzone(msg.axes[self.xbox_mappings["left_js_y"]], 0.15)
        )
        self.ra_cmd[self.ra_names.index("joint_c")] = self.ra_config["joint_c"]["multiplier"] * quadratic(
            -deadzone(msg.axes[self.xbox_mappings["right_js_y"]], 0.15)
        )
        self.ra_cmd[self.ra_names.index("joint_d")] = self.ra_config["joint_d"]["multiplier"] * quadratic(
            deadzone(msg.axes[self.xbox_mappings["right_js_x"]], 0.15)
        )
        self.ra_cmd[self.ra_names.index("joint_e")] = self.ra_config["joint_e"]["multiplier"] * quadratic(
            msg.buttons[self.xbox_mappings["right_trigger"]] - msg.buttons[self.xbox_mappings["left_trigger"]]
        )
        self.ra_cmd[self.ra_names.index("joint_f")] = self.ra_config["joint_f"]["multiplier"] * (
            msg.buttons[self.xbox_mappings["right_bumper"]] - msg.buttons[self.xbox_mappings["left_bumper"]]
        )

        self.ra_cmd[self.ra_names.index("finger")] = self.ra_config["finger"]["multiplier"] * (
            msg.buttons[self.xbox_mappings["y"]] - msg.buttons[self.xbox_mappings["a"]]
        )
        self.ra_cmd[self.ra_names.index("gripper")] = self.ra_config["gripper"]["multiplier"] * (
            msg.buttons[self.xbox_mappings["b"]] - msg.buttons[self.xbox_mappings["x"]]
        )

        self.ra_cmd_pub.publish(self.ra_cmd)


def main():
    ros.init_node("teleop")

    arm = ArmControl()
    drive = Drive()

    ros.Subscriber("joystick", Joy, drive.teleop_drive_callback)
    ros.Subscriber("xbox/ra_control", Joy, arm.ra_control_callback)

    ros.spin()


if __name__ == "__main__":
    main()
