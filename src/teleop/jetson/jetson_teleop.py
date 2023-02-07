#!/usr/bin/env python3
# Node for teleop-related callback functions

import math
from math import copysign
import typing
from enum import Enum
import rospy as ros
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from typing import NoReturn


def quadratic(val: float) -> float:
    return copysign(val**2, val)


# If below threshold, make output zero
def deadzone(magnitude: float, threshold: float) -> float:
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

    def teleop_drive_callback(self, msg: Joy):
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

        # Deadzones for each axis
        left_right = (
            deadzone(
                msg.axes[self.joystick_mappings["left_right"]] * self.drive_config["left_right"]["multiplier"], 0.4
            )
            if self.drive_config["left_right"]["enabled"]
            else 0
        )
        twist = quadratic(
            deadzone(msg.axes[self.joystick_mappings["twist"]] * self.drive_config["twist"]["multiplier"], 0.1)
        )

        angular = twist + left_right

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
    class ArmType(Enum):
        UNKNOWN = 0
        RA = 1
        SA = 2

    def __init__(self):
        self.xbox_mappings = ros.get_param("teleop/xbox_mappings")
        self.ra_config = ros.get_param("teleop/ra_controls")
        self.sa_config = ros.get_param("teleop/sa_controls")

        self.ra_cmd_pub = ros.Publisher("ra_cmd", JointState, queue_size=100)
        self.sa_cmd_pub = ros.Publisher("sa_cmd", JointState, queue_size=100)

        RA_NAMES = [
            "joint_a",
            "joint_b",
            "joint_c",
            "joint_d",
            "joint_e",
            "joint_f",
            "finger",
            "gripper",
        ]
        SA_NAMES = ["joint_a", "joint_b", "joint_c", "joint_d", "scoop", "microscope"]

        self.ra_cmd = JointState(
            name=[name for name in RA_NAMES],
            position=[math.nan for i in range(len(RA_NAMES))],
            velocity=[0.0 for i in range(len(RA_NAMES))],
            effort=[math.nan for i in range(len(RA_NAMES))],
        )

        self.sa_cmd = JointState(
            name=[name for name in SA_NAMES],
            position=[math.nan for i in range(len(SA_NAMES))],
            velocity=[0.0 for i in range(len(SA_NAMES))],
            effort=[math.nan for i in range(len(SA_NAMES))],
        )

    def filter_xbox_axis(
        self,
        axes_array: "list[float]",
        axis_name: str,
        deadzone_threshold: float,
        quad_control: bool = True,
    ) -> float:
        """
        Applies various filtering functions to an axis for controlling the arm
        :param axes_array: Axis array from sensor_msgs/Joy, each value is a float from [-1,1]
        :param axis_name: String representing the axis you are controlling, should match teleop.yaml
        :param deadzone_threshold: Float representing the deadzone of the axis that you would like to use
        :param quad_control: Bool for whether or not we want the axis to follow an x^2 curve instead of a linear one
        Velocities are sent in range [-1,1]
        :return: Returns an output velocity value for the given joint using the given axis_name
        """
        return (
            quadratic(deadzone(axes_array[self.xbox_mappings[axis_name]], deadzone_threshold))
            if quad_control
            else deadzone(axes_array[self.xbox_mappings[axis_name]], deadzone_threshold)
        )

    def filter_xbox_button(self, button_array: "list[int]", pos_button: str, neg_button: str) -> float:
        """
        Applies various filtering functions to an axis for controlling the arm
        :param button_array: Button array from sensor_msgs/Joy, each value is an int 0 or 1
        :param pos_button: String representing the positive button for controlling a joint
        :param neg_button: String representing the negtaive button for controlling a joint
        :return: Return -1, 0, or 1 depending on what buttons are being pressed
        """
        return button_array[self.xbox_mappings[pos_button]] - button_array[self.xbox_mappings[neg_button]]

    def ra_control_callback(self, msg: Joy) -> NoReturn:
        """
        Converts a Joy message with the Xbox inputs
        to a JointState to control the RA Arm in open loop
        :param msg: Has axis and buttons array for Xbox controller
        Velocities are sent in range [-1,1]
        :return:
        """

        # Filter for xbox triggers, they are typically [-1,1]
        raw_left_trigger = msg.axes[self.xbox_mappings["left_trigger"]]
        left_trigger = raw_left_trigger if raw_left_trigger > 0 else 0
        raw_right_trigger = msg.axes[self.xbox_mappings["right_trigger"]]
        right_trigger = raw_right_trigger if raw_right_trigger > 0 else 0

        self.ra_cmd.velocity = [
            self.ra_config["joint_a"]["multiplier"] * self.filter_xbox_axis(msg.axes, "left_js_x", 0.15, True),
            self.ra_config["joint_b"]["multiplier"] * self.filter_xbox_axis(msg.axes, "left_js_y", 0.15, True),
            self.ra_config["joint_c"]["multiplier"] * self.filter_xbox_axis(msg.axes, "right_js_y", 0.15, True),
            self.ra_config["joint_d"]["multiplier"] * self.filter_xbox_axis(msg.axes, "right_js_x", 0.15, True),
            self.ra_config["joint_e"]["multiplier"] * (right_trigger - left_trigger),
            self.ra_config["joint_f"]["multiplier"]
            * self.filter_xbox_button(msg.buttons, "right_bumper", "left_bumper"),
            self.ra_config["finger"]["multiplier"] * self.filter_xbox_button(msg.buttons, "y", "a"),
            self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(msg.buttons, "b", "x"),
        ]
        self.ra_cmd_pub.publish(self.ra_cmd)

    def sa_control_callback(self, msg: Joy) -> NoReturn:
        """
        Converts a Joy message with the Xbox inputs
        to a JointState to control the RA Arm in open loop
        :param msg: Has axis and buttons array for Xbox controller
        Velocities are sent in range [-1,1]
        :return:
        """

        # Filter for xbox triggers, they are typically [-1,1]
        raw_left_trigger = msg.axes[self.xbox_mappings["left_trigger"]]
        left_trigger = raw_left_trigger if raw_left_trigger > 0 else 0
        raw_right_trigger = msg.axes[self.xbox_mappings["right_trigger"]]
        right_trigger = raw_right_trigger if raw_right_trigger > 0 else 0

        self.sa_cmd.velocity = [
            self.sa_config["joint_a"]["multiplier"] * self.filter_xbox_axis(msg.axes, "left_js_x", 0.15, True),
            self.sa_config["joint_b"]["multiplier"] * self.filter_xbox_axis(msg.axes, "left_js_y", 0.15, True),
            self.sa_config["joint_c"]["multiplier"] * self.filter_xbox_axis(msg.axes, "right_js_y", 0.15, True),
            self.sa_config["joint_d"]["multiplier"] * self.filter_xbox_axis(msg.axes, "right_js_x", 0.15, True),
            self.sa_config["scoop"]["multiplier"] * (right_trigger - left_trigger),
            self.sa_config["microscope"]["multiplier"]
            * self.filter_xbox_button(msg.buttons, "right_bumper", "left_bumper"),
        ]
        self.sa_cmd_pub.publish(self.ra_cmd)


def main():
    ros.init_node("teleop")

    arm = ArmControl()
    drive = Drive()

    ros.Subscriber("joystick", Joy, drive.teleop_drive_callback)
    ros.Subscriber("xbox/ra_control", Joy, arm.ra_control_callback)
    ros.Subscriber("xbox/sa_control", Joy, arm.sa_control_callback)

    ros.spin()


if __name__ == "__main__":
    main()
