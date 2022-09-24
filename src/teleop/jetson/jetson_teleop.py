#!/usr/bin/env python3
# Node for teleop-related callback functions

from math import copysign
import typing
from enum import IntEnum
import rospy as ros
from sensor_msgs.msg import Joy, JointState


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


# Modifies joints dict when passed
# Function to assign values to a joint in a joint dict
def create_joint_msg(joints, joint, value):
    joints[joint] = JointState()
    joints[joint].velocity.append(value)


class Drive:
    def __init__(self, joystick_mappings, drive_config, track_radius, wheel_radius):
        self.joystick_mappings = joystick_mappings
        self.drive_config = drive_config

        # Constants for diff drive
        self.TRACK_RADIUS = track_radius  # meter
        self.WHEEL_RADIUS = wheel_radius  # meter
        self.left_wheel_pub = ros.Publisher("drive_cmd/wheels/left", JointState, queue_size=100)
        self.right_wheel_pub = ros.Publisher("drive_cmd/wheels/right", JointState, queue_size=100)

    def teleop_drive_callback(self, msg):
        joints: typing.Dict[str, JointState] = {}

        forward_back = deadzone(
            msg.axes[self.joystick_mappings["forward_back"]] * self.drive_config["forward_back"]["multiplier"], 0.05
        )
        left_right = deadzone(
            msg.axes[self.joystick_mappings["left_right"]] * self.drive_config["left_right"]["multiplier"]
            + msg.axes[self.joystick_mappings["twist"]] * self.drive_config["twist"]["multiplier"],
            0.05,
        )

        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg.axes[self.joystick_mappings["dampen"]], 0.01)

        left = dampen * (forward_back + left_right)
        right = dampen * (forward_back - left_right)

        # Ensure values are [-1,1] for each side
        if abs(left) > 1:
            left = left / abs(left)
        if abs(right) > 1:
            right = right / abs(right)

        create_joint_msg(joints, "left_wheels", left)
        create_joint_msg(joints, "right_wheels", right)

        self.left_wheel_pub.publish(joints["left_wheels"])
        self.right_wheel_pub.publish(joints["right_wheels"])


class ArmControl:
    def __init__(self, xbox_mappings, ra_config):
        self.xbox_mappings = xbox_mappings
        self.ra_config = ra_config

        # RA Joint Publishers
        self.joint_a_pub = ros.Publisher("ra/open_loop/joint_a", JointState, queue_size=100)
        self.joint_b_pub = ros.Publisher("ra/open_loop/joint_b", JointState, queue_size=100)
        self.joint_c_pub = ros.Publisher("ra/open_loop/joint_c", JointState, queue_size=100)
        self.joint_d_pub = ros.Publisher("ra/open_loop/joint_d", JointState, queue_size=100)
        self.joint_e_pub = ros.Publisher("ra/open_loop/joint_e", JointState, queue_size=100)
        self.joint_f_pub = ros.Publisher("ra/open_loop/joint_f", JointState, queue_size=100)

        # Hand Publishers
        self.finger_pub = ros.Publisher("hand/open_loop/finger", JointState, queue_size=100)
        self.grip_pub = ros.Publisher("hand/open_loop/grip", JointState, queue_size=100)

    def ra_control_callback(self, msg):
        joints: typing.Dict[str, JointState] = {}

        # Arm Joints
        create_joint_msg(
            joints,
            "joint_a",
            self.ra_config["joint_a"]["multiplier"]
            * quadratic(deadzone(msg.axes[self.xbox_mappings["left_js_x"]], 0.15)),
        )
        create_joint_msg(
            joints,
            "joint_b",
            self.ra_config["joint_b"]["multiplier"]
            * quadratic(-deadzone(msg.axes[self.xbox_mappings["left_js_y"]], 0.15)),
        )
        create_joint_msg(
            joints,
            "joint_c",
            self.ra_config["joint_c"]["multiplier"]
            * quadratic(-deadzone(msg.axes[self.xbox_mappings["right_js_y"]], 0.15)),
        )
        create_joint_msg(
            joints,
            "joint_d",
            self.ra_config["joint_d"]["multiplier"]
            * quadratic(deadzone(msg.axes[self.xbox_mappings["right_js_x"]], 0.15)),
        )
        create_joint_msg(
            joints,
            "joint_e",
            self.ra_config["joint_e"]["multiplier"]
            * quadratic(
                msg.buttons[self.xbox_mappings["right_trigger"]] - msg.buttons[self.xbox_mappings["left_trigger"]]
            ),
        )
        create_joint_msg(
            joints,
            "joint_f",
            self.ra_config["joint_f"]["multiplier"]
            * (msg.buttons[self.xbox_mappings["right_bumper"]] - msg.buttons[self.xbox_mappings["left_bumper"]]),
        )

        # Hand Joints
        create_joint_msg(
            joints,
            "hand_finger",
            self.ra_config["finger"]["multiplier"]
            * (msg.buttons[self.xbox_mappings["y"]] - msg.buttons[self.xbox_mappings["a"]]),
        )
        create_joint_msg(
            joints,
            "hand_grip",
            self.ra_config["grip"]["multiplier"]
            * (msg.buttons[self.xbox_mappings["b"]] - msg.buttons[self.xbox_mappings["x"]]),
        )

        self.joint_a_pub.publish(joints["joint_a"])
        self.joint_b_pub.publish(joints["joint_b"])
        self.joint_c_pub.publish(joints["joint_c"])
        self.joint_d_pub.publish(joints["joint_d"])
        self.joint_e_pub.publish(joints["joint_e"])
        self.joint_f_pub.publish(joints["joint_f"])
        self.finger_pub.publish(joints["hand_finger"])
        self.grip_pub.publish(joints["hand_grip"])


def main():
    ros.init_node("teleop")
    xbox = ros.get_param("teleop/xbox_mappings")
    joystick = ros.get_param("teleop/joystick_mappings")
    ra_config = ros.get_param("teleop/ra_controls")
    drive_config = ros.get_param("teleop/drive_controls")
    track_radius = ros.get_param("teleop/constants/track_radius")
    wheel_radius = ros.get_param("teleop/constants/wheel_radius")

    arm = ArmControl(xbox_mappings=xbox, ra_config=ra_config)
    drive = Drive(
        joystick_mappings=joystick, drive_config=drive_config, track_radius=track_radius, wheel_radius=wheel_radius
    )

    ros.Subscriber("joystick", Joy, drive.teleop_drive_callback)
    ros.Subscriber("xbox/ra_control", Joy, arm.ra_control_callback)

    ros.spin()


if __name__ == "__main__":
    main()
