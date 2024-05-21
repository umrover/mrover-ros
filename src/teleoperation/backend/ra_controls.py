from enum import Enum
from math import pi
from typing import Union

import numpy as np

import rospy
from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle, Position
from sensor_msgs.msg import JointState

TAU = 2 * pi

rospy.init_node("teleoperation", disable_signals=True)

throttle_publisher = rospy.Publisher("arm_throttle_cmd", Throttle, queue_size=1)
position_publisher = rospy.Publisher("arm_position_cmd", Position, queue_size=1)


class Joint(Enum):
    A = 0
    B = 1
    C = 2
    DE_PITCH = 3
    DE_ROLL = 4
    ALLEN_KEY = 5
    GRIPPER = 6


# The following are indexed with the values of the enum

JOINT_NAMES = [
    "joint_a",
    "joint_b",
    "joint_c",
    "joint_de_pitch",
    "joint_de_roll",
    "allen_key",
    "gripper",
]

JOINT_SCALES = [
    -1.0,
    0.8,
    1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
]

JOINT_DE_POSITION_SCALE = 1

JOINT_A_MICRO_SCALE = 0.7

CONTROLLER_STICK_DEADZONE = 0.18

# Positions reported by the arm sensors
joint_positions: Union[JointState, None] = None


def joint_positions_callback(msg: JointState) -> None:
    global joint_positions
    joint_positions = msg


rospy.Subscriber("arm_joint_data", JointState, joint_positions_callback)


def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_X),
            quadratic=True,
            scale=JOINT_SCALES[Joint.A.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        )
        + filter_input(
            simulated_axis(controller.buttons, ControllerButton.DPAD_LEFT, ControllerButton.DPAD_RIGHT),
            scale=JOINT_A_MICRO_SCALE,
        ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_Y),
            quadratic=True,
            scale=JOINT_SCALES[Joint.B.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.RIGHT_Y),
            quadratic=True,
            scale=JOINT_SCALES[Joint.C.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.DE_PITCH.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER),
            scale=JOINT_SCALES[Joint.DE_ROLL.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.Y, ControllerButton.A),
            scale=JOINT_SCALES[Joint.ALLEN_KEY.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.B, ControllerButton.X),
            scale=JOINT_SCALES[Joint.GRIPPER.value],
        ),
    ]


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]


def send_ra_controls(ra_mode: str, inputs: DeviceInputs) -> None:
    match ra_mode:
        case "manual" | "hybrid":
            back_pressed = safe_index(inputs.buttons, ControllerButton.BACK) > 0.5
            forward_pressed = safe_index(inputs.buttons, ControllerButton.FORWARD) > 0.5
            home_pressed = safe_index(inputs.buttons, ControllerButton.HOME) > 0.5
            match back_pressed, forward_pressed, home_pressed:
                case True, False, False:
                    de_roll = -TAU / 4
                case False, True, False:
                    de_roll = TAU / 4
                case False, False, True:
                    de_roll = 0
                case _:
                    de_roll = None

            if de_roll is None:
                manual_controls = compute_manual_joint_controls(inputs)

                match ra_mode:
                    case "manual":
                        throttle_publisher.publish(Throttle(JOINT_NAMES, manual_controls))
                    case "hybrid":
                        # Control all joints before DE normally
                        names, throttles = subset(JOINT_NAMES, manual_controls, {Joint.A, Joint.B, Joint.C})
                        throttle_publisher.publish(Throttle(names, throttles))

                        # Control DE joints based on current position
                        if joint_positions:
                            joint_de_pitch = joint_positions.position[joint_positions.name.index("joint_de_pitch")]
                            joint_de_roll = joint_positions.position[joint_positions.name.index("joint_de_roll")]

                            names, (de_pitch_throttle, de_roll_throttle) = subset(
                                JOINT_NAMES, manual_controls, {Joint.DE_PITCH, Joint.DE_ROLL}
                            )
                            position_publisher.publish(
                                Position(
                                    names,
                                    [
                                        # Extend out like a carrot on a stick
                                        np.clip(
                                            joint_de_pitch + de_pitch_throttle * JOINT_DE_POSITION_SCALE,
                                            -TAU / 4,
                                            TAU / 4,
                                        ),
                                        np.clip(
                                            joint_de_roll + de_roll_throttle * JOINT_DE_POSITION_SCALE,
                                            -TAU / 4,
                                            TAU / 4,
                                        ),
                                    ],
                                )
                            )
            else:
                if joint_positions:
                    de_pitch = joint_positions.position[joint_positions.name.index("joint_de_pitch")]
                    position_publisher.publish(Position(["joint_de_roll", "joint_de_pitch"], [de_roll, de_pitch]))
