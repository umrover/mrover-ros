from enum import Enum
from typing import Union

import rospy
from backend.input import Inputs, filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle, Position
from sensor_msgs.msg import JointState

rospy.init_node("teleoperation", disable_signals=True)

throttle_publisher = rospy.Publisher("arm_throttle_cmd", Throttle, queue_size=1)
position_publisher = rospy.Publisher("arm_position_cmd", Position, queue_size=1)


class Joint(Enum):
    A = 0
    B = 1
    C = 2
    DE_PITCH = 3
    DE_ROLL = 4


# The following are indexed with the values of the enum

JOINT_NAMES = [
    "joint_a",
    "joint_b",
    "joint_c",
    "joint_de_pitch",
    "joint_de_roll",
]

JOINT_SCALES = [
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
]

JOINT_DE_POSITION_SCALE = 1

DEADZONE = 0.18

# Positions reported by the arm sensors
joint_positions: Union[JointState, None] = None


def joint_positions_callback(msg: JointState) -> None:
    global joint_positions
    joint_positions = msg


rospy.Subscriber("arm_joint_data", JointState, joint_positions_callback)


def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        filter_input(safe_index(controller.axes, ControllerAxis.LEFT_X),
                     quadratic=True, scale=JOINT_SCALES[Joint.A.value], deadzone=DEADZONE),
        filter_input(safe_index(controller.axes, ControllerAxis.LEFT_Y),
                     quadratic=True, scale=JOINT_SCALES[Joint.B.value], deadzone=DEADZONE),
        filter_input(safe_index(controller.axes, ControllerAxis.RIGHT_Y),
                     quadratic=True, scale=JOINT_SCALES[Joint.C.value], deadzone=DEADZONE),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.DE_PITCH.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER),
            scale=JOINT_SCALES[Joint.DE_ROLL.value],
        ),
    ]


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]


def compute_ra_controls(inputs: Inputs) -> None:
    match inputs.ra_arm_mode:
        case "manual" | "hybrid":
            manual_controls = compute_manual_joint_controls(inputs.controller)

            match inputs.ra_arm_mode:
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
                                    joint_de_pitch + de_pitch_throttle * JOINT_DE_POSITION_SCALE,
                                    joint_de_roll + de_roll_throttle * JOINT_DE_POSITION_SCALE,
                                ],
                            )
                        )
