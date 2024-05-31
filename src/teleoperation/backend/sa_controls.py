from enum import Enum
from math import pi

import rospy
from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle

TAU = 2 * pi

rospy.init_node("teleoperation", disable_signals=True)

throttle_publisher = rospy.Publisher("sa_throttle_cmd", Throttle, queue_size=1)


class Joint(Enum):
    SA_X = 0
    SA_Y = 1
    SA_Z = 2
    SAMPLER = 3
    SENSOR_ACTUATOR = 4


# The following are indexed with the values of the enum

JOINT_NAMES = [
    "sa_x",
    "sa_y",
    "sa_z",
    "sampler",
    "sensor_actuator",
]

JOINT_SCALES = [
    -1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
]

CONTROLLER_STICK_DEADZONE = 0.18


def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_Y),
            quadratic=True,
            scale=JOINT_SCALES[Joint.SA_X.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_X),
            quadratic=True,
            scale=JOINT_SCALES[Joint.SA_Y.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.RIGHT_Y),
            quadratic=True,
            scale=JOINT_SCALES[Joint.SA_Z.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER),
            scale=JOINT_SCALES[Joint.SAMPLER.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.SENSOR_ACTUATOR.value],
        ),
    ]


def send_sa_controls(inputs: DeviceInputs) -> None:
    manual_controls = compute_manual_joint_controls(inputs)
    throttle_publisher.publish(Throttle(JOINT_NAMES, manual_controls))
