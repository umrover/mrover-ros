from dataclasses import dataclass

from enum import Enum
from typing import Union, TypeAlias


# Mappings can be found easily with: https://hardwaretester.com/gamepad

# The following are for an XBox 360 controller and a Thrustmaster joystick

class XboxButton(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3


class XboxAxis(Enum):
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_BUMPER = 4
    RIGHT_BUMPER = 5
    LEFT_TRIGGER = 6
    RIGHT_TRIGGER = 7
    BACK = 8
    FORWARD = 9
    LEFT_STICK_CLICK = 10
    RIGHT_STICK_CLICK = 11
    DPAD_UP = 12
    DPAD_DOWN = 13
    DPAD_LEFT = 14
    DPAD_RIGHT = 15
    HOME = 16


class JoystickAxis(Enum):
    LEFT_RIGHT = 0
    FORWARD_BACK = 1
    TWIST = 2
    DAMPEN = 3
    PAN = 4
    TILT = 5


@dataclass
class SimulatedAxis:
    positive: Union[XboxAxis, JoystickAxis, XboxButton]
    negative: Union[XboxAxis, JoystickAxis, XboxButton]


@dataclass
class Input:
    source: Union[XboxAxis, JoystickAxis, XboxButton, SimulatedAxis]
    deadzone: float = 0.1
    quadratic: bool = False
    multiplier: float = 1.0


@dataclass
class ArmControls:
    joint_a: Input = Input(XboxAxis.LEFT_X)
    joint_b: Input = Input(XboxAxis.LEFT_Y)
    joint_c: Input = Input(XboxAxis.RIGHT_Y)
    joint_de_pitch: Input = Input(SimulatedAxis(XboxAxis.RIGHT_TRIGGER, XboxAxis.LEFT_TRIGGER))
    joint_de_roll: Input = Input(SimulatedAxis(XboxAxis.RIGHT_BUMPER, XboxAxis.LEFT_BUMPER))
    allen_key: Input = Input(SimulatedAxis(XboxButton.Y, XboxButton.A))
    gripper: Input = Input(SimulatedAxis(XboxButton.B, XboxButton.X))


@dataclass
class DriveControls:
    pedal: Input = Input(JoystickAxis.FORWARD_BACK)
    wheel: Input = Input(JoystickAxis.LEFT_RIGHT)
