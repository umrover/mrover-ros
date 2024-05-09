"""
Mappings can be found easily with: https://hardwaretester.com/gamepad
The following are for an XBox 360 controller and a Thrustmaster T.16000M joystick
"""

from enum import Enum


class ControllerButton(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
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


class ControllerAxis(Enum):
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3


class JoystickAxis(Enum):
    LEFT_RIGHT = 0
    FORWARD_BACK = 1
    TWIST = 2
    # The slider at the base of the joystick, used to adjust maximum speed
    THROTTLE = 3
    # The small joystick on top of the big one
    MICRO_LEFT_RIGHT = 4
    MICRO_FORWARD_BACK = 5


class KeyboardButton(Enum):
    W = 0
    A = 1
    S = 2
    D = 3
