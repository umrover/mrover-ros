"""This file has all the configuration information to be used
in odrives.py"""
from enum import Enum


class Axis(Enum):
    """Each ODrive controls two wheels with its axis0 and axis1.
    This keeps track of which axis maps to the side of the wheel pair."""
    LEFT = 0
    RIGHT = 1


class PAIR(Enum):
    """This lets you know which
    ODrives control
    the front, middle, and back
    wheels.
    ODrive 0 --> front motors
    ODrive 1 --> middle motors
    ODrive 2 --> back motors"""
    FRONT = 0
    MIDDLE = 1
    BACK = 2


MOTOR_MAP = {(Axis.LEFT, PAIR.FRONT): 0,
             (Axis.RIGHT, PAIR.FRONT): 1,
             (Axis.LEFT, PAIR.MIDDLE): 2,
             (Axis.RIGHT, PAIR.MIDDLE): 3,
             (Axis.LEFT, PAIR.BACK): 4,
             (Axis.RIGHT, PAIR.BACK): 5}


# scales normalized inputs to max physical speed of rover in turn/s
SPEED_MULTIPLIER = 50
TURNS_TO_M_S_MULTIPLIER = 0.02513  # from turns/sec to m/s (for 2022 rover)
CURRENT_LIM = 4
ODRIVE_WATCHDOG_TIMEOUT = 0.1
AXIS_SPEED_MULTIPLIER_MAP = [-SPEED_MULTIPLIER, SPEED_MULTIPLIER]
AXIS_VEL_ESTIMATE_MULTIPLIER_MAP = [
    TURNS_TO_M_S_MULTIPLIER, -TURNS_TO_M_S_MULTIPLIER]

FRONT_ODRIVE_ID = "335D36623539"
MIDDLE_ODRIVE_ID = "335B36563539"
BACK_ODRIVE_ID = "2066377F5753"
ODRIVE_IDS = [FRONT_ODRIVE_ID, MIDDLE_ODRIVE_ID, BACK_ODRIVE_ID]

"""ALL OF THE FOLLOWING MAY BE USED IF CAN IS IMPLEMENTED"""


class PROTOCOL(Enum):
    """Communication is either done via USB or CAN."""
    USB = True
    CAN = False


ACTIVE_PROTOCOL = PROTOCOL.USB
"""This has information on the axis CAN node IDs."""
AXIS_CAN_ID = {(Axis.LEFT, PAIR.FRONT): 0x0,
               (Axis.RIGHT, PAIR.FRONT): 0x1,
               (Axis.LEFT, PAIR.MIDDLE): 0x2,
               (Axis.RIGHT, PAIR.MIDDLE): 0x3,
               (Axis.LEFT, PAIR.BACK): 0x4,
               (Axis.RIGHT, PAIR.BACK): 0x5}
