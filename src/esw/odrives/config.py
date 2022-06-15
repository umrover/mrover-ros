"""This file has all the configuration information to be used
in odrives.py"""
from enum import Enum


class OdriveEvent(Enum):
    """These are the the possible Odrive events.
    The OdriveBridge keeps track of a State. This
    State will change depending on the current Odrive event."""
    DISCONNECTED_ODRIVE_EVENT = 0
    ARM_CMD_EVENT = 1
    ODRIVE_ERROR_EVENT = 2


class Axis(Enum):
    """Each Odrive controls two wheels with its axis0 and axis1.
    This keeps track of which axis maps to the side of the wheel pair."""
    LEFT = 0
    RIGHT = 1


# Odrive 0 --> front motors
# Odrive 1 --> middle motors
# Odrive 2 --> back motors
MOTOR_MAP = {(Axis.LEFT, 0): 0, (Axis.RIGHT, 0): 1,
             (Axis.LEFT, 1): 2, (Axis.RIGHT, 1): 3,
             (Axis.LEFT, 2): 4, (Axis.RIGHT, 2): 5}
# scales normalized inputs to max physical speed of rover in turn/s
SPEED_MULTIPLIER = 50
TURNS_TO_M_S_MULTIPLIER = 0.02513  # from turns/sec to m/s (for 2022 rover)
AXIS_SPEED_MULTIPLIER_MAP = [-SPEED_MULTIPLIER, SPEED_MULTIPLIER]
AXIS_VEL_ESTIMATE_MULTIPLIER_MAP = [
    TURNS_TO_M_S_MULTIPLIER, -TURNS_TO_M_S_MULTIPLIER]
CURRENT_LIM = 4
ODRIVE_WATCHDOG_TIMEOUT = 0.1


FRONT_ODRIVE_ID = "335D36623539"
MIDDLE_ODRIVE_ID = "335B36563539"
BACK_ODRIVE_ID = "2066377F5753"
ODRIVE_IDS = [FRONT_ODRIVE_ID, MIDDLE_ODRIVE_ID, BACK_ODRIVE_ID]
