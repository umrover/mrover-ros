"""This file reads all the information to be used
in odrives.py"""
from enum import Enum

import yaml

with open('config.yml', 'r') as file:
    config = yaml.safe_load(file)


class Axis(Enum):
    """Each ODrive controls two wheels with its axis0 and axis1.
    This keeps track of which axis maps to the side of the wheel pair."""
    LEFT = config['axis']['left']
    RIGHT = config['axis']['right']


class Pair(Enum):
    """Dictates which pair is controlled
    by odrives.py argument"""
    FRONT = config['pair']['front']
    MIDDLE = config['pair']['middle']
    BACK = config['pair']['back']


MOTOR_MAP = {(Axis.LEFT, Pair.FRONT): config['combo']['front_left'],
             (Axis.RIGHT, Pair.FRONT): config['combo']['front_right'],
             (Axis.LEFT, Pair.MIDDLE): config['combo']['middle_left'],
             (Axis.RIGHT, Pair.MIDDLE): config['combo']['middle_right'],
             (Axis.LEFT, Pair.BACK): config['combo']['back_left'],
             (Axis.RIGHT, Pair.BACK): config['combo']['back_right']}


TURNS_TO_M_S_MULTIPLIER = config['info']['turns_to_m_s_multiplier']
CURRENT_LIM = config['config']['current_lim']
ODRIVE_WATCHDOG_TIMEOUT = config['config']['watchdog_timeout']

AXIS_SPEED_MULTIPLIER_MAP = [None] * 2
AXIS_SPEED_MULTIPLIER_MAP[Axis.LEFT.value] = \
    config['info']['speed_multiplier_left']
AXIS_SPEED_MULTIPLIER_MAP[Axis.RIGHT.value] = \
    config['info']['speed_multiplier_right']

AXIS_VEL_ESTIMATE_MULTIPLIER_MAP = [None] * 2
AXIS_VEL_ESTIMATE_MULTIPLIER_MAP[Axis.LEFT.value] = \
    config['info']['turns_to_m_s_multiplier_left']
AXIS_VEL_ESTIMATE_MULTIPLIER_MAP[Axis.RIGHT.value] = \
    config['info']['turns_to_m_s_multiplier_right']

ODRIVE_IDS = [None] * 3
ODRIVE_IDS[Pair.FRONT.value] = config['ids']['front']
ODRIVE_IDS[Pair.MIDDLE.value] = config['ids']['middle']
ODRIVE_IDS[Pair.BACK.value] = config['ids']['back']
