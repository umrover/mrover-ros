"""This file has the config variables that is used in cameras.py"""
from enum import Enum

NUMBER_OF_PIPELINES = 4

MAX_VIDEO_DEVICE_ID_NUMBER = 10

ARGUMENTS_144 = ['--headless', '--bitrate=300000',
                 '--width=256', '--height=144']
ARGUMENTS_360 = ['--headless', '--bitrate=800000',
                 '--width=480', '--height=360']
ARGUMENTS_720 = ['--headless', '--bitrate=1800000',
                 '--width=1280', '--height=720']

DEFAULT_MISSION = "ERD"

# 10.0.0.1 represents the ip of the main base station laptop
# 10.0.0.2 represents the ip of the secondary science laptop
AUTON_IPS = ["10.0.0.1:5000", "10.0.0.1:5001"]
ERD_IPS = ["10.0.0.1:5000", "10.0.0.1:5001"]
ES_IPS = ["10.0.0.1:5000", "10.0.0.1:5001"]
SCIENCE_IPS = ["10.0.0.1:5000", "10.0.0.1:5001",
               "10.0.0.2:5000", "10.0.0.2:5001"]


class Mission(Enum):
    "This creates all the mission enums and sets them equal to a number"
    AUTON = 0
    ERD = 1
    ES = 2
    SCIENCE = 3


MISSION_MAP = {
    "AUTON": [Mission.AUTON, AUTON_IPS, ARGUMENTS_144],
    "ERD": [Mission.ERD, ERD_IPS, ARGUMENTS_144],
    "ES": [Mission.ES, ES_IPS, ARGUMENTS_720],
    "SCIENCE": [Mission.SCIENCE, SCIENCE_IPS, ARGUMENTS_144]
}
