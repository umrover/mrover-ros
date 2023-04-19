import numpy as np
import rospy
from pandas import DataFrame
from util.ros_utils import get_rosparam

DF_THRESHOLD = get_rosparam("watchdog/dataframe_threshold", 100)
STUCK_THRESHOLD = get_rosparam("watchdog/stuck_threshold", 50)
ANGULAR_THRESHOLD = get_rosparam("watchdog/angular_threshold", 0.001)
LINEAR_THRESHOLD = get_rosparam("watchdog/linear_threshold", 0.55)


class WatchDog:
    def __init__(self, collector_in):
        self.collector = collector_in

    def update_pointers(self):
        self.collector.right_pointer = self.collector.row_counter
        self.collector.left_pointer = self.collector.right_pointer - DF_THRESHOLD

    def check_angular_stuck(self, dataframe: DataFrame):
        counter = 0
        self.collector.left_pointer = self.collector.right_pointer - DF_THRESHOLD
        while self.collector.left_pointer < self.collector.right_pointer:
            actual = (dataframe.loc[dataframe["row"] == self.collector.left_pointer]["calculated_angular_velocity"]).iloc[0]
            commanded = (dataframe.loc[dataframe["row"] == self.collector.left_pointer]["cmd_vel_twist"]).iloc[0]
            if actual < ANGULAR_THRESHOLD and commanded > 0:
                counter += 1
            self.collector.left_pointer += 1
        return counter >= STUCK_THRESHOLD

    def check_linear_stuck(self, dataframe: DataFrame):
        counter = 0
        self.collector.left_pointer = self.collector.right_pointer - DF_THRESHOLD
        while self.collector.left_pointer < self.collector.right_pointer:
            actual = (dataframe.loc[dataframe["row"] == self.collector.left_pointer]["calculated_linear_velocity"]).iloc[0]
            commanded = (dataframe.loc[dataframe["row"] == self.collector.left_pointer]["cmd_vel_x"]).iloc[0]
            if actual < LINEAR_THRESHOLD and commanded > 0:
                counter += 1
            self.collector.left_pointer += 1
        return counter >= STUCK_THRESHOLD

    def is_stuck(self, dataframe: DataFrame):
        l = len(dataframe)
        if len(dataframe) > DF_THRESHOLD:
            self.update_pointers()
            if self.check_angular_stuck(dataframe):
                return True
            if self.check_linear_stuck(dataframe):
                return True
        return False
