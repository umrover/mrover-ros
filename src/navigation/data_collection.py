import numpy as np
import pandas as pd
import rospy
from sensor_msgs.msg import JointState
from mrover.msg import MotorsStatus
from std_msgs.msg import Bool
import datetime
from util.SE3 import SE3
from util.SO3 import SO3
from pathlib import Path
import os
import math
from dataclasses import dataclass
from pandas import DataFrame

AVERAGE_LEN = 101
DELTAT_THRESHOLD = 0.001


@dataclass
class DataManager:
    _df: DataFrame
    # collector_context = ""
    collecting = True
    row = 0

    # Initializes the first _cur_row dataframe and call the subscribers
    # Initialize the dictionary with the Rover's first position, rotation, and timestamp
    # When the datacollection starts.
    def __init__(self, rover_in):
        self.rover = rover_in
        self.row_counter = 1
        three_zero = np.zeros(3)
        four_zero = np.zeros(4)
        six_zero = np.zeros(6)
        six_string = np.full((6), "")
        self.dict = {
            "row": [0],
            "timestamp": [0],
            "rotation": [four_zero],
            "position": [three_zero],
            "actual_linear_vel": [three_zero],
            "actual_angular_vel": [three_zero],
            "wheel_names": [six_string],
            "wheel_effort": [six_zero],
            "wheel_vel": [six_zero],
            "commanded_linear": [three_zero],
            "commanded_angular": [three_zero],
            "actual_linear_speed": [0],
        }

        self._df = DataFrame(self.dict)
        self._cur_row = self.dict
        self._avg_df = DataFrame(self.dict)

        # Remove after debugging
        # self._df_all = DataFrame(self.dict)

        # rospy.logerr(f"Ran __init__ in data_collection.py")
        #rospy.Subscriber("/drive_status", MotorsStatus, self.make_esw_dataframe)
        #rospy.Subscriber("/rover_stuck", Bool, self.set_collecting)

    # Query the tf tree to get odometry. Calculate the linear and angular velocities with this data
    # We will only call this when the object list is not empty. When the list is empty the initial actual
    # linear and angular velocities will be their default values set to zero.
    def update_tf_vel(self) -> bool:
        pass

    # Calculate the average of the previous 10 data frams and store the outcome
    # In a new dataframe that will ultimately be appended to _df
    def average(self) -> DataFrame:
        pass
    # This function will only be called/invoked when we receive new esw data
    # Callback function for subscriber to JointState
    def make_esw_dataframe(self, esw_data):
        pass

    # This function will only be called/invoked when there is a commanded velocity
    # Called in drive.py
    def make_cmd_vel_dataframe(self, cmd_vel):
        pass

    # Receives whether we are collecting data from Teleop GUI via the subscriber
    def set_collecting(self, data):
        pass

    # Outputs the overall dataframe to the csv
    def write_to_csv(self):
        pass

        # Remove after debugging averaging
        # self._df_all.to_csv(file2)

    # def set_context(self, context_in):
    #     self.collector_context = context_in
