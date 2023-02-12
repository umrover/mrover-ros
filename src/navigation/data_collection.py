import numpy as np
import pandas as pd
import rospy
from sensor_msgs.msg import JointState
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
    collector_context = ""
    collecting = True
    row = 0


    # Initializes the first _cur_row dataframe and call the subscribers
    # Initialize the dictionary with the Rover's first position, rotation, and timestamp
    # When the datacollection starts.
    def __init__(self):
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
        self._df_all = DataFrame(self.dict)

        # rospy.logerr(f"Ran __init__ in data_collection.py")
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_dataframe)
        rospy.Subscriber("/rover_stuck", Bool, self.set_collecting)

    # Query the tf tree to get odometry. Calculate the linear and angular velocities with this data
    # We will only call this when the object list is not empty. When the list is empty the initial actual
    # linear and angular velocities will be their default values set to zero.
    def update_tf_vel(self) -> bool:
        se3_time = self.collector_context.rover.get_pose_with_time()
        newest_position = se3_time[0].position
        newest_rotation_so3 = se3_time[0].rotation
        newest_timestamp = se3_time[1].to_sec()

        delta_t = newest_timestamp - self._cur_row["timestamp"][0]
        if delta_t < DELTAT_THRESHOLD:
            return False
        delta_theta = newest_rotation_so3.rot_distance_to(SO3(self._cur_row["rotation"][0]))
        actual_linear_vel = (newest_position - (self._cur_row["position"])[0]) / delta_t
        actual_angular_vel = delta_theta / delta_t
        if math.isnan(actual_angular_vel):
            return False

        self._cur_row["timestamp"] = [newest_timestamp]
        self._cur_row["position"] = [newest_position]
        self._cur_row["rotation"] = [newest_rotation_so3.quaternion]
        self._cur_row["actual_linear_vel"] = [actual_linear_vel]
        self._cur_row["actual_angular_vel"] = [actual_angular_vel]
        self._cur_row["actual_linear_speed"] = [np.linalg.norm(actual_linear_vel)]
        return True

    # Calculate the average of the previous 10 data frams and store the outcome
    # In a new dataframe that will ultimately be appended to _df
    def average(self) -> DataFrame:
        # clear average df
        # return a df containing the averaged values
        # For each key
        #   1 through 10 (10 values in total)
        sum = np.zeros(3)
        temp = self._cur_row.copy()
        self._avg_df.reset_index(inplace=True)
        for k in self.dict.keys():
            if not (k == "wheel_names"):
                if k == "timestamp" or k == "actual_angular_vel" or k == "actual_linear_speed" or k == "row":
                    sum = 0.0
                elif len(self.dict[k][0]) == 3:
                    sum = np.zeros(3)
                elif len(self.dict[k][0]) == 4:
                    sum = np.zeros(4)
                else:
                    sum = np.zeros(6)
                for i in range(1, AVERAGE_LEN):
                    sum += self._avg_df.at[i, k]
                average_result = sum / (AVERAGE_LEN - 1)
                temp[k] = [average_result]
        self._avg_df = DataFrame(self.dict)
        # rospy.logerr("AVERAGED")
        temp["row"] = [self.row_counter]
        self.row_counter += 1
        return DataFrame(temp)

    # This function will only be called/invoked when we receive new esw data
    # Callback function for subscriber to JointState
    def make_esw_dataframe(self, esw_data):
        if not self.collecting:
            return

        self._cur_row = self._cur_row.copy()
        self._cur_row["wheel_names"] = [np.array(esw_data.name[0:5])]
        self._cur_row["wheel_effort"] = [np.array(esw_data.effort[0:5])]
        self._cur_row["wheel_vel"] = [np.array(esw_data.velocity[0:5])]
        if self.update_tf_vel():
            self._df = pd.concat([self._df_all, DataFrame(self._cur_row)], axis=0)
            if len(self._avg_df) == AVERAGE_LEN:
                # average
                self._df = pd.concat([self._df, self.average()], axis=0)
                self._avg_df = pd.concat([self._avg_df, DataFrame(self._cur_row)], axis=0)
            else:
                # concat
                self._avg_df = pd.concat([self._avg_df, DataFrame(self._cur_row)], axis=0)

    # This function will only be called/invoked when there is a commanded velocity
    # Called in drive.py
    def make_cmd_vel_dataframe(self, cmd_vel):
        if not self.collecting:
            return

        self._cur_row = self._cur_row.copy()
        self._cur_row["commanded_linear"] = [np.array([cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z])]
        self._cur_row["commanded_angular"] = [np.array([cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z])]
        if self.update_tf_vel():
            # rospy.logerr(f"curr row: {self._cur_row}")
            # rospy.logerr(f"DF ALL LENGTH: {len(self._df_all)}")
            # rospy.logerr(f"DF LENGTH: {len(self._df)}")
            # Remove after debugging
            self._df_all = pd.concat([self._df_all, DataFrame(self._cur_row)], axis=0)
            if len(self._avg_df) == AVERAGE_LEN:
                # average
                self._df = pd.concat([self._df, self.average()], axis=0)
                self._avg_df = pd.concat([self._avg_df, DataFrame(self._cur_row)], axis=0)
            else:
                # concat
                self._avg_df = pd.concat([self._avg_df, DataFrame(self._cur_row)], axis=0)

    # Receives whether we are collecting data from Teleop GUI via the subscriber
    def set_collecting(self, data):
        self.collecting = data

    # Outputs the overall dataframe to the csv
    def write_to_csv(self):
        if not self.collecting:
            return

        now = datetime.datetime.now()
        day = now.strftime("%m%d%Y")
        hour = now.strftime("%H-%M-%S")
        time_stamp = day + "_" + hour
        home = str(Path.home())
        folder = home + "/catkin_ws/src/mrover/failure_data"
        if not os.path.exists(folder):
            os.makedirs(folder)
        file = folder + "/output_averaged_" + time_stamp + ".csv"

        # remove after debugging averaging
        file2 = folder + "/output_" + time_stamp + ".csv"
        # rospy.logerr(f"Created {file} in data_collection.py")
        self._df.to_csv(file)

        # Remove after debugging averaging
        self._df_all.to_csv(file2)

    def set_context(self, context_in):
        self.collector_context = context_in
