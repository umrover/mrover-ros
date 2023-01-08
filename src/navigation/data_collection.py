import numpy as np
import pandas as pd
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import datetime
from util.SE3 import SE3
from util.SO3 import SO3
from threading import Lock
from pathlib import Path
import os
from dataclasses import dataclass
from pandas import DataFrame

@dataclass
class DataManager:
    _df : DataFrame
    _cur_row : DataFrame
    collector_context = ""
    collecting = True

    #Initializes the first _cur_row dataframe and call the subscribers
    #Initialize the dictionary with the Rover's first position, rotation, and timestamp
    #When the datacollection starts.
    def __init__(self):
        self.dict = {"timestamp": 0, "rotation":np.array(np.zeros(4)), "position":np.array([np.zeros(3)]), 
        "actual_linear_vel":np.array(np.zeros(3)), "actual_angular_vel":np.array(np.zeros(3)),
        "wheel_names":np.array(["","","","","",""]), "wheel_effort": np.array(np.zeros(6)), 
        "wheel_vel":np.array(np.zeros(6)), "commanded_linear":np.array(np.zeros(3)),
        "commanded_angular":np.array(np.zeros(3))}
        self._df = DataFrame()
        self._cur_row = DataFrame(self.dict)
        rospy.logerr(f"Ran __init__ in data_collection.py")
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_dataframe)
        rospy.Subscriber("/rover_stuck", Bool, self.set_collecting)


    # Query the tf tree to get odometry. Calculate the linear and angular velocities with this data
    # We will only call this when the object list is not empty. When the list is empty the initial actual
    # linear and angular velocities will be their default values set to zero.
    def update_tf_vel(self):
        se3_time = self.collector_context.rover.get_pose_with_time()
        newest_position = se3_time[0].position
        newest_rotation_so3 = se3_time[0].rotation
        newest_timestamp = se3_time[1].to_sec()

        delta_t = (newest_timestamp - self._cur_row["timestamp"])

        delta_theta = newest_rotation_so3.rot_distance_to(SO3(((self._cur_row["rotation"])[0]).copy()))
        actual_linear_vel = (newest_position - (self._cur_row["position"])[0]) / delta_t
        actual_angular_vel = delta_theta / delta_t

        self._cur_row["timestamp"] = newest_timestamp
        self._cur_row["position"] = np.array(newest_position)
        self._cur_row["rotation"] = np.array(newest_rotation_so3.quaternion)
        self._cur_row["actual_linear_vel"] = np.array(actual_linear_vel)
        self._cur_row["actual_angular_vel"] = np.array(actual_angular_vel)

    # This function will only be called/invoked when we receive new esw data
    # Callback function for subscriber to JointState
    def make_esw_dataframe(self, esw_data):
        if not self.collecting:
            return
        self._cur_row = self._cur_row.copy()    
        self._cur_row["wheel_names"] = np.array(esw_data.name[0:5])
        self._cur_row["wheel_effort"] = np.array(esw_data.effort[0:5])
        self._cur_row["wheel_vel"] = np.array(esw_data.velocity[0:5])
        self.update_tf_vel()
        
        self._df.merge(self._cur_row)

    # This function will only be called/invoked when there is a commanded velocity
    # Called in drive.py
    def make_cmd_vel_dataframe(self, cmd_vel):
        if not self.collecting:
             return
        self._cur_row = self._cur_row.copy() 
        self._cur_row["commanded_linear"] = np.array(cmd_vel.linear)
        self._cur_row["commanded_angular"] = np.array(cmd_vel.angular)
        self.update_tf_vel()
        self._df.merge(self._cur_row)
    
    #Receives whether we are collecting data from Teleop GUI via the subscriber
    def set_collecting(self, data):
        self.collecting = data

    #Outputs the overall dataframe to the csv
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
        file = folder + "/output_" + time_stamp + ".csv"
        rospy.logerr(f"Created {file} in data_collection.py")
        self._df.to_csv(file)

    def set_context(self, context_in):
        self.collector_context = context_in
        # se3_time = self.collector_context.rover.get_pose_with_time()
        # first_position = se3_time[0].position
        # first_rotation_so3 = se3_time[0].rotation.quaternion
        # first_timestamp = se3_time[1].to_sec()