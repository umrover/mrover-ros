import numpy as np
import rospy
from pandas import DataFrame
from util.ros_utils import get_rosparam
from util.SO3 import SO3

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

    def get_start_end_positions(self, dataframe: DataFrame):
        #get the average of the first 25% of the dataframes x, y, z position from the inputted dataframe
        cutoff = int(len(dataframe) * 0.25)
        start = dataframe.loc[dataframe["row"] <= cutoff]
        end = dataframe.loc[dataframe["row"] >= len(dataframe) - cutoff]
        start_x = np.mean(start["x"])
        start_y = np.mean(start["y"])
        start_z = np.mean(start["z"])
        start_pos = np.array([start_x, start_y, start_z])
        end_x = np.mean(end["x"])
        end_y = np.mean(end["y"])
        end_z = np.mean(end["z"])
        end_pos = np.array([end_x, end_y, end_z])
        return start_pos, end_pos
    
    def get_start_end_rotations(self, dataframe: DataFrame):
        #get the average of the first 25% of the dataframes rotation from the inputted dataframe
        cutoff = int(len(dataframe) * 0.25)
        start = dataframe.loc[dataframe["row"] <= cutoff]
        end = dataframe.loc[dataframe["row"] >= len(dataframe) - cutoff]
        start_rot = np.array([np.mean(start["rot_x"]), np.mean(start["rot_y"]), np.mean(start["rot_z"]), np.mean(start["rot_w"])])
        end_rot = np.array([np.mean(end["rot_x"]), np.mean(end["rot_y"]), np.mean(end["rot_z"]), np.mean(end["rot_w"])])
        return SO3(start_rot), SO3(end_rot)
        
    def get_start_end_time(self, dataframe: DataFrame):
        #get the average of the first 25% of the dataframes time from the inputted dataframe
        cutoff = int(len(dataframe) * 0.25)
        start = dataframe.loc[dataframe["row"] <= cutoff]
        end = dataframe.loc[dataframe["row"] >= len(dataframe) - cutoff]
        start_time = np.mean(start["time"])
        end_time = np.mean(end["time"])
        #convert start and end time from duration to seconds
        #start_time = start_time.nsecs / 1000000000
        #end_time = end_time.nsecs / 1000000000
        return start_time, end_time
    
    def check_angular_stuck(self, delta_time, delta_rot, dataframe):
        #check to make sure all cmd_vel_twist values are the same sign and cmd_vel_x values are 0
        turn_are_all_same_sign = dataframe["cmd_vel_twist"].apply(np.sign).eq(dataframe["cmd_vel_twist"].iloc[0]).all()
        linear_are_all_zero = dataframe["cmd_vel_x"].eq(0).all()
        if not turn_are_all_same_sign or not linear_are_all_zero:
            return False
        #check if the angular velocity is less than the threshold for the entire dataframe
        angular_velocity = delta_rot / delta_time
        return abs(angular_velocity) < ANGULAR_THRESHOLD
    
    def check_linear_stuck(self, delta_time, delta_pos, dataframe):
        print(dataframe["cmd_vel_x"])
        print(len(dataframe["cmd_vel_x"]))
        #check to make sure all cmd_vel_x values are the same sign and cmd_vel_twist values are 0
        linear_are_all_same_sign = dataframe["cmd_vel_x"].apply(np.sign).eq(dataframe["cmd_vel_x"].iloc[0]).all()
        #turn_are_all_zero = dataframe["cmd_vel_twist"].eq(0).all()
        linear_are_all_non_zero = dataframe["cmd_vel_x"].ne(0).all()
        print(linear_are_all_same_sign, linear_are_all_non_zero)
        if not linear_are_all_same_sign or not linear_are_all_non_zero:
            print("not driving straight")
            return False
        #check if the linear velocity is less than the threshold for the entire dataframe
        linear_velocity = delta_pos / delta_time
        linear_velocity = np.linalg.norm(linear_velocity)
        print(linear_velocity, LINEAR_THRESHOLD)
        return abs(linear_velocity) < LINEAR_THRESHOLD
        

    """
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
    """
    def is_stuck(self, dataframe: DataFrame):
        if len(dataframe) > DF_THRESHOLD:
            self.update_pointers()
            print(self.collector.left_pointer)
            print(self.collector.right_pointer)
            #get the dataframe from left pointer to right pointer
            #dataframe_after_left = dataframe.loc[dataframe["row"] >= self.collector.left_pointer]
            dataframe_sliced = dataframe.tail(DF_THRESHOLD)#dataframe.loc[dataframe_after_left["row"] <= self.collector.right_pointer]
            #get the start and end position and rotation
            start_pos, end_pos = self.get_start_end_positions(dataframe_sliced)
            start_rot, end_rot = self.get_start_end_rotations(dataframe_sliced)
            #get the start and end time
            start_time, end_time = self.get_start_end_time(dataframe_sliced)
            print(start_time, end_time)
            #get the delta time and delta position
            delta_time = end_time - start_time
            delta_pos = end_pos - start_pos
            #use SO3 distance to find the delta rotation
            delta_rot = start_rot.rot_distance_to(end_rot)
            #check if the robot is stuck
            if self.check_angular_stuck(delta_time, delta_rot, dataframe_sliced) or self.check_linear_stuck(delta_time, delta_pos, dataframe_sliced):
                return True
        return False
