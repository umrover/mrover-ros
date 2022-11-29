import time
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


def make_filename(csv_data):
    # makes filename based on current time. "output_mmddyyyy_hr-min-sec.csv"
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
    return file


class Data:
    # Default constructor for each data object
    def __init__(self):
        self.mutex = Lock()
        self.wheel_vel = np.array([0, 0, 0, 0, 0, 0])  # m/s
        self.effort = np.array([0, 0, 0, 0, 0, 0])  # Nm
        self.wheel_names = np.array(
            ["", "", "", "", "", ""]
        )  # FrontLeft, FrontRight, MiddleLeft, MiddleRight, BackLeft, BackRight
        self.commanded_linear_vel = np.array([0, 0, 0])
        self.commanded_angular_vel = np.array([0, 0, 0])
        self.actual_linear_vel = np.array([0, 0, 0])
        self.actual_angular_vel = 0
        self.timestamp = rospy.Time()
        self.curr_position = np.array([0, 0, 0])
        self.curr_rotation = SO3()

    # Uses data published by ESW to figure out wheel names, current wheel velocity, and effort
    # when one paramter is passed (i.e., len(args) == 1) we have received ESW data from the subscriber
    # If there is no ESW data received and we are calling from make_cmd_vel_obj() then we just pass 3 arguments:
    # These are the wheel_vel, effort, and wheel_names from the previous data object in the list
    def set_esw_data(self, *args):
        with self.mutex:
            if len(args) == 1:
                self.wheel_names = np.array([args[0].name[0:5]])
                self.effort = np.array([args[0].effort[0:5]])
                self.wheel_vel = np.array([args[0].velocity[0:5]])
            else:
                self.wheel_vel = args[0]
                self.effort = args[1]
                self.wheel_names = args[2]

    # This function is called in drive.py. We are then send the commanded velocity
    # If there is one argument passed, we are calling from drive.py (i.e. we received new commanded velocity data)
    # If there are two arguments passed, we are calling when we get new ESW data and the arguments passed are just
    # The commanded linear and commanded angular velocities from the the previous data object in the list
    def update_commanded_vel(self, *args):
        with self.mutex:
            if len(args) == 1:
                self.commanded_linear_vel = args[0].linear.x
                self.commanded_angular_vel = args[0].angular.z

            else:
                self.commanded_linear_vel = args[0]
                self.commanded_angular_vel = args[1]

    # Query the tf tree to get odometry. Calculate the linear and angular velocities with this data
    # We will only call this when the object list is not empty. When the list is empty the initial actual
    # linear and angular velocities will be their default values set to zero.
    def update_tf_vel(self, context, previous):
        with self.mutex:
            se3_time = context.rover.get_pose_with_time()
            curr_position = se3_time[0].position
            curr_rotation_so3 = se3_time[0].rotation
            self.timestamp = se3_time[1]

            # curr_position = context.rover.get_pose().position
            # curr_rotation_so3 = context.rover.get_pose().rotation
            self.curr_position = curr_position
            self.curr_rotation = curr_rotation_so3
            delta_t = (self.timestamp - previous.timestamp).to_sec()
            previous_position = previous.curr_position
            previous_rotation = previous.curr_rotation
            delta_theta = curr_rotation_so3.rot_distance_to(previous_rotation)
            self.actual_linear_vel = (curr_position - previous_position) / delta_t
            self.actual_angular_vel = delta_theta / delta_t


# The data collector will only create new data objects when there is new ESW data published or there is
# a commanded velocity from drive.py
class DataCollector:
    def __init__(self):
        rospy.logerr(f"Ran __init__ in data_collection.py")
        self.previous_obj = Data()
        self.collecting = False
        self.context = ""
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)
        rospy.Subscriber("/rover_stuck", Bool, self.set_collecting)
        self.csv_data = {
            "time": 0.0,
            "wheel_names": [[]],
            "wheel_vel": [[]],
            "wheel_effort": [[]],
            "commanded_linear_vel": [[]],
            "actual_linear_vel": [[]],
            "commanded_angular_vel": [[]],
            "actual_angular_vel": 0,
            "curr_position": [[]],
            "curr_rotation": [[]],
        }
        self.out_file = make_filename(self.csv_data)

    # Sets the bool value for if we are collecting data when the rover is stuck
    def set_collecting(self, data):
        self.collecting = data

    # This creates a dataframe containing one Data object to send to the csv file
    def create_dataframe(self, d: Data):
        self.csv_data["time"] = d.timestamp
        self.csv_data["wheel_names"] = [d.wheel_names]
        self.csv_data["wheel_vel"] = [d.wheel_vel]
        self.csv_data["wheel_effort"] = [d.effort]
        self.csv_data["commanded_linear_vel"] = [d.commanded_linear_vel]
        self.csv_data["actual_linear_vel"] = [d.actual_linear_vel]
        self.csv_data["commanded_angular_vel"] = [d.commanded_angular_vel]
        self.csv_data["actual_angular_vel"] = [d.actual_angular_vel]
        self.csv_data["curr_position"] = [d.curr_position]
        self.csv_data["curr_rotation"] = [d.curr_rotation.quaternion]
        df = pd.DataFrame(self.csv_data)
        return df

    # This function will only be called/invoked when we receive new esw data
    # Callback function for subscriber to JointState
    def make_esw_data_obj(self, esw_data):
        if not self.collecting:
            return

        rospy.logerr(f"Called make_esw_data_obj")
        d = Data()
        d.update_commanded_vel(self.previous_obj.commanded_linear_vel, self.previous_obj.commanded_angular_vel)
        d.update_tf_vel(self.context, self.previous_obj)
        d.set_esw_data(esw_data)
        # create dataframe and send to csv
        rospy.logerr(f"Create dataframe in esw data and send to csv")
        df = self.create_dataframe(d)
        # Only add header once at the top
        hdr = False if os.path.isfile(self.out_file) else True
        df.to_csv(self.out_file, header=hdr, mode="a", sep="\t")
        self.previous_obj = d

    # This function will only be called/invoked when there is a commanded velocity
    # Called in drive.py
    def make_cmd_vel_obj(self, cmd_vel):
        # if not self.collecting:
        #     return

        d = Data()
        d.set_esw_data(self.previous_obj.wheel_vel, self.previous_obj.effort, self.previous_obj.wheel_names)
        d.update_tf_vel(self.context, self.previous_obj)
        d.update_commanded_vel(cmd_vel)
        # create dataframe and send to csv
        rospy.logerr(f"Create dataframe in cmd vel and send to csv")
        df = self.create_dataframe(d)
        # Only add header once at the top
        hdr = False if os.path.isfile(self.out_file) else True
        df.to_csv(self.out_file, header=hdr, mode="a", sep="\t")
        self.previous_obj = d

    def set_context(self, context_in):
        self.context = context_in
