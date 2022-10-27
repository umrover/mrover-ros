from context import Rover
import numpy as np
import pandas as pd
import rospy
import sensor_msgs
from datetime import datetime
from util.SE3 import SE3
from util.SO3 import SO3
from threading import Lock

class Data:
    #Default constructor for each data object
    def __init__(self):
        self.mutex = Lock()
        self.wheel_vel= np.array([0,0,0,0,0,0]) # m/s
        self.effort = np.array([0,0,0,0,0,0]) # Nm
        self.wheel_names = np.array(["", "", "", "", "", ""]) # FrontLeft, FrontRight, MiddleLeft, MiddleRight, BackLeft, BackRight 
        self.commanded_linear_vel = np.array([0,0,0])
        self.commanded_angular_vel = np.array([0,0,0])
        self.actual_linear_vel = np.array([0,0,0])
        self.actual_angular_vel = np.array([0,0,0])
        self.timestamp = 0.0
        self.curr_position = np.ndarray([0,0,0])
        self.curr_rotation = SO3()

    # Uses data published by ESW to figure out wheel names, current wheel velocity, and effort
    #when one paramter is passed (i.e., len(args) == 1) we have received ESW data from the subscriber
    #If there is no ESW data received and we are calling from make_cmd_vel_obj() then we just pass 3 arguments:
    #These are the wheel_vel, effort, and wheel_names from the previous data object in the list
    def set_esw_data(self, *args):
        with self.mutex:
            if len(args) == 1:
                self.wheel_names = np.array([args[0].name[:5]])
                self.effort = np.array([args[0].effort[:5]])
                self.wheel_vel = np.array([args[0].velocity[:5]])
            else:
                self.wheel_vel = args[0]
                self.effort = args[1]
                self.wheel_names = args[2]

    # This function is called in drive.py. We are then send the commanded velocity
    #If there is one argument passed, we are calling from drive.py (i.e. we received new commanded velocity data)
    #If there are two arguments passed, we are calling when we get new ESW data and the arguments passed are just
    #The commanded linear and commanded angular velocities from the the previous data object in the list
    def update_commanded_vel(self, *args):
        with self.mutex:
            if(len(args) == 1):
                self.commanded_linear_vel = args[0].linear.x
                self.commanded_angular_vel = args[0].angular.z
            else:
                self.commanded_linear_vel = args[0]
                self.commanded_angular_vel = args[1]

     # Query the tf tree to get odometry. Calculate the linear and angular velocities with this data
     #We will only call this when the object list is not empty. When the list is empty the initial actual
     # linear and angular velocities will be their default values set to zero. 
    def update_tf_vel(self, prev):
        with self.mutex:
            self.curr_position = Rover.get_pose().position
            self.curr_rotation = Rover.get_pose().rotation
            self.timestamp = datetime.now().time()
            self.actual_linear_vel = (self.curr_position - prev.curr_postion) / (self.timestamp - prev.timestamp)
            self.actual_angular_vel = self.curr_rotation.rot_distance_to(prev.curr_rotation) / (self.timestamp - prev.timestamp)


#The data collector will only create new data objects when there is new ESW data published or there is 
#a commanded velocity from drive.py
class DataCollector:
    def __init__(self):
        self.data_objs = [] # array holding Data_collection type objects
        self.collecting = False
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)

    #This function will only be called/invoked when we receive new esw data
    def make_esw_data_obj(self, esw_data):
        d = Data()
        if len(self.data_objs) > 0:
            d.update_commanded_vel(self.data_objs[-1].commanded_linear_vel, self.data_objs[-1].commanded_angular_vel)
            d.update_tf_vel(self.data_objs[-1])
        d.set_esw_data(esw_data)
        self.data_objs.append(d)
    
    #This function will only be called/invoked when there is a commanded velocity
    def make_cmd_vel_obj(self, cmd_vel):
        d = Data()
        if len(self.data_objs) > 0:
            d.set_esw_data(self.data_objs[-1].wheel_vel, self.data_objs[-1].effort, self.data_objs[-1].wheel_names)
            d.update_tf_vel(self.data_objs[-1]) 
        d.update_commanded_vel(cmd_vel)
        self.data_objs.append(d)

    #Output to the csv file using pandas. I did not write this as I am not familiar with pandas
    def output_to_file(self):
        pass
