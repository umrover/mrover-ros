import time
import numpy as np
import pandas as pd
import rospy
from sensor_msgs.msg import JointState
import datetime
from util.SE3 import SE3
from util.SO3 import SO3
from threading import Lock

def make_filename():
    # makes filename based on current time. "output_mmddyyyy_hr-min-sec.csv"
    now = datetime.datetime.now()
    day = now.strftime("%m%d%Y")
    hour = now.strftime("%H-%M-%S")
    time_stamp = day+"_"+hour
    file = "output_"+time_stamp+".csv"
    rospy.logerr(f"{file}")
    return file

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
        self.actual_angular_speed = 0
        self.timestamp = 0.0
        self.curr_position = np.array([0,0,0])
        self.curr_rotation = SO3()

    # Uses data published by ESW to figure out wheel names, current wheel velocity, and effort
    #when one paramter is passed (i.e., len(args) == 1) we have received ESW data from the subscriber
    #If there is no ESW data received and we are calling from make_cmd_vel_obj() then we just pass 3 arguments:
    #These are the wheel_vel, effort, and wheel_names from the previous data object in the list
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
    def update_tf_vel(self, context):
        with self.mutex:
            curr_position = context.rover.get_pose().position
            curr_rotation_so3 = context.rover.get_pose().rotation
            self.curr_position = curr_position
            #Wait delta_t seconds
            delta_t = 3
            time.sleep(delta_t)
            final_position = context.rover.get_pose().position
            final_rotation_so3 = context.rover.get_pose().rotation
            delta_theta = final_rotation_so3.rot_distance_to(curr_rotation_so3)
            self.actual_linear_vel = (final_position - curr_position) / delta_t
            self.actual_angular_vel = delta_theta / delta_t

#The data collector will only create new data objects when there is new ESW data published or there is 
#a commanded velocity from drive.py
class DataCollector:
    def __init__(self):
        self.data_objs = [] # array holding Data_collection type objects
        self.collecting = False
        self.context = ""
        rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)
        self.out_file = make_filename()
        self.csv_data = {"time":0.0,
                    "wheel_names":[],
                    "wheel_vel":[],
                    "wheel_effort":[],
                    "commanded_linear_vel":[],
                    "actual_linear_vel":[],
                    "commanded_angular_vel":[],
                    "actual_angular_vel":0,
                    "curr_position":[],
                    "curr_rotation": SO3()
                    }

        # rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)
        # rospy.Subscriber("/drive_vel_data", JointState, self.make_esw_data_obj)
    
    # This creates a dataframe containing one Data object to send to the csv file 
    def create_dataframe(self, d:Data):
        self.csv_data["time"] = d.timestamp
        self.csv_data["wheel_names"] = [d.wheel_names]
        self.csv_data["wheel_vel"] = [d.wheel_vel]
        self.csv_data["wheel_effort"] = [d.effort]
        self.csv_data["commanded_linear_vel"] = [d.commanded_linear_vel]
        self.csv_data["actual_linear_vel"] = [d.actual_linear_vel]
        self.csv_data["commanded_angular_vel"] = [d.commanded_angular_vel]
        self.csv_data["actual_angular_vel"] = [d.actual_angular_vel]
        self.csv_data["curr_position"] = [d.curr_position]
        self.csv_data["curr_rotation"] = [d.curr_rotation]
        df = pd.DataFrame(self.csv_data)
        return df

    #This function will only be called/invoked when we receive new esw data
    def make_esw_data_obj(self, esw_data):
        d = Data()
        if len(self.data_objs) > 0:
            d.update_commanded_vel(self.data_objs[-1].commanded_linear_vel, self.data_objs[-1].commanded_angular_vel)
            d.update_tf_vel(self.context)
        d.set_esw_data(esw_data)
        # create dataframe and send to csv
        df = self.create_dataframe(d)
        df.to_csv(self.out_file)
        self.data_objs.append(d)
    
    #This function will only be called/invoked when there is a commanded velocity
    def make_cmd_vel_obj(self, cmd_vel):
        d = Data()
        if len(self.data_objs) > 0:
            d.set_esw_data(self.data_objs[-1].wheel_vel, self.data_objs[-1].effort, self.data_objs[-1].wheel_names)
            d.update_tf_vel(self.context) 
        d.update_commanded_vel(cmd_vel)
        # create dataframe and send to csv
        df = self.create_dataframe(d)
        df.to_csv(self.out_file)
        self.data_objs.append(d)

    def set_context(self, context_in):
        self.context = context_in

collection = DataCollector()