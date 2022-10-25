from context import Rover
import numpy as np
import pandas as pd
import rospy
import sensor_msgs
from datetime import datetime
from util.SE3 import SE3
from util.SO3 import SO3

class Data_collection:
    def __init__(self):
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
        #self.stuck = False

    def update_wheel_vel(self, data):
        self.wheel_vel = np.array([data.velocity[0], data.velocity[1], data.velocity[2],
                                    data.velocity[3], data.velocity[4], data.velocity[5]])

    def update_current_effort(self, data):
        self.effort = np.array([data.effort[0], data.effort[1], data.effort[2],
                                    data.effort[3], data.effort[4], data.effort[5]])

    def update_wheel_names(self, data):
        self.wheel_names = np.array([data.name[0], data.name[1], data.name[2],
                                    data.name[3], data.name[4], data.name[5]])
    
    # This function is called in drive.py. We are then send the commanded velocity
    #TODO: if not called from outside functions, maintain previous values
    def update_commanded_vel(self, cmd_vel):
        self.commanded_linear_vel = cmd_vel.linear.x
        self.commanded_angular_vel = cmd_vel.angular.z

    def update_tf_vel(self, prev, greaterThanZero):
        # Query the tf tree to get odometry. Calculate the linear and angular velocities with this data
        self.curr_position = Rover.get_pose().position
        self.curr_rotation = Rover.get_pose().rotation
        self.timestamp = datetime.now().time()
        #if there is a previous entry in array calculate linear and angular velocity
        if greaterThanZero:
            self.actual_linear_vel = (self.curr_position - prev.curr_postion) / (self.timestamp - prev.timestamp)
            self.actual_angular_vel = self.curr_rotation.rot_distance_to(prev.curr_rotation) / (self.timestamp - prev.timestamp)
    
    # Uses data published by ESW to figure out wheel names, current wheel velocity, and effort
    #to get ESW data
    #TODO: if not called from outside functions, maintain previous values
    #TODO: figure out if Subscriber is a "listener"/continuous or one time 
    def init_subscribers(self):
        # rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/drive_vel_data", JointState, self.update_wheel_vel)
        rospy.Subscriber("/drive_vel_data", JointState, self.update_current_effort)
        rospy.Subscriber("/drive_vel_data", JointState, self.update_wheel_names)

class DataCollector:
    def __init__(self):
        self.states = [] # array holding Data_collection type objects
        self.collecting = False
    
    def make_data_obj(self, prev: Data_collection):
        d = Data_collection()
        d.get_wheel_data()
        d.update_commanded_vel()
        #greaterThanZero = True if len(self.states) > 0 else False
        d.update_tf_vel(prev, True if len(self.states) > 0 else False)
        self.states.append(d)
        
    def update(self):
        self.make_data_obj(self.states[-1])
        # Everything below don't need
        # state = Data_collection()
        # state.update_current_vel()
        # state.update_current_effort()
        # state.update_state_machine_name()
        # state.update_current_linear_vel()
        # state.update_current_angular_vel()
        # state.update_wheel_names()
        # state.update_tf_vel()
        # self.states.append(state)

    def start_collection(self):
        pass
        # while self.collecting
        # every 10 seconds ???
        # make_data_obj

