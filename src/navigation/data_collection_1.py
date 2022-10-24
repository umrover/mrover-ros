from context import Rover
import numpy as np
import pandas as pd
import rospy
import sensor_msgs
from datetime import datetime
from util.SE3 import SE3

states = []

class State:
    def __init__(self):
        self.wheel_vel= np.array([0,0,0,0,0,0])
        self.effort = np.array([0,0,0,0,0,0])
        self.wheel_names = np.array(["", "", "", "", "", ""])
        self.commanded_linear_vel = np.array([0,0,0])
        self.commanded_angular_vel = np.array([0,0,0])
        self.actual_linear_vel = np.array([0,0,0])
        self.actual_angular_vel = np.array([0,0,0])
        self.timestamp = 0.0
        self.curr_position = np.ndarray([0,0,0])
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

    def update_commanded_vel(self, cmd_vel, arrived):
        self.commanded_linear_vel = cmd_vel.linear.x
        self.commanded_angular_vel = cmd_vel.angular.z

    def update_tf_vel(self):
        #Query the tf tree to get odometry. Calculate the velocity with this data
        self.curr_position = Rover.get_pose().position
        self.timestamp = datetime.now().time()
        if len(states) != 0:
            self.actual_linear_vel = (self.curr_position - states[-1].curr_postion) / (self.timestamp - states[-1].timestamp)

    #to get ESW data
    def get_wheel_data(self):
        rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/drive_vel_data", JointState, update_wheel_vel)
        rospy.Subscriber("/drive_vel_data", JointState, update_current_effort)
        rospy.Subscriber("/drive_vel_data", JointState, update_wheel_names)

class DataCollection:
    def __init__(self):
        self.states = []
        self.collecting = False

    def update(self):
        state = State()
        state.update_current_vel()
        state.update_current_effort()
        state.update_state_machine_name()
        state.update_current_linear_vel()
        state.update_current_angular_vel()
        state.update_wheel_names()
        state.update_tf_vel()
        self.states.append(state)
