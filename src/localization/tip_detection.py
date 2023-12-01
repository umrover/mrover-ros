#!/usr/bin/env python3

from pathlib import Path
from typing import Optional

import message_filters
import numpy as np
import pandas as pd
import rospy
from geometry_msgs.msg import Twist
from mrover.msg import MotorsStatus
from nav_msgs.msg import Odometry
from pandas import DataFrame
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Bool
from util.ros_utils import get_rosparam

# detect what the pitch is like in the function
# if the pitch is too high, make it a fail state
# do fail states later? 

class tip_detection: 
    def __init__(self):
        rospy.Subscriber("imu/imu_only", Imu, self.detect_tip)
        #read the orientation data from the given Imu message, store that value in `self.pose`, then publish that pose to the TF tree.

    def imu_callback(self, msg: Imu):
        self.pose = SE3.from_pos_quat(self.pose.position, np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")
        odometry_sub = message_filters.Subscriber("global_ekf/odometry", Odometry)
        
        #subscribe to IMU
        
    def detect_tip(self, odometry):
        # get rover orientation

        x = odometry.pose.pose.orientation.x
        y = odometry.pose.pose.orientation.y
        z = odometry.pose.pose.orientation.z
        w = odometry.pose.pose.orientation.w

        # get rover linear acceleration and velocity
        linear_velocity_norm = np.linalg.norm(
            np.array([odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z])
        )

        linear_velocity = linear_velocity_norm 
        angular_velocity = odometry.twist.twist.angular.z


        
        """"
        check if rover is tipped over over a certain period of time
        if there are a lot of fluctuations over an arbitrary time period,
        then the rover is tipped

        ex. if there are 50 over-tipped measurements over 10 secs, then tipped
        made up numbers ^^
        

        
        ways we can check the cases for tipping:
        - measure magnitudes of angular velocity and see if there is one that is above a threshold
        - velocity is = or near 0 for a set amount of time
            - check orientation
        





        quartonions:
        basically weird way to store the way the rotations are rotating
        qx = ax * sin(angle/2)
        qy = ay * sin(angle/2)
        qz = az * sin(angle/2)
        qw = cos(angle/2)

        ax, ay, and az are how much it is angled off of each axis
        phone rotating on table: 0, 0, 1
        w is how much it's being rotated based on x,y,z

        when talking ab multiplying quartonions, it's just adding them
        one 90 deg rotate + one 90 deg rotate = 180 deg rotate
        """"