#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from sensor_msgs.msg import Imu
import tf2_ros
import numpy as np
 
class OdomFrame:
    def __init__(self):
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()
        #self.IMU_Subscriber  = 

    
    def imu_callback(self, msg: Imu)


# TODO:
# 1. Get a subscriber to IMU data
# 2. Implement EKF using http://docs.ros.org/en/noetic/api/robot_localization/html/index.html
# 3. Publish to TF tree using SE3 class

# Add stuff to launch/config file potentially

# <node pkg="robot_localization" type="ekf_localization_node" name="ekf_odom">
#     <param="publish_tf" value=true/> <!-- Automatically publishes the world->base transform, i.e. odom to base_link -->
#     <param name="imu0" value="imu/data"/> <!-- [sensor] -->
#     <rosparam param="imu0_config">[]</rosparam> <!-- TODO: ASK RILEY FOR WHAT OUR STATE IS -->
#     <param name=base_link_frame" value="base_link"/>
#     <param name="odom_frame" value="odom"/>
#     <param name="world_frame" value="odom"/>
#     <rosparam param="process_noise_covariance">[0,0,0;0,0,0;0,0,0]</rosparam> <!--Definitely use this-->
# </node>

# Things to think about??
# dynamic_process_noise_covariance
# initial_estimate_covariance