#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from sensor_msgs.msg import Imu
import tf2_ros
import tf
import numpy as np
 
class WorldToOdom:
    def __init__(self):
        self.imu_transform_subscriber  = tf2_ros.TransformListener()

    def world_to_odom(self, world_to_base):
        """
        Call every time we get a new world_to_base transform from the GPS
        Get the latest odom_to_base transform and calculate world_to_odom, then publish it to the tf tree
        """
        imu_transform_subscriber = tf.transformListener()
        #Need to get these values
        odom_to_base = SE3.from_tf_tree()

        world_to_odom = SE3()
        # make sure these are SE3 objects
        homogenous_world_to_base = SE3.get_homogenous_transform(world_to_base) 
        homogenous_odom_to_base = SE3.get_homogenous_transform(odom_to_base)

        homogenous_world_to_odom = np.matmul(np.inv(homogenous_odom_to_base), homogenous_world_to_base)
        world_to_odom = SE3.from_homogenous_transform(homogenous_world_to_odom)
        world_to_odom.publish_to_tf_tree(tf2_ros.TransformBroadcaster, "world", "odom")


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