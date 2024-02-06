#!/usr/bin/env python3

from pathlib import Path
from typing import Optional
import tf2_ros
from sensor_msgs.msg import NavSatFix, Imu
import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from mrover.msg import MotorsStatus
from nav_msgs.msg import Odometry
from pandas import DataFrame
from std_msgs.msg import Bool
from util.ros_utils import get_rosparam
from nav_msgs.msg import Odometry
from util.SE3 import SE3
import time


class TipDetection:
    hit_count: int
    orientation_threshold: float
    angular_velocity_threshold_x: float
    angular_velocity_threshold_y: float
    time_threshold: float
    hit_count_threshold: int
    reset_hit_count_threshold: int
    time_counter: time

    def __init__(self):
        rospy.Subscriber("imu/imu_only", Imu, self.imu_callback)
        # read the orientation data from the given Imu message, store that value in `self.pose`, then publish that pose to the TF tree.
        self.hit_count = 0
        self.orientation_threshold = 0.5
        self.angular_velocity_threshold_x = 0.25  # pitch
        self.angular_velocity_threshold_y = 0.25  # roll
        self.time_threshold = 1
        self.hit_count_threshold = 5
        self.reset_hit_count_threshold = 10
        self.time_counter = time.time()

        self.buffer = tf2_ros.Buffer()
        self.world_frame = rospy.get_param("world_frame")
        self.rover_frame = rospy.get_param("rover_frame")
        print(self.world_frame)
        print(self.rover_frame)
        self.in_loop()

        # odometry_sub = message_filters.Subscriber("global_ekf/odometry", Odometry)
        # odometry_sub.registerCallback(self.odometry_callback)

    def in_loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                print(SE3.from_tf_tree(self.buffer, self.world_frame, self.rover_frame))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
        

    def imu_callback(self, msg: Imu):
        pass

    # every time data is gotten from odometry, odometry_callback() is called from init()
    # calling the rest of the functions like detect_tip() in odometry_callback()
    def odometry_callback(self, odometry):

        self.x = abs(odometry.pose.pose.orientation.x)
        self.y = abs(odometry.pose.pose.orientation.y)
        self.z = abs(odometry.pose.pose.orientation.z)
        self.w = abs(odometry.pose.pose.orientation.w)

        self.detect_tip(odometry)
        self.perform_robotic_tasks(self.time_threshold)
        self.second_check(odometry)
        self.check_for_hit_count(self.hit_count)
        self.reset_hit_count_time(self.reset_hit_count_threshold, self.time_counter)

    # checking the rover's velocity
    def detect_tip(self, odometry):
        # printing rover orientation
        rospy.loginfo("self.x" + str(self.x))
        rospy.loginfo("self.y" + str(self.y))
        rospy.loginfo("self.z" + str(self.z))
        rospy.loginfo("self.w" + str(self.w))

        # checking if orientation exceeds orientation_threshold, then increment hit_count
        if self.w >= self.orientation_threshold:
            self.hit_count += 1

        # checking angular velocity of rover

        # create the separate angular velocity variables
        angular_velocity_x = abs(odometry.twist.twist.angular.x)
        angular_velocity_y = abs(odometry.twist.twist.angular.y)

        # check angular velocity magnitudes of each axis to see if it's above the threshold

        # Tipping over (pitch)
        if angular_velocity_x >= self.angular_velocity_threshold_x:
            self.hit_count += 1
        # Rolling over (roll)
        if angular_velocity_y >= self.angular_velocity_threshold_y:
            self.hit_count += 1

    # crates the time difference
    def perform_robotic_tasks(self, time_threshold):
        # Simulate robotic tasks
        rospy.loginfo("performing tasks...")
        rospy.sleep(time_threshold)  # Simulate a x second delay

    # checking acceleration of rover
    # why don't we have an acceleration threshold? why are we using orientation threshold? -audrey
    def second_check(self, odometry):
        pass
        start_time = rospy.Time.now()
        self.perform_robotic_tasks(self.time_threshold)
        current_time = rospy.Time.now()
        # Calculate the time difference
        elapsed_time = current_time - start_time
        # Extract the elapsed time in seconds
        elapsed_seconds = elapsed_time.to_sec()

        linear_acceleration_x = abs(odometry.twist.twist.linear.x)
        linear_acceleration_y = abs(odometry.twist.twist.linear.y)

        # checking if the acceleration exceeds our threshold ?
        # not sure why we're checking for 0, need clarification ^^ -audrey
        if linear_acceleration_x == 0 and elapsed_seconds > self.time_threshold:
            if self.w >= self.orientation_threshold:
                self.hit_count += 1
            elif self.x >= self.orientation_threshold:
                self.hit_count += 1
            elif self.y >= self.orientation_threshold:
                self.hit_count += 1
        elif linear_acceleration_y == 0 and elapsed_seconds > self.time_threshold:
            if self.w >= self.orientation_threshold:
                self.hit_count += 1
            elif self.x >= self.orientation_threshold:
                self.hit_count += 1
            elif self.y >= self.orientation_threshold:
                self.hit_count += 1

    # seeing if hit_count is too big
    def check_for_hit_count(self, hit_count):
        if hit_count > self.hit_count_threshold:
            rospy.loginfo("tipping")
            rospy.loginfo(hit_count)
        else:
            rospy.loginfo("not tipping")
            rospy.loginfo(hit_count)

    def reset_hit_count_time(self, reset_hit_count_threshold, time_counter):
        print(f"time {time.time()- self.time_counter}")
        if time.time() - self.time_counter > self.reset_hit_count_threshold:
            rospy.loginfo("resetting hit count...")
            self.hit_count = 0
            self.time_counter = time.time()


def main():
    rospy.loginfo("===== tip detection starting =====")
    rospy.init_node("tip_detection")
    tip_detector = TipDetection()

    # rospy.spin()  # taken from failure identification but idk what it does exactly


if __name__ == "__main__":
    main()

    """
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
    """
