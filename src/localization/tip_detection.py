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
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Bool
from util.ros_utils import get_rosparam
from nav_msgs.msg import Odometry
from util.SE3 import SE3

# detect what the pitch is like in the function
# if the pitch is too high, make it a fail state
# do fail states later?

class tip_detection:
hit_count: int
threshold: int
angular_threshold: int
linear_threshold: int

def __init__(self):
rospy.Subscriber("imu/imu_only", Imu, self.detect_tip)
#read the orientation data from the given Imu message, store that value in `self.pose`, then publish that pose to the TF tree.
self.hit_count = 0
self.threshold = 0
self.angular_threshold = 0
self.linear_threshold = 0

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


# hit count set to different threshold values; if more than x values out of 10 point out to the rover tipping over; the rover is tipping over

if w >= self.threshold or -self.threshold >= w:
hit_count +=1
elif x >= self.threshold or -self.threshold >= x:
hit_count +=1
elif y >= self.threshold or -self.threshold >= y:
hit_count +=1

def first_check(self, odometry, angular_velocity):
# check magnitude of angular velocity for each axis
x = odometry.pose.pose.orientation.x
y = odometry.pose.pose.orientation.y
z = odometry.pose.pose.orientation.z
w = odometry.pose.pose.orientation.w

# create the separate angular velocity variables
angular_velocity_x = odometry.twist.twist.angular.x
angular_velocity_y = odometry.twist.twist.angular.y

# create the magnitude variables
angular_velocity_x_magnitude = [angular_velocity_x]
angular_velocity_y_magnitude = [angular_velocity_y]

#check angular velocity magnitudes of each axis to see if it's above the threshold

# Tipping over (pitch)
if angular_velocity_x_magnitude > self.angular_threshold:
hit_count += 1
# Rolling over (roll)
elif angular_velocity_y_magnitude > self.angular_threshold:
hit_count += 1

def second_check(self, odometry, linear_acceleration):
x = odometry.pose.pose.orientation.x
y = odometry.pose.pose.orientation.y
z = odometry.pose.pose.orientation.z
w = odometry.pose.pose.orientation.w







linear_velocity_norm = np.linalg.norm(
np.array([odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z])
)

linear_velocity = linear_velocity_norm



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