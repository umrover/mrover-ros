#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import csv
import tf2_ros
import math

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
# from util.SE3 import SE3
# from util.SO3 import SO3
from matplotlib import pyplot as plt

# import gps_linearization


lat_arr = []
long_arr = []

coord_arr_left_latitude = []
coord_arr_left_longitude = []
coord_arr_right_latitude = []
coord_arr_right_longitude = []
# coord_arr_left = []
# coord_arr_right = []
distances = []


class Localization:
    # pose: SE3

    def __init__(self):
        # create subscribers for GPS data, linking them to our callback functions
        # TODO
        rospy.Subscriber("gps/fix/rover_gps_left", NavSatFix, self.gps_left_callback)
        rospy.Subscriber("gps/fix/rover_gps_right", NavSatFix, self.gps_right_callback)

        # csvfile = open("/home/daniel/catkin_ws/src/mrover/src/localization/test_gps_sim.csv", "w")
        # csvwriter = csv.writer(csvfile)
        # csvwriter.writerow([9, 10])
        # create a transform broadcaster for publishing to the TF tree
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # initialize pose to all zeros
        # self.pose = SE3()

    def gps_left_callback(self, msg: NavSatFix):
        print(msg)
        # coord_arr_left_latitude.append(gps_linearization.geodetic2enu(msg.latitude, msg.longitude)
        # coord_arr_left_longitude.append(msg.longitude)

        coord_arr_left_latitude.append(msg.latitude)
        coord_arr_left_longitude.append(msg.longitude)
        # coord_arr_left.append([msg.latitude, msg.longitude])

        # lat_arr.append(msg.latitude)
        # long_arr.append(msg.longitude)

    def gps_right_callback(self, msg: NavSatFix):
        print(msg)
        # coord_arr_right.append([msg.latitude, msg.longitude])
        coord_arr_right_latitude.append(msg.latitude)
        coord_arr_right_longitude.append(msg.longitude)
        # coord_arr_right.append([msg.latitude, msg.longitude])

    def main():
        # initialize the node
        rospy.init_node("localization_test")

        # create and start our localization system
        localization = Localization()

        # let the callback functions run asynchronously in the background
        rospy.spin()


if __name__ == "__main__":
    Localization.main()

    # plt.plot(coord_arr_left_latitude[i], coord_arr_left_longitude[i], color="red")
    # plt.plot(coord_arr_right_latitude[i], coord_arr_right_longitude[i], color="blue")
    # plt.show()

    # plt.scatter(coord_arr_left_latitude, coord_arr_left_longitude, color="red")
    # plt.scatter(coord_arr_right_latitude, coord_arr_right_longitude, color="blue")

    # plt.show()

    for i in range(len(coord_arr_left_latitude)):
        print(coord_arr_left_latitude[i] - coord_arr_right_latitude[i])
        print(coord_arr_left_longitude[i] - coord_arr_right_longitude[i])

    # print("\n")
    # print(coord_arr_right_latitude[:10])
    # print(coord_arr_left_latitude[:10])

    # print("\n")
    # print(coord_arr_right_longitude[:10])
    # print(coord_arr_left_longitude[:10])
    # print(
    #     (sum(coord_arr_left_latitude) / len(coord_arr_left_latitude))
    #     - (sum(coord_arr_right_latitude) / len(coord_arr_right_latitude))
    # )
    # print(
    #     (sum(coord_arr_left_longitude) / len(coord_arr_right_longitude))
    #     - (sum(coord_arr_right_longitude) / len(coord_arr_right_longitude))
    # )
