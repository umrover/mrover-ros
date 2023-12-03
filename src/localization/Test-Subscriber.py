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

# coord_arr_left_latitude = []
# coord_arr_left_longitude = []
# coord_arr_right_latitude = []
# coord_arr_right_longitude = []
coord_arr_left = np.array()
coord_arr_right = np.array()
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

    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
        """
        This is a utility function that should convert spherical (latitude, longitude)
        coordinates into cartesian (x, y, z) coordinates using the specified reference point
        as the center of the tangent plane used for approximation.
        :param spherical_coord: the spherical coordinate to convert,
                                given as a numpy array [latitude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a numpy array [latitude, longitude]
        :returns: the approximated cartesian coordinates in meters, given as a numpy array [x, y, z]
        """
        crc = 6371000
        longDist = (
            crc
            * (np.radians(spherical_coord[1]) - np.radians(reference_coord[1]))
            * np.cos(np.radians(reference_coord[0]))
        )
        latDist = crc * (np.radians(spherical_coord[0]) - np.radians(reference_coord[0]))
        z = 0
        return np.array([longDist, latDist])

    def gps_left_callback(self, msg: NavSatFix):
        # print(msg)
        # coord_arr_left_latitude.append(gps_linearization.geodetic2enu(msg.latitude, msg.longitude)
        # coord_arr_left_longitude.append(msg.longitude)
        print(self.spherical_to_cartesian(np.array([msg.latitude, msg.longitude]), np.array([42.293195, -83.7096706])))
        # print(np.array([42.293195, -83.7096706]))
        coord_arr_left.append(
            self.spherical_to_cartesian(np.array([msg.latitude, msg.longitude]), np.array([42.293195, -83.7096706]))
        )

        # lat_arr.append(msg.latitude)
        # long_arr.append(msg.longitude)

    def gps_right_callback(self, msg: NavSatFix):
        print(msg)
        # coord_arr_right.append(
        #     self.spherical_to_cartesian(np.array([msg.latitude, msg.longitude]), np.array([42.293195, -83.7096706]))
        # )
        # coord_arr_right_latitude.append(msg.latitude)
        # coord_arr_right_longitude.append(msg.longitude)

    def main():
        # initialize the node
        rospy.init_node("localization_test")

        # create and start our localization system
        localization = Localization()

        # let the callback functions run asynchronously in the background
        rospy.spin()


if __name__ == "__main__":
    main()

    # array_left = np.array(coord_arr_left)
    # array_right = np.array(coord_arr_right)
    # for i in range(min(len(coord_arr_left_latitude), len(coord_arr_right_latitude))):
    # plt.plot(coord_arr_left_latitude[i], coord_arr_left_longitude[i], color="red")
    # plt.plot(coord_arr_right_latitude[i], coord_arr_right_longitude[i], color="blue")
    # plt.show()
    # print("here")
    # distances[i] = math.dist([coord_arr_left_latitude])
    plt.scatter(coord_arr_left, color="red")
    plt.scatter(coord_arr_right, color="blue")
    plt.show()
