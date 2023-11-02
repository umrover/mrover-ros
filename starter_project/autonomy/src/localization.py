#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import tf2_ros

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3


class Localization:
    pose: SE3

    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu/imu_only", Imu, self.imu_callback)

        # create a transform broadcaster for publishing to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # initialize pose to all zeros
        self.pose = SE3()

    def gps_callback(self, msg: NavSatFix):
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordinates, store that value in `self.pose`, then publish
        that pose to the TF tree.
        """

        position = Localization.spherical_to_cartesian(
            np.array([msg.latitude, msg.longitude, 0]),
            np.array([42.275192064475604, -83.74166490288644, 0]),
        )

        self.pose = SE3(position, self.pose.rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")

        print(msg)

    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """

        self.pose = SE3.from_pos_quat(
            self.pose.position, np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        )

        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")

        print(msg)

    @staticmethod
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

        r = 6371000
        x = r * (spherical_coord[0] - reference_coord[0])
        y = r * (spherical_coord[1] - spherical_coord[1]) * np.cos(np.radians(reference_coord[1]))
        z = 0

        return np.array([y, -x, z])


def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()

    # rospy.loginfo(
    #     Localization.spherical_to_cartesian(
    #         np.array([42.293195, -83.7096706, 0]), np.array([42.275192064475604, -83.74166490288644, 0])
    #     ),
    # )

    rospy.loginfo()


if __name__ == "__main__":
    main()
