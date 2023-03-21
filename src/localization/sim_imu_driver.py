#!/usr/bin/env python3
import rospy
import numpy as np
from mrover.msg import ImuAndMag
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
import message_filters
from typing import List
from esw.imu_driver import get_covariances, publish_mag_pose


class SimIMUDriver:
    """
    This node is for use in sim - it merges the /imu/imu_only & /imu/mag_only messages
    published by gazebo & publish to /imu/data, to replicate the behavior of imu_driver.py
    """

    imu_pub: rospy.Publisher
    mag_pose_pub: rospy.Publisher

    orientation_covariance: List
    gyro_covariance: List
    accel_covariance: List
    mag_pose_covariance: List

    world_frame: str

    def __init__(self):
        (
            self.orientation_covariance,
            self.gyro_covariance,
            self.accel_covariance,
            self.mag_pose_covariance,
        ) = get_covariances()
        self.world_frame = rospy.get_param("world_frame")

        imu_sub = message_filters.Subscriber("imu/imu_only", Imu)
        mag_sub = message_filters.Subscriber("imu/mag_only", Vector3Stamped)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, mag_sub], 10, 0.5)
        ts.registerCallback(self.imu_callback)

        self.imu_pub = rospy.Publisher("imu/data", ImuAndMag, queue_size=1)
        self.mag_pose_pub = rospy.Publisher("imu/mag_pose", PoseWithCovarianceStamped, queue_size=1)

    def imu_callback(self, imu_msg: Imu, mag_vector_msg: Vector3Stamped):
        imu_msg.orientation_covariance = self.orientation_covariance
        imu_msg.angular_velocity_covariance = self.gyro_covariance
        imu_msg.linear_acceleration_covariance = self.accel_covariance

        mag_msg = MagneticField(header=mag_vector_msg.header, magnetic_field=mag_vector_msg.vector)

        self.imu_pub.publish(
            ImuAndMag(header=imu_msg.header, imu=imu_msg, mag=mag_msg),
        )
        publish_mag_pose(self.mag_pose_pub, mag_msg, self.mag_pose_covariance, self.world_frame)


def main():
    rospy.init_node("imu_packager")
    SimIMUDriver()
    rospy.spin()


if __name__ == "__main__":
    main()
