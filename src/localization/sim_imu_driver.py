#!/usr/bin/env python3
import rospy
import numpy as np
from mrover.msg import ImuAndMag
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
import message_filters
from esw.imu_driver import get_covariances, inject_covariances, publish_mag_pose


class SimIMUDriver:
    """
    This node is for use in sim - it merges the /imu/imu_only & /imu/mag_only messages
    published by gazebo & publish to /imu/data, to replicate the behavior of imu_driver.py
    """

    imu_pub: rospy.Publisher
    mag_pose_pub: rospy.Publisher

    orientation_covariance: np.ndarray
    gyro_covariance: np.ndarray
    accel_covariance: np.ndarray
    mag_pose_covariance: np.ndarray

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

    def imu_callback(self, imu_msg: Imu, mag_msg: Vector3Stamped):
        imu_msg = inject_covariances(imu_msg, self.orientation_covariance, self.gyro_covariance, self.accel_covariance)
        self.imu_pub.publish(
            ImuAndMag(
                header=imu_msg.header,
                imu=imu_msg,
                mag=MagneticField(
                    header=mag_msg.header,
                    magnetic_field=mag_msg.vector,
                ),
            )
        )
        publish_mag_pose(self.mag_pose_pub, mag_msg, self.mag_pose_covariance, self.world_frame)


def main():
    rospy.init_node("imu_packager")
    SimIMUDriver()
    rospy.spin()


if __name__ == "__main__":
    main()
