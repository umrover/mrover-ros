#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
from mrover.msg import ImuAndMag
from sensor_msgs.msg import Imu, MagneticField


class ImuPackager:
    """
    This node is for use in sim - it merges the /imu/imu_only & /imu/mag_only messages
    published by gazebo & publish to /imu/data, to replicate the behavior of imu_driver.py
    """

    imu_pub: rospy.Publisher
    curr_mag: Vector3Stamped

    orientation_covariance: np.ndarray
    gyro_covariance: np.ndarray
    accel_covariance: np.ndarray
    mag_covariance: np.ndarray

    def __init__(self):
        self.orientation_covariance = np.array(rospy.get_param("global_ekf/imu_orientation_covariance"))
        self.gyro_covariance = np.array(rospy.get_param("global_ekf/imu_gyro_covariance"))
        self.accel_covariance = np.array(rospy.get_param("global_ekf/imu_accel_covariance"))
        self.mag_covariance = np.array(rospy.get_param("global_ekf/imu_mag_covariance"))

        rospy.Subscriber("imu/imu_only", Imu, self.imu_callback)
        rospy.Subscriber("imu/mag_only", Vector3Stamped, self.mag_callback)

        self.imu_pub = rospy.Publisher("imu/data", ImuAndMag, queue_size=1)
        self.curr_mag = None

    def imu_callback(self, msg: Imu):
        msg.orientation_covariance = self.orientation_covariance.flatten().tolist()
        msg.angular_velocity_covariance = self.gyro_covariance.flatten().tolist()
        msg.linear_acceleration_covariance = self.accel_covariance.flatten().tolist()

        if self.curr_mag is not None:
            self.imu_pub.publish(
                ImuAndMag(
                    header=msg.header,
                    imu=msg,
                    mag=MagneticField(
                        header=self.curr_mag.header,
                        magnetic_field=self.curr_mag.vector,
                        magnetic_field_covariance=self.mag_covariance.flatten().tolist(),
                    ),
                )
            )

    def mag_callback(self, msg: Vector3Stamped):
        self.curr_mag = msg


def main():
    rospy.init_node("imu_packager")
    ImuPackager()
    rospy.spin()


if __name__ == "__main__":
    main()
