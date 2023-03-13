#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from mrover.msg import ImuAndMag
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion
from std_msgs.msg import Header
import message_filters


class ImuPackager:
    """
    This node is for use in sim - it merges the /imu/imu_only & /imu/mag_only messages
    published by gazebo & publish to /imu/data, to replicate the behavior of imu_driver.py
    """

    imu_pub: rospy.Publisher
    mag_pose_pub: rospy.Publisher

    orientation_covariance: np.ndarray
    gyro_covariance: np.ndarray
    accel_covariance: np.ndarray
    mag_covariance: np.ndarray

    def __init__(self):
        self.orientation_covariance = np.array(rospy.get_param("global_ekf/imu_orientation_covariance"))
        self.gyro_covariance = np.array(rospy.get_param("global_ekf/imu_gyro_covariance"))
        self.accel_covariance = np.array(rospy.get_param("global_ekf/imu_accel_covariance"))
        self.mag_pose_covariance = np.array(rospy.get_param("global_ekf/imu_mag_pose_covariance"))

        imu_sub = message_filters.Subscriber("imu/imu_only", Imu)
        mag_sub = message_filters.Subscriber("imu/mag_only", Vector3Stamped)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, mag_sub], 10, 0.5)
        ts.registerCallback(self.imu_callback)

        self.imu_pub = rospy.Publisher("imu/data", ImuAndMag, queue_size=1)
        self.mag_pose_pub = rospy.Publisher("mag_pose/data", PoseWithCovarianceStamped, queue_size=1)

    def publish_mag_pose(self, msg: Vector3Stamped):

        # get unit magnetic field vector in the XY plane
        mag_vec = np.array([msg.vector.x, msg.vector.y])
        mag_vec = mag_vec / np.linalg.norm(mag_vec)

        # convert it to a rotation about the Z axis
        rotationMatrix = np.array(
            [[mag_vec[1], -1 * mag_vec[0], 0, 0], [mag_vec[0], mag_vec[1], 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        q = quaternion_from_matrix(rotationMatrix)

        # publish as a pose with the configured mag covariance matrix for rotation
        self.mag_pose_pub.publish(
            PoseWithCovarianceStamped(
                header=Header(stamp=msg.header.stamp, frame_id="map"),
                pose=PoseWithCovariance(
                    pose=Pose(orientation=Quaternion(*q)), covariance=self.mag_pose_covariance.flatten().tolist()
                ),
            )
        )

    def imu_callback(self, imu_msg: Imu, mag_msg: Vector3Stamped):
        imu_msg.orientation_covariance = self.orientation_covariance.flatten().tolist()
        imu_msg.angular_velocity_covariance = self.gyro_covariance.flatten().tolist()
        imu_msg.linear_acceleration_covariance = self.accel_covariance.flatten().tolist()

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
        self.publish_mag_pose(mag_msg)


def main():
    rospy.init_node("imu_packager")
    ImuPackager()
    rospy.spin()


if __name__ == "__main__":
    main()
