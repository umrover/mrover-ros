#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from mrover.msg import ImuAndMag
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion
from std_msgs.msg import Header


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
        self.mag_pose_pub = rospy.Publisher("mag_pose/data", PoseWithCovarianceStamped, queue_size=1)
        self.curr_mag = None

    def mag_pose(self, msg: Vector3Stamped):
        vec_mag = np.array([msg.vector.x, msg.vector.y])
        norm_vec = vec_mag / np.linalg.norm(vec_mag)
        rotationMatrix = np.array([[norm_vec[1], -1 * norm_vec[0], 0, 0], [norm_vec[0], norm_vec[1], 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        q = quaternion_from_matrix(rotationMatrix)
        self.mag_pose_pub.publish(
            PoseWithCovarianceStamped(
                header=Header(stamp=msg.header.stamp, frame_id="map"),
                pose=PoseWithCovariance(pose=Pose(orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
            )
        )

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
        # this should return a quaternion
        self.curr_mag = msg
        self.mag_pose(msg)


def main():
    rospy.init_node("imu_packager")
    ImuPackager()
    rospy.spin()


if __name__ == "__main__":
    main()
