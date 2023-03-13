#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Temperature, Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped, PoseWithCovariance, Vector3Stamped, Pose
from std_msgs.msg import Header
from mrover.msg import CalibrationStatus, ImuAndMag
from tf.transformations import quaternion_about_axis, quaternion_multiply, rotation_matrix, quaternion_from_matrix
from typing import Tuple

import serial
from serial import SerialException, SerialTimeoutException


def get_covariances() -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Get the IMU covariances for orientation, gyroscope, accelerometer, and magnetometer pose
    from rosparam.

    :returns: a tuple containing the orientation, gyro, accel, and mag pose covariance matrices in order
    """
    orientation = np.array(rospy.get_param("global_ekf/imu_orientation_covariance"))
    gyro = np.array(rospy.get_param("global_ekf/imu_gyro_covariance"))
    accel = np.array(rospy.get_param("global_ekf/imu_accel_covariance"))
    mag_pose = np.array(rospy.get_param("global_ekf/imu_mag_pose_covariance"))
    return (orientation, gyro, accel, mag_pose)


def publish_mag_pose(pub: rospy.Publisher, msg: Vector3Stamped, covariance: np.ndarray, frame: str):
    """
    Estimates the rover's yaw angle from the magnetometer measurement and publishes it
    as a PoseWithCovarianceStamped for use in the EKF.

    :param pub: PoseWithCovarianceStamped publisher used to publish the message
    :param msg: magnetic field message used to estimate the yaw angle
    :param covariance: 6x6 covariance matrix for the estimated pose,
                       each row/col is [x, y, z, roll, pitch, yaw]
    :param frame: the frame in which to publish the estimated pose
    """

    # get unit magnetic field vector in the XY plane
    mag_vec = np.array([msg.vector.x, msg.vector.y])
    mag_vec = mag_vec / np.linalg.norm(mag_vec)

    # convert it to a rotation about the Z axis
    rotationMatrix = np.array(
        [[mag_vec[1], -1 * mag_vec[0], 0, 0], [mag_vec[0], mag_vec[1], 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )
    q = quaternion_from_matrix(rotationMatrix)

    # publish as a pose with the configured mag covariance matrix for rotation
    pub.publish(
        PoseWithCovarianceStamped(
            header=Header(stamp=msg.header.stamp, frame_id=frame),
            pose=PoseWithCovariance(pose=Pose(orientation=Quaternion(*q)), covariance=covariance.flatten().tolist()),
        )
    )


def inject_covariances(imu_msg: Imu, orientation_cov: np.ndarray, gyro_cov: np.ndarray, accel_cov: np.ndarray):
    """
    Inserts the given covariance matrices into the given IMU message, modifying it in place.

    :param imu_msg: the IMU message to add covariances to
    :param orientation_cov: 3x3 orientation covariance matrix [roll, pitch, yaw]
    :param gyro_cov: 3x3 gyroscope (angular vel) covariance matrix [roll, pitch, yaw]
    :param accel_cov: 3x3 accelerometer (linear accel) covariance matrix [x, y, z]
    """
    imu_msg.orientation_covariance = orientation_cov.flatten().tolist()
    imu_msg.angular_velocity_covariance = gyro_cov.flatten().tolist()
    imu_msg.linear_acceleration_covariance = accel_cov.flatten().tolist()


def main():
    """
    This program reads in IMU data over serial, converts it to the ENU frame,
    and adds necessary metadata used for filtering.
    """
    orientation_covariance, gyro_covariance, accel_covariance, mag_pose_covariance = get_covariances()
    world_frame = rospy.get_param("world_frame")

    # publishers for all types of IMU data, queue size is 1 to make sure we don't publish old data
    imu_pub = rospy.Publisher("imu/data", ImuAndMag, queue_size=1)
    temp_pub = rospy.Publisher("imu/temp", Temperature, queue_size=1)
    calibration_pub = rospy.Publisher("imu/calibration", CalibrationStatus, queue_size=1)
    mag_pose_pub = rospy.Publisher("imu/mag_pose", PoseWithCovarianceStamped, queue_size=1)

    # no rate used because rate is effectively set by Arduino serial publisher
    rospy.init_node("imu_driver")

    # read serial connection info and IMU frame from parameter server
    port = rospy.get_param("imu_driver/port", "/dev/imu")
    baud = rospy.get_param("imu_driver/baud", 115200)
    imu_frame = rospy.get_param("imu_driver/frame_id", "imu_link")

    # create serial connection with Arduino
    ser = serial.Serial(port, baud)
    attempts = 0

    while not rospy.is_shutdown():

        # try to read a line from the serial connection,
        # if this fails 100 times in a row then end the program
        try:
            line = ser.readline()
            attempts = 0
        except SerialException:
            attempts += 1
            if attempts > 100:
                raise SerialTimeoutException(f"Unable to read from serial port {port}, timed out after 100 attempts")

        # parse data into a list of floats,
        # if it fails to parse then skip this message
        try:
            data = [float(val.strip()) for val in line.split()]

        except ValueError:
            rospy.logerr("invalid msg format")
            continue

        # partition data into different sensors, converting calibration data from float to int
        # if the indices of data don't exist, then skip this message
        try:
            orientation_data = data[:4]
            accel_data = data[4:7]
            gyro_data = data[7:10]
            mag_data = data[10:13]
            temp_data = data[13]
            cal_data = [int(n) for n in data[14:18]]

        except IndexError:
            rospy.logerr("incomplete msg")
            continue

        # rotate the quaternion and mag vector by 90 degrees about the Z axis to convert it to ENU frame
        offset_quat = quaternion_about_axis(np.pi / 2, [0, 0, 1])
        enu_orientation = quaternion_multiply(offset_quat, orientation_data)
        R = rotation_matrix(np.pi / 2, [0, 0, 1])
        enu_mag_vec = R @ np.array(mag_data)

        # fill in all sensor messages, setting timestamps of each message to right now,
        # and setting the reference frame of all messages to IMU frame
        header = Header(stamp=rospy.Time.now(), frame_id=imu_frame)
        mag_msg = (
            MagneticField(
                header=header,
                magnetic_field=Vector3(*enu_mag_vec),
            ),
        )
        imu_msg = (
            Imu(
                header=header,
                orientation=Quaternion(*enu_orientation),
                linear_acceleration=Vector3(*accel_data),
                angular_velocity=Vector3(*gyro_data),
            ),
        )
        inject_covariances(imu_msg, orientation_covariance, gyro_covariance, accel_covariance)

        imu_msg = ImuAndMag(
            header=header,
            imu=imu_msg,
            mag=mag_msg,
        )

        temp_msg = Temperature(header=header, temperature=temp_data)

        calibration_msg = CalibrationStatus(header, *cal_data)

        # publish each message
        imu_pub.publish(imu_msg)
        temp_pub.publish(temp_msg)
        calibration_pub.publish(calibration_msg)
        publish_mag_pose(mag_pose_pub, mag_msg, mag_pose_covariance, world_frame)


if __name__ == "__main__":
    main()
