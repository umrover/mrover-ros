#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Temperature, Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from mrover.msg import CalibrationStatus, ImuAndMag

import serial
from serial import SerialException, SerialTimeoutException


def main():
    # publishers for all types of IMU data, queue size is 1 to make sure we don't publish old data
    imu_pub = rospy.Publisher("imu/data", ImuAndMag, queue_size=1)

    temp_pub = rospy.Publisher("imu/temp", Temperature, queue_size=1)
    calibration_pub = rospy.Publisher("imu/calibration", CalibrationStatus, queue_size=1)

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

        # fill in all sensor messages, setting timestamps of each message to right now,
        # and setting the reference frame of all messages to IMU frame
        imu_msg = ImuAndMag(
            header=Header(stamp=rospy.Time.now(), frame_id=imu_frame),
            imu=Imu(
                header=Header(stamp=rospy.Time.now(), frame_id=imu_frame),
                orientation=Quaternion(*orientation_data),
                linear_acceleration=Vector3(*accel_data),
                angular_velocity=Vector3(*gyro_data),
            ),
            mag=MagneticField(
                header=Header(stamp=rospy.Time.now(), frame_id=imu_frame),
                magnetic_field=Vector3(*mag_data),
            ),
        )

        temp_msg = Temperature(header=Header(stamp=rospy.Time.now(), frame_id=imu_frame), temperature=temp_data)

        calibration_msg = CalibrationStatus(Header(stamp=rospy.Time.now(), frame_id=imu_frame), *cal_data)

        # publish each message
        imu_pub.publish(imu_msg)
        temp_pub.publish(temp_msg)
        calibration_pub.publish(calibration_msg)


if __name__ == "__main__":
    main()
