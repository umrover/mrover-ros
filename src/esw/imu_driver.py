#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from mrover.msg import CalibrationStatus

import serial
from serial import SerialException


def main():
    # publishers for all types of IMU data, queue size is 1 to make sure we don't publish old data
    imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
    mag_pub = rospy.Publisher("magnetometer", MagneticField, queue_size=1)
    temp_pub = rospy.Publisher("imu_temp", Temperature, queue_size=1)
    calibration_pub = rospy.Publisher("imu_calibration", CalibrationStatus, queue_size=1)

    # no rate used because rate is effectively set by Arduino serial publisher
    rospy.init_node("imu_driver")

    # read serial connection info and IMU frame from parameter server
    port = rospy.get_param("imu_driver/port", "/dev/imu")
    baud = rospy.get_param("imu_driver/baud", 115200)
    imu_frame = rospy.get_param("imu_driver/frame_id", "imu_link")

    # create serial connection with Arduino
    ser = serial.Serial(port, baud)
    attempts = 0

    # initialize messages for IMU data
    imu_msg = Imu()
    mag_msg = MagneticField()
    temp_msg = Temperature()
    calibration_msg = CalibrationStatus()

    while not rospy.is_shutdown():

        # try to read a line from the serial connection,
        # if this fails 100 times in a row then end the program
        try:
            line = ser.readline()
            attempts = 0
        except SerialException:
            attempts += 1
            if attempts > 100:
                print("unable to read from serial port...ending program")
                break

        # parse data into a list of floats,
        # if it fails to parse then skip this message
        try:
            data = [float(val.strip()) for val in line.split()]

        except ValueError:
            data = []
            print("invalid msg format")
            continue

        # load parsed data into ROS messages,
        # if it fails to get the data then skip this message
        try:
            # orientation
            imu_msg.orientation.x = data[0]
            imu_msg.orientation.y = data[1]
            imu_msg.orientation.z = data[2]
            imu_msg.orientation.w = data[3]

            # acceleration
            imu_msg.linear_acceleration.x = data[4]
            imu_msg.linear_acceleration.y = data[5]
            imu_msg.linear_acceleration.z = data[6]

            # angular velocity
            imu_msg.angular_velocity.x = data[7]
            imu_msg.angular_velocity.y = data[8]
            imu_msg.angular_velocity.z = data[9]
            mag_msg.magnetic_field.x = data[10]
            mag_msg.magnetic_field.y = data[11]
            mag_msg.magnetic_field.z = data[12]

            # temperature
            temp_msg.temperature = data[13]

            # calibration status, convert to ints
            calibration_msg.system_calibration = int(data[14])
            calibration_msg.gyroscope_calibration = int(data[15])
            calibration_msg.accelerometer_calibration = int(data[16])
            calibration_msg.magnetometer_calibration = int(data[17])

        except IndexError:
            print("incomplete msg")
            continue

        # set timestamps of each message to right now
        imu_msg.header.stamp = rospy.Time.now()
        mag_msg.header.stamp = rospy.Time.now()
        temp_msg.header.stamp = rospy.Time.now()
        calibration_msg.header.stamp = rospy.Time.now()

        # set the reference frame of all messages to IMU frame
        imu_msg.header.frame_id = imu_frame
        mag_msg.header.frame_id = imu_frame
        temp_msg.header.frame_id = imu_frame
        calibration_msg.header.frame_id = imu_frame

        # publish each message
        imu_pub.publish(imu_msg)
        mag_pub.publish(mag_msg)
        temp_pub.publish(temp_msg)
        calibration_pub.publish(calibration_msg)


if __name__ == "__main__":
    main()
