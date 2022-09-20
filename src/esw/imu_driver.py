#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from mrover.msg import CalibrationStatus
import serial
from serial import SerialException


def main():
    rospy.init_node("imu_driver")
    imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
    mag_pub = rospy.Publisher("magnetometer", MagneticField, queue_size=1)
    temp_pub = rospy.Publisher("imu_temp", Temperature, queue_size=1)
    calibration_pub = rospy.Publisher("imu_calibration", CalibrationStatus, queue_size=1)

    # TODO: does this thing need a specified rate?
    # TODO: figure out why this thing wont stop with ctrl c
    port = rospy.get_param("imu_driver/port", "/dev/imu")
    baud = rospy.get_param("imu_driver/baud", 115200)
    imu_frame = rospy.get_param("imu_driver/frame_id", "imu_link")

    ser = serial.Serial(port, baud)
    attempts = 0

    while True:
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()
        calibration_msg = CalibrationStatus()

        try:
            line = ser.readline()
            attempts = 0
        except SerialException:
            attempts += 1
            if attempts > 100:
                print("unable to read from serial port...ending program")
                break

        try:
            # parse data into a list of floats
            vals = [float(val.strip()) for val in line.split()]

            # convert calibration values to ints
            vals[-4:] = [int(val) for val in vals[-4:]]
        except ValueError:
            vals = []
            print("invalid msg format")

        # print("vals: ", vals)

        # TODO: should we assign timestamps in the arduino driver and send them over serial with the rest of the data?
        # TODO: add calibration msg
        # TODO: figure out if linear acceleration is different from our acceleration

        # format: orientation_x, orientation_y, orientation_z, orientation_w,
        #         accel_x, accel_y, accel_z,
        #         gyro_x, gyro_y, gyro_z,
        #         mag_x, mag_y, mag_z,
        #         temp,
        #         sys_cal, gyro_cal, accel_cal, mag_cal
        try:
            (
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w,
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z,
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z,
                mag_msg.magnetic_field.x,
                mag_msg.magnetic_field.y,
                mag_msg.magnetic_field.z,
                temp_msg.temperature,
                calibration_msg.system_calibration,
                calibration_msg.gyroscope_calibration,
                calibration_msg.accelerometer_calibration,
                calibration_msg.magnetometer_calibration,
            ) = vals

        except (IndexError, ValueError):
            print("incomplete msg")

        imu_msg.header.stamp = rospy.Time.now()
        mag_msg.header.stamp = rospy.Time.now()
        temp_msg.header.stamp = rospy.Time.now()
        calibration_msg.header.stamp = rospy.Time.now()

        imu_msg.header.frame_id = imu_frame
        mag_msg.header.frame_id = imu_frame
        temp_msg.header.frame_id = imu_frame
        calibration_msg.header.frame_id = imu_frame

        imu_pub.publish(imu_msg)
        mag_pub.publish(mag_msg)
        temp_pub.publish(temp_msg)
        calibration_pub.publish(calibration_msg)


if __name__ == "__main__":
    main()
