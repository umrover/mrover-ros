#!/usr/bin/env python3

import traceback

import board
import busio

import rospy
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from geometry_msgs.msg import Quaternion, Vector3
from mrover.msg import CalibrationStatus
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

BN0085_I2C_ADDRESS = 0x4A


def main() -> None:
    rospy.init_node("imu_driver")

    rospy.loginfo("IMU I2C driver starting...")

    frame_id = rospy.get_param("imu_driver/frame_id")
    update_rate = rospy.get_param("imu_driver/update_rate", 30)

    calib_imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=1)
    uncalib_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
    mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=1)
    calib_pub = rospy.Publisher("/imu/calibration", CalibrationStatus, queue_size=1)

    rospy.loginfo("Initializing IMU I2C connection...")

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c, address=BN0085_I2C_ADDRESS)

    rospy.loginfo("Starting IMU dynamic calibration...")

    bno.begin_calibration()

    rospy.loginfo("Configuring IMU reports...")

    try:
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        # Orientation with no reference heading
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    except Exception as e:
        # TODO: Toggle reset pin
        rospy.logwarn(f"Failed to enable all features: {e}, retrying...")
        rospy.logwarn(traceback.format_exc())
        return

    rospy.loginfo("IMU armed")

    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        header = Header(stamp=rospy.Time.now(), frame_id=frame_id)

        try:
            calib_imu_pub.publish(
                Imu(
                    header=header,
                    orientation=Quaternion(*bno.quaternion),
                    angular_velocity=Vector3(*bno.gyro),
                    linear_acceleration=Vector3(*bno.acceleration),
                ),
            )

            uncalib_pub.publish(
                Imu(
                    header=header,
                    orientation=Quaternion(*bno.game_quaternion),
                )
            )

            mag_pub.publish(MagneticField(header=header, magnetic_field=Vector3(*bno.magnetic)))

            _, c2, c3 = bno.calibration_status
            # c1, c2, c3 = bno.calibration_status


            calib_pub.publish(CalibrationStatus(header, 0, c2, c3))
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn(traceback.format_exc())
            start = rospy.Time.now()
            while rospy.Time.now() - start < rospy.Duration(1):
                try:
                    rospy.loginfo_throttle(1, "Attempting to re-enable readings...")
                    bno._readings.clear()
                    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                    bno.enable_feature(BNO_REPORT_GYROSCOPE)
                    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
                    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
                    rospy.loginfo("Restarted!")
                    break
                except:
                    rospy.logwarn_throttle(2, "Re-enable failed...")
                    pass
            else:
                rospy.logfatal("Failed to restart IMU driver, exiting...")
                break

        rate.sleep()


if __name__ == "__main__":
    main()
