#!/usr/bin/env python3

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

import rospy
from geometry_msgs.msg import Quaternion, Vector3
from mrover.msg import ImuAndMag, CalibrationStatus
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

BN0085_I2C_ADDRESS = 0x4A


def main() -> None:
    rospy.init_node("imu_driver")

    imu_pub = rospy.Publisher("/imu/data", ImuAndMag, queue_size=1)
    calib_pub = rospy.Publisher("/imu/calibration", CalibrationStatus, queue_size=1)

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c, address=BN0085_I2C_ADDRESS)

    all_done = False

    while not all_done and not rospy.is_shutdown():
        try:
            bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            bno.enable_feature(BNO_REPORT_GYROSCOPE)
            bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            all_done = True
        except:
            rospy.logwarn("Failed to enable all features, retrying...")

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        header = Header(stamp=rospy.Time.now(), frame_id="imu_link")
        imu_pub.publish(
            ImuAndMag(
                header,
                Imu(
                    header=header,
                    orientation=Quaternion(*bno.quaternion),
                    angular_velocity=Vector3(*bno.gyro),
                    linear_acceleration=Vector3(*bno.linear_acceleration),
                ),
                MagneticField(header=header, magnetic_field=Vector3(*bno.magnetic)),
            )
        )

        calib_pub.publish(CalibrationStatus(header, bno.calibration_status))

        rate.sleep()


if __name__ == "__main__":
    main()
