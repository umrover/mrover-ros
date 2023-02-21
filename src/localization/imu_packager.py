#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from mrover.msg import ImuAndMag
from sensor_msgs.msg import Imu, MagneticField


class ImuPackager:
    """
    This node is for use in sim - it merges the /imu/imu_only & /imu/mag_only messages
    published by gazebo & publish to /imu/data, to replicate the behavior of imu_driver.py
    """

    def __init__(self):
        rospy.Subscriber("imu/imu_only", Imu, self.imu_callback)
        rospy.Subscriber("imu/mag_only", Vector3Stamped, self.mag_callback)

        self.imu_pub = rospy.Publisher("imu/data", ImuAndMag, queue_size=1)
        self.curr_mag = None

    def mag_pose(self, msg: Vector3Stamped):
        mag2d = np.array([msg.x, msg.y])
        rotationMatrix = np.array([[msg.x, -1 * msg.y], [msg.y, msg.x]])
        poseVec = rotationMatrix * mag2d

    def imu_callback(self, msg: Imu):
        if self.curr_mag is not None:
            self.imu_pub.publish(
                ImuAndMag(
                    header=msg.header,
                    imu=msg,
                    mag=MagneticField(header=self.curr_mag.header, magnetic_field=self.curr_mag.vector),
                )
            )

    def mag_callback(self, msg: Vector3Stamped):
        self.curr_mag = msg
        self.mag_pose(msg)


def main():
    rospy.init_node("imu_packager")
    ImuPackager()
    rospy.spin()


if __name__ == "__main__":
    main()
