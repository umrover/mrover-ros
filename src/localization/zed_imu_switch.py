#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros


def main() -> None:
    rospy.init_node("zed_imu_switch")

    imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
    tf_pub = tf2_ros.TransformBroadcaster()

    tf_buffer = tf2_ros.Buffer()
    tf_sub = tf2_ros.TransformListener(tf_buffer)

    def zed_imu_callback(msg: Imu) -> None:
        imu_pub.publish(msg)

    rospy.Subscriber("/zed_imu/data_raw", Imu, zed_imu_callback)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            tf: TransformStamped = tf_buffer.lookup_transform("base_link_zed", "map", rospy.Time())
            tf.header.frame_id = "base_link"
            tf_pub.sendTransform(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        rate.sleep()

    rospy.spin()
