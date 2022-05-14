#!/usr/bin/env python3

import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def send_waypoint(name: str, x: float, y: float):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'odom'
    t.child_frame_id = name
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.w = 1
    tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('debug_course_publisher')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        send_waypoint('course1', -10, -10)
        send_waypoint('course2', 20, -10)
        send_waypoint('course3', -20, -10)
        time.sleep(0.1)
