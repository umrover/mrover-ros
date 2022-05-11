#!/usr/bin/env python3

import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('debug_course_publisher')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'course'
        t.transform.translation.x = 1
        t.transform.translation.y = 1
        t.transform.rotation.w = 1
        tf_broadcaster.sendTransform(t)
        time.sleep(0.1)
