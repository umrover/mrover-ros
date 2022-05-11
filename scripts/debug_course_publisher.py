#!/usr/bin/env python3

import time

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('debug_course_publisher')
    tf_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        tf_broadcaster.sendTransform(
            (1, 1, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            'course',
            'base_link'
        )
        time.sleep(0.1)
