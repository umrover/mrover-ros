#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry


def main():
    print('===== localization starting =====')
    rospy.init_node("gps_to_tf")
    tf2_broadcaster = tf2_ros.TransformBroadcaster()

    def odom_callback(odom: Odometry):
        pose = odom.pose.pose
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rover'
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        tf2_broadcaster.sendTransform(t)

    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
