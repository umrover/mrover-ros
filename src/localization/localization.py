#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import NavSatFix

from util.tf_utils import gps_to_world


def main():
    rospy.init_node("gps_to_odom")
    ref_gps_point: NavSatFix = rospy.get_param('ref_gps_point')
    tf_broadcaster = tf.TransformBroadcaster()

    def gps_callback(gps: NavSatFix):
        stamped_cartesian_transform = gps_to_world(gps, ref_gps_point, 'world')
        print(stamped_cartesian_transform)
        tf_broadcaster.sendTransformMessage(stamped_cartesian_transform)

    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
