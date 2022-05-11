#!/usr/bin/env python3

import rospy
import tf
# from geodesy.utm import fromLatLong
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix, Imu
from util.tf_utils import gps_to_world

SIM_SAT_FIX = NavSatFix(latitude=42.199999689512715, longitude=-83.69999929025072, altitude=0.4652098556018964)


def main():
    print('===== localization starting =====')
    rospy.init_node("gps_to_odom")
    ref_gps_point: NavSatFix = rospy.get_param('ref_gps_point', SIM_SAT_FIX)
    tf_broadcaster = tf.TransformBroadcaster()

    orientation = Quaternion(0, 0, 0, 1)

    def gps_callback(gps: NavSatFix):
        stamped_cartesian_transform = gps_to_world(gps, ref_gps_point, "rover", "world")
        stamped_cartesian_transform.transform.rotation = orientation
        tf_broadcaster.sendTransformMessage(stamped_cartesian_transform)
        # utm = fromLatLong(gps.latitude, gps.longitude, gps.altitude)
        # tf_broadcaster.sendTransform(
        #     (utm.easting, utm.northing, utm.altitude),
        #     (0, 0, 0, 1),
        #     rospy.Time.now(),
        #     'rover',
        #     'base_link'
        # )

    # TODO: do we want to only publish a transform when we get a GPS callback, not an IMU callback?
    def imu_callback(imu: Imu):
        nonlocal orientation
        orientation = imu.orientation

    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
