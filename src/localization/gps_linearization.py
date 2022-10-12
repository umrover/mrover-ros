#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from sensor_msgs.msg import NavSatFix, Imu
import tf2_ros
import numpy as np
from pymap3d.enu import geodetic2enu


class GPSLinearization:
    def __init__(self):
        rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # TODO: is this valid init state?
        self.pose = SE3()
        # TODO: what happens if these params dont exist? we want to throw an error
        self.ref_lat = rospy.get_param("reference_point_latitude")
        self.ref_lon = rospy.get_param("reference_point_longitude")
        self.ref_alt = rospy.get_param("reference_point_altitude")

    def gps_callback(self, msg: NavSatFix):
        geodetic = np.array([msg.latitude, msg.longitude])
        # TODO: should we/when should we add altitude?
        cartesian = geodetic2enu(
            msg.latitude, msg.longitude, msg.altitude, self.ref_lat, self.ref_lon, self.ref_alt, deg=True)
        self.pose = SE3(position=cartesian, rotation=self.pose.rotation)
        # self.pose = SE3.from_pos_quat(position=cartesian, rotation=self.pose.rotation.quaternion)
        # TODO: should the frames be params?
        # print(cartesian)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame="odom", child_frame="base_link")

    def imu_callback(self, msg: Imu):
        quaternion = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.pose = SE3.from_pos_quat(
            position=self.pose.position, quaternion=quaternion)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame="odom", child_frame="base_link")


def main():
    rospy.init_node("gps_linearization")
    gps_linearization = GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
