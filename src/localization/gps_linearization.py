#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from sensor_msgs.msg import NavSatFix, Imu
import tf2_ros
import numpy as np
from pymap3d.enu import geodetic2enu


class GPSLinearization:
    def __init__(self):
        # subscribe to the topics containing GPS and IMU data,
        # assigning them our corresponding callback functions
        rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)

        # create a transform broadcaster so we can publish to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # TODO: is this valid init state?
        self.pose = SE3()

        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")

        self.world_frame = rospy.get_param("gps_linearization/world_frame")
        # TODO: account for separate GPS and IMU frames?
        self.rover_frame = rospy.get_param("gps_linearization/rover_frame")

    def gps_callback(self, msg: NavSatFix):
        """
        Callback function that receives GPS messages, linearizes them,
        updates the rover pose, and publishes it to the TF tree.

        :param msg: The NavSatFix message containing GPS data that was just received
        """
        cartesian = np.array(geodetic2enu(
            msg.latitude, msg.longitude, msg.altitude, self.ref_lat, self.ref_lon, self.ref_alt, deg=True
        ))
        cartesian[2] = 0
        self.pose = SE3(position=cartesian, rotation=self.pose.rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame=self.world_frame, child_frame=self.rover_frame)

    def imu_callback(self, msg: Imu):
        """
        Callback function that receives IMU messages, updates the rover pose,
        and publishes it to the TF tree.

        :param msg: The Imu message containing IMU data that was just received
        """
        quaternion = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.pose = SE3.from_pos_quat(position=self.pose.position, quaternion=quaternion)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame=self.world_frame, child_frame=self.rover_frame)


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization")
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
