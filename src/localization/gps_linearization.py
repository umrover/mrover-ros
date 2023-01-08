#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from sensor_msgs.msg import NavSatFix, Imu
import tf2_ros
import numpy as np
from pymap3d.enu import geodetic2enu
from tf.transformations import quaternion_about_axis, quaternion_multiply


class GPSLinearization:
    """
    This node subscribes to GPS and IMU data, linearizes the GPS data
    into ENU coordinates, then combines the linearized GPS position and the IMU
    orientation into a pose estimate for the rover and publishes it to the TF tree.
    """

    def __init__(self):
        rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu/data", Imu, self.imu_callback)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # init to zero pose
        self.pose = SE3()

        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")

        self.world_frame = rospy.get_param("gps_linearization/world_frame")
        self.rover_frame = rospy.get_param("gps_linearization/rover_frame")

    def gps_callback(self, msg: NavSatFix):
        """
        Callback function that receives GPS messages, linearizes them,
        updates the rover pose, and publishes it to the TF tree.

        :param msg: The NavSatFix message containing GPS data that was just received
        """
        cartesian = np.array(
            geodetic2enu(msg.latitude, msg.longitude, msg.altitude, self.ref_lat, self.ref_lon, self.ref_alt, deg=True)
        )
        if np.any(np.isnan(cartesian)):
            return

        # ignore Z
        cartesian[2] = 0
        self.pose = SE3(position=cartesian, rotation=self.pose.rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame=self.world_frame, child_frame=self.rover_frame)

    def imu_callback(self, msg: Imu):
        """
        Callback function that receives IMU messages, updates the rover pose,
        and publishes it to the TF tree.

        :param msg: The Imu message containing IMU data that was just received
        """
        # convert ROS msg quaternion to numpy array
        imu_quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        # normalize to avoid rounding errors
        imu_quat = imu_quat / np.linalg.norm(imu_quat)

        # get a quaternion to rotate about the Z axis by 90 degrees
        offset_quat = quaternion_about_axis(np.pi / 2, np.array([0, 0, 1]))

        # rotate the IMU quaternion by the offset to convert it to the ENU frame
        enu_quat = quaternion_multiply(offset_quat, imu_quat)
        self.pose = SE3.from_pos_quat(position=self.pose.position, quaternion=enu_quat)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame=self.world_frame, child_frame=self.rover_frame)


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization")
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
