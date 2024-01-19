#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from util.SE3 import SE3
from pymap3d.enu import enu2geodetic

class GPSSim:
    latest_odom: Odometry = None
    ref_point: np.ndarray
    right_gps_pub: rospy.Publisher
    left_gps_pub: rospy.Publisher
    ground_truth_sub: rospy.Subscriber
    
    def __init__(self):
        ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")
        self.ref_point = np.array([ref_lat, ref_lon, ref_alt])
        
        self.right_gps_pub = rospy.Publisher("right_gps_driver/fix", NavSatFix, queue_size=10)
        self.left_gps_pub = rospy.Publisher("left_gps_driver/fix", NavSatFix, queue_size=10)
        self.ground_truth_sub = rospy.Subscriber("ground_truth", Odometry, self.ground_truth_callback)
    
    def ground_truth_callback(self, msg: Odometry):
        self.latest_odom = msg

    def publish_gps(self):
        if self.latest_odom is None:
            return
        
        # position of each GPS antenna relative to base_link, hardcoded for now
        left_gps_offset = np.array([0, 1, 0])
        right_gps_offset = np.array([0, -1, 0])

        # get the homogeneous transform matrix from base_link to map from the latest ground truth reading
        pose = self.latest_odom.pose.pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        rover_in_map = SE3.from_pos_quat(p, q)
        rover_to_map = rover_in_map.transform_matrix()

        # calculate the position of each GPS antenna in the map frame
        left_gps_in_map = rover_to_map @ np.append(left_gps_offset, 1)
        right_gps_in_map = rover_to_map @ np.append(right_gps_offset, 1)
        
        # convert GPS positions to geodetic coordinates
        (left_lat, left_lon, left_alt) = enu2geodetic(*left_gps_in_map[:3], *self.ref_point, deg=True)
        (right_lat, right_lon, right_alt) = enu2geodetic(*right_gps_in_map[:3], *self.ref_point, deg=True)

        # publish GPS messages
        left_msg = NavSatFix()
        left_msg.header.stamp = rospy.Time.now()
        left_msg.latitude = left_lat
        left_msg.longitude = left_lon
        left_msg.altitude = left_alt
        self.left_gps_pub.publish(left_msg)

        right_msg = NavSatFix()
        right_msg.header.stamp = rospy.Time.now()
        right_msg.latitude = right_lat
        right_msg.longitude = right_lon
        right_msg.altitude = right_alt
        self.right_gps_pub.publish(right_msg)
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_gps()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("temp_gps_sim")
    node = GPSSim()
    node.run()
