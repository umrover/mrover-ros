#!/usr/bin/env python3

import sys
import unittest

import numpy as np

import rospy
import rostest
import tf2_ros
from mrover.msg import Waypoint, WaypointType, LED
from util.SE3 import SE3
from util.course_publish_helpers import publish_waypoints, convert_waypoint_to_gps

COMPLETION_TOLERANCE = 3.0


class TestIntegration(unittest.TestCase):
    def test_integration(self):
        rospy.init_node("integration_test")

        rospy.loginfo("Autonomy integration test starting...")

        waypoints = [
            (
                Waypoint(tag_id=0, type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([4, 4, 0])),
            ),
            (
                Waypoint(tag_id=0, type=WaypointType(val=WaypointType.POST)),
                SE3(position=np.array([-2, -2, 0])),
            ),
        ]
        publish_waypoints(list(map(convert_waypoint_to_gps, waypoints)))

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        # def led_callback(msg: LED):
        #     rospy.loginfo(f"LED message received: {msg}")
        #
        # rospy.Subscriber("/led", LED, led_callback)

        rate = rospy.Rate(20)
        for waypoint in waypoints:
            waypoint_data, pose = waypoint
            rospy.loginfo(f"Moving to waypoint: {waypoint}")

            def distance_to_target():
                try:
                    rover_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="base_link")
                    if waypoint_data.type == WaypointType(val=WaypointType.NO_SEARCH):
                        waypoint_in_world = pose
                    elif waypoint_data.type == WaypointType(val=WaypointType.POST):
                        waypoint_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame=f"tag_{waypoint_data.tag_id}_truth")
                    elif waypoint_data.type == WaypointType(val=WaypointType.MALLET):
                        waypoint_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="mallet_truth")
                    elif waypoint_data.type == WaypointType(val=WaypointType.WATER_BOTTLE):
                        waypoint_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="bottle_truth")
                    else:
                        raise ValueError(f"Unknown waypoint type: {waypoint_data.type}")
                    distance_to_target = waypoint_in_world.pos_distance_to(rover_in_world)
                    rospy.logdebug(distance_to_target)
                    return distance_to_target
                except tf2_ros.TransformException as e:
                    rospy.logwarn_throttle(1, f"Transform exception: {e}")
                    return float("inf")

            while distance_to_target() > COMPLETION_TOLERANCE:
                if rospy.is_shutdown():
                    return
                rospy.loginfo_throttle(1, f"Distance to current waypoint: {distance_to_target()}")
                rate.sleep()

            rospy.loginfo("Waypoint reached")

        rospy.signal_shutdown("Test Complete")


if __name__ == "__main__":
    rostest.rosrun("mrover", "integration_test", TestIntegration, sys.argv)
