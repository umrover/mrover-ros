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
            (
                Waypoint(tag_id=1, type=WaypointType(val=WaypointType.POST)),
                SE3(position=np.array([11, -10, 0])),
            )
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
            rospy.loginfo(f"Moving to waypoint: {waypoint}")

            def distance_to_current_waypoint():
                try:
                    rover_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="base_link")
                    waypoint_in_world = waypoint[1]
                    distance_to_target = waypoint_in_world.pos_distance_to(rover_in_world)
                    rospy.logdebug(distance_to_target)
                    return distance_to_target
                except tf2_ros.TransformException as e:
                    rospy.logwarn_throttle(1, f"Transform exception: {e}")
                    return float("inf")

            while not rospy.is_shutdown() and distance_to_current_waypoint() > COMPLETION_TOLERANCE:
                rospy.loginfo_throttle(1, f"Distance to current waypoint: {distance_to_current_waypoint()}")
                rate.sleep()

            rospy.loginfo("Waypoint reached")

        rospy.signal_shutdown("Test Complete")


if __name__ == "__main__":
    rostest.rosrun("mrover", "integration_test", TestIntegration, sys.argv)
