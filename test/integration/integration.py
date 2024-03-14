#!/usr/bin/env python3

import sys
import unittest
from typing import Optional

import numpy as np

import rospy
import rostest
import tf2_ros
from nav_msgs.msg import Odometry
from mrover.msg import Waypoint, WaypointType, StateMachineStateUpdate
from util.SE3 import SE3
from util.course_publish_helpers import publish_waypoints, convert_waypoint_to_gps

COMPLETION_TOLERANCE = 3.0
LOCALIZATION_ERROR_TOLERANCE = 1.0


class TestIntegration(unittest.TestCase):
    def test_integration(self):
        rospy.init_node("integration_test")

        rospy.loginfo("Autonomy integration test starting...")

        waypoints = [
            (
                Waypoint(tag_id=0, type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([5, 3, 0])),
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

        nav_state = StateMachineStateUpdate()

        def nav_state_callback(msg: StateMachineStateUpdate) -> None:
            nonlocal nav_state
            nav_state = msg
        
        rover_in_world_gt = SE3()
        
        def ground_truth_callback(msg: Odometry) -> None:
            nonlocal rover_in_world_gt
            pose = msg.pose.pose
            position = np.array([pose.position.x, pose.position.y, pose.position.z])
            orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            rover_in_world_gt = SE3(position, orientation)

        rospy.Subscriber("/nav_state", StateMachineStateUpdate, nav_state_callback)
        rospy.Subscriber("/ground_truth", Odometry, ground_truth_callback)

        rate = rospy.Rate(20)
        for waypoint in waypoints:
            waypoint_data, pose = waypoint
            rospy.loginfo(f"Moving to waypoint: {waypoint}")

            def distance_to_target() -> float:
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
                    return distance_to_target
                except tf2_ros.TransformException as e:
                    rospy.logwarn_throttle(1, f"Transform exception: {e}")
                    return float("inf")
            
            def localization_error() -> float:
                try:
                    rover_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="base_link")
                    localization_error = rover_in_world.pos_distance_to(rover_in_world_gt)
                    return localization_error
                except tf2_ros.TransformException as e:
                    rospy.logwarn_throttle(1, f"Transform exception: {e}")
                    return 0

            while (distance := distance_to_target()) > COMPLETION_TOLERANCE and nav_state.state != "DoneState":
                if rospy.is_shutdown():
                    return
                error = localization_error()
                self.assertLessEqual(error, LOCALIZATION_ERROR_TOLERANCE)
                # rospy.loginfo
                rospy.loginfo_throttle(1, f"Distance to current waypoint: {distance}, navigation state: {nav_state.state}, localization error: {error}")
                rate.sleep()

            rospy.loginfo("Waypoint reached")

        rospy.signal_shutdown("Test Complete")


if __name__ == "__main__":
    rostest.rosrun("mrover", "integration_test", TestIntegration, sys.argv)
