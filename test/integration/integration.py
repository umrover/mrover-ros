#!/usr/bin/env python3

import sys
import unittest

import numpy as np

import rospy
import rostest
import tf2_ros
from mrover.msg import Waypoint, WaypointType
from util.SE3 import SE3
from util.course_publish_helpers import publish_waypoints, convert_waypoint_to_gps
from typing import List, Tuple
from smach_msgs.msg import SmachContainerStatus
from mrover.msg import EnableAuton
from mrover.srv import PublishEnableAuton

COMPLETION_TOLERANCE = 3.0


class TestIntegration(unittest.TestCase):
    def get_waypoint_list(self, waypoint, waypointType: WaypointType, id: int):
        return [
            (
                Waypoint(fiducial_id=id, tf_id=f"course{id}", type=WaypointType(val=waypointType)),
                waypoint,
            ),
        ]
    
    def send_waypoint(self, waypoint_list: List[Tuple[Waypoint, SE3]]):
        publish_waypoints([convert_waypoint_to_gps(waypoint) for waypoint in waypoint_list])

    def nav_status_callback(self, status_msg: SmachContainerStatus):
        """
        Recieves and updates nav status

        :param status_msg: nav status message
        """
        self.nav_state = status_msg.active_states[0]
    
    def wait_for_completion(self):
        while not rospy.is_shutdown() and self.nav_state != "DoneState":
            rospy.sleep(0.1)

    def send_disable(self):
        rospy.wait_for_service("enable_auton")
        try:
            publish_enable = rospy.ServiceProxy("enable_auton", PublishEnableAuton)
            msg = EnableAuton([], False)
            publish_enable(msg)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def verify_arrived(self, pt: SE3, tf_buffer: tf2_ros.Buffer):
        """
        Verifies that the given point is within COMPLETION_TOLERANCE of the current robot position

        :param pt: point to verify
        :param tf_buffer: tf buffer to use for lookup
        """
        try:
            rover_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="base_link")
            position = rover_in_world.position
            self.assertLess(np.linalg.norm(pt.position - position), COMPLETION_TOLERANCE)
        except tf2_ros.TransformException as e:
            rospy.logerr(e)
            self.fail("Failed to lookup transform")

    def verify_gate(self, pt: SE3, tf_buffer: tf2_ros.Buffer):
        """
        TODO:Implement this so it checks if the rover has gone through the gate. Not 100% sure on the best way to do this..
        """
        pass


    def test_integration(self):
        rospy.logdebug("Integration Test Starting")

        rospy.init_node("integration_test")

        rospy.loginfo("Integration Test Ready")

        

        self.nav_state = ""
        rospy.Subscriber("smach/container_status", SmachContainerStatus, self.nav_status_callback)

        waypoint_1 = SE3(position=np.array([-5.5, 0.0, 0.0]))
        waypoint_2 = SE3(position=np.array([-10.0, -10.0, 0.0]))
        waypoint_3 = SE3(position=np.array([-15.0, 0.0, 0.0]))

        #post 1 (at 10.0, -5.5)
        post1_pos = np.array([10, -5.5, 0.0])
        waypoint_4 = SE3(position=np.array([8.0, -7.5, 0.0]))
        #post 2 (at -3.5, 15.5)
        post2_pos = np.array([-3.5, 15.5, 0.0])
        waypoint_5 = SE3(position=np.array([0, 15, 0.0]))
        #post 3 (at 13.5, 5.5)
        post3_pos = np.array([13.5, 5.5, 0.0])
        waypoint_6 = SE3(position=np.array([10, 2, 0.0]))

        #gate (at -4.5, -15.5)
        waypoint_7 = SE3(position=np.array([-2, -12, 0.0]))

        gps_points = [self.get_waypoint_list(w, WaypointType.NO_SEARCH, -1) for w in [waypoint_1, waypoint_2, waypoint_3]]

        post_points = [self.get_waypoint_list(w, WaypointType.POST, i) for w, i in [(waypoint_4, 1), (waypoint_5, 2), (waypoint_6, 3)]]
        post_locs = [post1_pos, post2_pos, post3_pos]
        gate_point = self.get_waypoint_list(waypoint_7, WaypointType.GATE, 4)


        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        for gps_pt in gps_points:
            self.send_waypoint(gps_pt)
            self.wait_for_completion()
            self.verify_arrived(gps_pt[0][1], tf_buffer)
            self.send_disable()
            rospy.sleep(2.0)
        
        for post_pt, actual_post_loc in zip(post_points, post_locs):
            self.send_waypoint(post_pt)
            self.wait_for_completion()
            self.verify_arrived(actual_post_loc, tf_buffer)
            self.send_disable()
            rospy.sleep(2.0)
        
        self.send_waypoint(gate_point)
        self.wait_for_completion()
        self.verify_gate(gate_point[0][1], tf_buffer)

if __name__ == "__main__":
    rostest.rosrun("mrover", "integration_test", TestIntegration, sys.argv)
