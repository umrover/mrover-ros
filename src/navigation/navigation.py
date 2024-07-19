#!/usr/bin/env python3
import threading

import rospy
from util.state_lib.state_machine import StateMachine
from util.state_lib.state_publisher_server import StatePublisher
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from navigation.approach_target import ApproachTargetState
from navigation.context import Context
from navigation.long_range import LongRangeState
from navigation.post_backup import PostBackupState
from navigation.recovery import RecoveryState
from navigation.search import SearchState
from navigation.state import DoneState, OffState, off_check
from navigation.water_bottle_search import WaterBottleSearchState
from navigation.waypoint import WaypointState
from navigation.follow_lights import FollowLightsState


class Navigation(threading.Thread):
    state_machine: StateMachine
    context: Context
    state_machine_server: StatePublisher

    def __init__(self, context: Context):
        super().__init__()
        self.name = "NavigationThread"
        self.state_machine = StateMachine[Context](OffState(), "NavStateMachine", context)
        self.state_machine.add_transitions(
            ApproachTargetState(),
            [
                WaypointState(),
                SearchState(), 
                WaterBottleSearchState(), 
                RecoveryState(), 
                DoneState()
                ],
        )
        self.state_machine.add_transitions(
            PostBackupState(),
            [
                WaypointState(),
                RecoveryState()
                ],
        )
        self.state_machine.add_transitions(
            RecoveryState(),
            [
                WaypointState(),
                SearchState(),
                PostBackupState(),
                ApproachTargetState(),
                LongRangeState(),
                WaterBottleSearchState(),
            ],
        )
        self.state_machine.add_transitions(
            SearchState(),
            [
                ApproachTargetState(), 
                FollowLightsState(),
                LongRangeState(), 
                WaypointState(), 
                RecoveryState()
            ],
        )
        self.state_machine.add_transitions(
            DoneState(), 
            [
                WaypointState()
            ],
        )
        self.state_machine.add_transitions(
            WaypointState(),
            [
                PostBackupState(),
                ApproachTargetState(),
                WaterBottleSearchState(),
                FollowLightsState(),
                LongRangeState(),
                SearchState(),
                RecoveryState(),
                DoneState(),
            ],
        )
        self.state_machine.add_transitions(
            LongRangeState(),
            [
                ApproachTargetState(), 
                SearchState(), 
                WaterBottleSearchState(), 
                WaypointState(), 
                RecoveryState()
            ],
        )
        self.state_machine.add_transitions(
            OffState(), 
            [
                WaypointState(), 
                DoneState()
            ],
        )
        self.state_machine.add_transitions(
            WaterBottleSearchState(), 
            [
                WaypointState(), 
                RecoveryState(), 
                ApproachTargetState(), 
                LongRangeState()
            ],
        )
        self.state_machine.add_transitions(
            FollowLightsState(), 
            [
                FollowLightsState(),
                ApproachTargetState(),
                SearchState()
            ],
        )
        self.state_machine.configure_off_switch(OffState(), off_check)
        self.state_machine_server = StatePublisher(self.state_machine, "nav_structure", 1, "nav_state", 10)
        self.context = context

    def run(self):
        self.state_machine.run()

    def stop(self):
        # Requests current state to go into 'terminated' to cleanly exit state machine
        self.state_machine.stop()
        self.state_machine_server.stop()
        self.join()
        self.state_machine.context.rover.send_drive_stop()

    def publish_path(self):
        self.context.rover.path_history.header = Header()
        self.context.rover.path_history.header.frame_id = "map"
        poses = []
        rate = rospy.Rate(1)
        while not self.state_machine.stop_event.is_set():
            pose_stamped = PoseStamped()
            pose_stamped.header = Header()
            pose_stamped.header.frame_id = "map"
            rover_pose_in_map = self.context.rover.get_pose_in_map()
            if rover_pose_in_map is not None:
                point = Point(rover_pose_in_map.position[0], rover_pose_in_map.position[1], 0)
                quat = Quaternion(0, 0, 0, 1)
                pose_stamped.pose = Pose(point, quat)
                poses.append(pose_stamped)
                self.context.rover.path_history.poses = poses
                self.context.path_history_publisher.publish(self.context.rover.path_history)
            rate.sleep()


def main():
    rospy.init_node("navigation")
    context = Context()
    navigation = Navigation(context)
    rospy.on_shutdown(navigation.stop)
    navigation.start()
    rospy.loginfo("Navigation starting")
    navigation.publish_path()


if __name__ == "__main__":
    main()
