#!/usr/bin/env python3
import threading

import rospy
from util.state_lib.state_machine import StateMachine
from util.state_lib.state_publisher_server import StatePublisher
from navigation.approach_target import ApproachTargetState
from navigation.context import Context
from navigation.long_range import LongRangeState
from navigation.post_backup import PostBackupState
from navigation.recovery import RecoveryState
from navigation.search import SearchState
from navigation.state import DoneState, OffState, off_check
from navigation.water_bottle_search import WaterBottleSearchState
from navigation.waypoint import WaypointState


class Navigation(threading.Thread):
    state_machine: StateMachine
    context: Context
    state_machine_server: StatePublisher

    def __init__(self, context: Context):
        super().__init__()
        self.name = "NavigationThread"
        self.state_machine = StateMachine[Context](OffState(), "NavStateMachine", context)
        self.state_machine.add_transitions(
            ApproachTargetState(), [WaypointState(), SearchState(), RecoveryState(), DoneState()]
        )
        self.state_machine.add_transitions(PostBackupState(), [WaypointState(), RecoveryState()])
        self.state_machine.add_transitions(
            RecoveryState(),
            [
                WaypointState(),
                SearchState(),
                PostBackupState(),
                ApproachTargetState(),
                LongRangeState(),
            ],
        )
        self.state_machine.add_transitions(
            SearchState(),
            [ApproachTargetState(), LongRangeState(), WaypointState(), RecoveryState()],
        )
        self.state_machine.add_transitions(DoneState(), [WaypointState()])
        self.state_machine.add_transitions(
            WaypointState(),
            [
                PostBackupState(),
                ApproachTargetState(),
                WaterBottleSearchState(),
                LongRangeState(),
                SearchState(),
                RecoveryState(),
                DoneState(),
            ],
        )
        self.state_machine.add_transitions(ApproachTargetState(), [DoneState(), SearchState(), RecoveryState()])
        self.state_machine.add_transitions(
            LongRangeState(), [ApproachTargetState(), SearchState(), WaypointState(), RecoveryState()]
        )
        self.state_machine.add_transitions(OffState(), [WaypointState(), DoneState()])
        self.state_machine.add_transitions(
            WaterBottleSearchState(), [WaypointState(), RecoveryState(), ApproachTargetState()]
        )
        self.state_machine.configure_off_switch(OffState(), off_check)
        self.state_machine_server = StatePublisher(self.state_machine, "nav_structure", 1, "nav_state", 10)

    def run(self):
        self.state_machine.run()

    def stop(self):
        # Requests current state to go into 'terminated' to cleanly exit state machine
        self.state_machine.stop()
        self.state_machine_server.stop()
        self.join()
        self.state_machine.context.rover.send_drive_stop()


def main():
    rospy.init_node("navigation")
    context = Context()
    navigation = Navigation(context)
    rospy.on_shutdown(navigation.stop)
    navigation.start()
    rospy.loginfo("Navigation starting")


if __name__ == "__main__":
    main()
