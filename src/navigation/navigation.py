#!/usr/bin/env python3

import signal
import sys
import threading

import rospy

from util.state_lib.state_machine import StateMachine
from util.state_lib.state_publisher_server import StatePublisher

from navigation.approach_post import ApproachPostState
from navigation.context import Context
from navigation.post_backup import PostBackupState
from navigation.recovery import RecoveryState
from navigation.search import SearchState
from navigation.state import DoneState, OffState, off_check
from navigation.waypoint import WaypointState


class Navigation(threading.Thread):
    state_machine: StateMachine
    context: Context
    state_machine_server: StatePublisher

    def __init__(self, context: Context):
        super().__init__()
        self.name = "NavigationThread"
        self.state_machine = StateMachine(OffState(), "NavStateMachine")
        self.state_machine.set_context(context)
        self.state_machine.add_transitions(ApproachPostState(), [WaypointState(), SearchState(), RecoveryState()])
        self.state_machine.add_transitions(PostBackupState(), [WaypointState(), RecoveryState()])
        self.state_machine.add_transitions(
            RecoveryState(), [WaypointState(), SearchState(), PostBackupState(), ApproachPostState()]
        )
        self.state_machine.add_transitions(SearchState(), [ApproachPostState(), WaypointState(), RecoveryState()])
        self.state_machine.add_transitions(DoneState(), [WaypointState()])
        self.state_machine.add_transitions(
            WaypointState(), [PostBackupState(), ApproachPostState(), SearchState(), RecoveryState()]
        )
        self.state_machine.add_transitions(OffState(), [WaypointState()])
        self.state_machine.configure_off_switch(OffState(), off_check)
        self.state_machine_server = StatePublisher(self.state_machine, "nav_structure", 1, "nav_state", 10)

    def run(self):
        self.state_machine.run()

    def stop(self):
        # Requests current state to go into 'terminated' to cleanly exit state machine
        self.state_machine.stop()
        self.join()
        self.state_machine.context.rover.send_drive_stop()


def main():
    rospy.loginfo("===== navigation starting =====")
    rospy.init_node("navigation")
    context = Context()
    navigation = Navigation(context)

    # Define custom handler for Ctrl-C that shuts down smach properly
    def sigint_handler(_sig, _frame):
        navigation.stop()
        rospy.signal_shutdown("keyboard interrupt")
        try:
            sys.exit(0)
        except SystemExit:
            # TODO: not really sure why needed but it is bugging me! >:(
            pass

    signal.signal(signal.SIGINT, sigint_handler)
    navigation.start()


if __name__ == "__main__":
    main()
