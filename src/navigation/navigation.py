#!/usr/bin/env python3

import signal
import sys
import threading

import rospy
import smach
import smach_ros
from smach.log import loginfo
from smach.log import set_loggers
from std_msgs.msg import String

from approach_post import ApproachPostState, ApproachPostStateTransitions
from context import Context
from gate import GateTraverseState, GateTraverseStateTransitions
from partial_gate import PartialGateState, PartialGateStateTransitions
from post_backup import PostBackupState, PostBackupTransitions
from recovery import RecoveryState, RecoveryStateTransitions
from search import SearchState, SearchStateTransitions
from state import DoneState, DoneStateTransitions, OffState, OffStateTransitions
from waypoint import WaypointState, WaypointStateTransitions


class Navigation(threading.Thread):
    state_machine: smach.StateMachine
    context: Context
    sis: smach_ros.IntrospectionServer

    def __init__(self, context: Context):
        super().__init__()
        set_loggers(info=lambda _: None, warn=loginfo, error=loginfo, debug=loginfo)
        self.name = "NavigationThread"
        self.state_machine = smach.StateMachine(outcomes=["terminated"])
        self.state_machine.userdata.waypoint_index = 0
        self.context = context
        self.sis = smach_ros.IntrospectionServer("", self.state_machine, "/SM_ROOT")
        self.sis.start()
        self.state_publisher = rospy.Publisher("/nav_state", String, queue_size=1)
        with self.state_machine:
            self.state_machine.add(
                "OffState", OffState(self.context), transitions=self.get_transitions(OffStateTransitions)
            )
            self.state_machine.add(
                "DoneState", DoneState(self.context), transitions=self.get_transitions(DoneStateTransitions)
            )
            self.state_machine.add(
                "WaypointState", WaypointState(self.context), transitions=self.get_transitions(WaypointStateTransitions)
            )
            self.state_machine.add(
                "ApproachPostState",
                ApproachPostState(self.context),
                transitions=self.get_transitions(ApproachPostStateTransitions),
            )
            self.state_machine.add(
                "SearchState", SearchState(self.context), transitions=self.get_transitions(SearchStateTransitions)
            )
            self.state_machine.add(
                "GateTraverseState",
                GateTraverseState(self.context),
                transitions=self.get_transitions(GateTraverseStateTransitions),
            )
            self.state_machine.add(
                "RecoveryState", RecoveryState(self.context), transitions=self.get_transitions(RecoveryStateTransitions)
            )
            self.state_machine.add(
                "PartialGateState",
                PartialGateState(self.context),
                transitions=self.get_transitions(PartialGateStateTransitions),
            )
            self.state_machine.add(
                "PostBackupState",
                PostBackupState(self.context),
                transitions=self.get_transitions(PostBackupTransitions),
            )
            rospy.Timer(rospy.Duration(0.1), self.publish_state)

    def get_transitions(self, transitions_enum):
        transition_dict = {transition.name: transition.value for transition in transitions_enum}
        transition_dict["off"] = "OffState"  # logic for switching to offstate is built into OffState
        return transition_dict

    def publish_state(self, event=None):
        with self.state_machine:
            active_states = self.state_machine.get_active_states()
            if len(active_states) > 0:
                self.state_publisher.publish(active_states[0])

    def run(self):
        self.state_machine.execute()

    def stop(self):
        self.sis.stop()
        # Requests current state to go into 'terminated' to cleanly exit state machine
        self.state_machine.request_preempt()
        # Wait for smach thread to terminate
        self.join()
        self.context.rover.send_drive_stop()


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
