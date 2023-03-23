#!/usr/bin/env python3

import signal
import sys
import threading

import rospy
import smach
import smach_ros
from context import Context
from gate import GateTraverseState, GateTraverseStateTransitions
from approach_post import ApproachPostState, ApproachPostStateTransitions
from state import DoneState, DoneStateTransitions, OffState, OffStateTransitions
from waypoint import WaypointState, WaypointStateTransitions
from search import SearchState, SearchStateTransitions
<<<<<<< HEAD
from recovery import RecoveryState, RecoveryStateTransitions
# from drive import collector
=======

from partial_gate import PartialGateState, PartialGateStateTransitions
>>>>>>> failure_identification


class Navigation(threading.Thread):
    state_machine: smach.StateMachine
    context: Context
    sis: smach_ros.IntrospectionServer

    def __init__(self, context: Context):
        super().__init__()
        self.name = "NavigationThread"
        self.state_machine = smach.StateMachine(outcomes=["terminated"])
        self.state_machine.userdata.waypoint_index = 0
        self.context = context
        self.sis = smach_ros.IntrospectionServer("", self.state_machine, "/SM_ROOT")
        self.sis.start()
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
                # The lines below are necessary because ApproachPostState inherits from WaypointState, so WaypointState's transitions
                # need to be registered for ApproachPostState as well.
                transitions=dict(
                    self.get_transitions(ApproachPostStateTransitions), **self.get_transitions(WaypointStateTransitions)
                ),
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
<<<<<<< HEAD
                "RecoveryState", 
                RecoveryState(self.context), 
                transitions=self.get_transitions(RecoveryStateTransitions)
=======
                "PartialGateState",
                PartialGateState(self.context),
                transitions=self.get_transitions(PartialGateStateTransitions),
>>>>>>> failure_identification
            )

    def get_transitions(self, transitions_enum):
        transition_dict = {transition.name: transition.value for transition in transitions_enum}
        transition_dict["off"] = "OffState"  # logic for switching to offstate is built into OffState
        return transition_dict

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
<<<<<<< HEAD
    rospy.logerr(f"Set context in navigation.py")
#    collector.set_context(context)
=======
>>>>>>> failure_identification
    navigation = Navigation(context)

    # Define custom handler for Ctrl-C that shuts down smach properly
    def sigint_handler(_sig, _frame):
<<<<<<< HEAD
        context.rover.collector.write_to_csv()
=======
>>>>>>> failure_identification
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
