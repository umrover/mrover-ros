#!/usr/bin/env python3

import signal
import sys
import threading

import rospy
import smach
import smach_ros
from context import Context
from single_fiducial import SingleFiducialState
from state import DoneState
from waypoint import WaypointState
from search import SearchState


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
                "DoneState",
                DoneState(self.context),
                transitions={"waypoint_traverse": "WaypointState", "done": "DoneState"},
            )
            self.state_machine.add(
                "WaypointState",
                WaypointState(self.context),
                transitions={
                    "waypoint_traverse": "WaypointState",
                    "single_fiducial": "SingleFiducialState",
                    "search": "SearchState",
                    "done": "DoneState",
                },
            )
            self.state_machine.add(
                "SingleFiducialState",
                SingleFiducialState(self.context),
                transitions={
                    "waypoint_traverse": "WaypointState",
                    "single_fiducial": "SingleFiducialState",
                    "search": "SearchState",
                    "done": "DoneState",
                },
            )
            self.state_machine.add(
                "SearchState",
                SearchState(self.context),
                transitions={
                    "waypoint_traverse": "WaypointState",
                    "single_fiducial": "SingleFiducialState",
                    "search": "SearchState",
                },
            )

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
