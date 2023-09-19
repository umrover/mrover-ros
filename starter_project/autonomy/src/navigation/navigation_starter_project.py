#!/usr/bin/env python3

import signal
import sys
import threading

# ros and state machine imports
import rospy
import smach
import smach_ros

# navigation specific imports
from context import Context
from drive_state import DriveState
from state import DoneState
from tag_seek import TagSeekState


class Navigation(threading.Thread):
    state_machine: smach.StateMachine
    context: Context
    sis: smach_ros.IntrospectionServer

    def __init__(self, context: Context):
        super().__init__()
        self.name = "NavigationThread"
        self.state_machine = smach.StateMachine(outcomes=["terminated"])
        self.context = context
        self.sis = smach_ros.IntrospectionServer("", self.state_machine, "/SM_ROOT")
        self.sis.start()
        with self.state_machine:
            # TODO: add DriveState and its transitions here

            # DoneState and its transitions
            self.state_machine.add(
                "DoneState",
                DoneState(self.context),
                transitions={"done": "DoneState"},
            )
            # TODO: add TagSeekState and its transitions here

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
    # TODO: init a node called "navigation"

    # context and navigation objects
    context = Context()
    navigation = Navigation(context)

    # Define custom handler for Ctrl-C that shuts down smach properly
    def sigint_handler(_sig, _frame):
        navigation.stop()
        rospy.signal_shutdown("keyboard interrupt")
        try:
            sys.exit(0)
        except SystemExit:
            pass

    signal.signal(signal.SIGINT, sigint_handler)
    navigation.start()


if __name__ == "__main__":
    main()
