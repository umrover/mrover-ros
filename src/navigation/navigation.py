#!/usr/bin/env python3

import signal
import sys
import threading

import rospy
import smach
import smach_ros
from common import DoneState, Context
from geometry_msgs.msg import Twist
from tag import SingleTagState
from waypoint import WaypointState


class Navigation(threading.Thread):
    state_machine: smach.StateMachine
    context: Context
    sis: smach_ros.IntrospectionServer

    def __init__(self, context: Context):
        super().__init__()
        self.setName('NavigationThread')
        self.state_machine = smach.StateMachine(outcomes=['terminated'])
        self.context = context
        self.sis = smach_ros.IntrospectionServer('server_name', self.state_machine, '/SM_ROOT')
        self.sis.start()
        with self.state_machine:
            self.state_machine.add(
                'WaypointState', WaypointState(self.context),
                transitions={
                    'waypoint': 'WaypointState',
                    'waypoint_done': 'DoneState'
                },
            )
            self.state_machine.add(
                'SingleTagState', SingleTagState(self.context),
                transitions={
                    'single_tag': 'SingleTagState',
                    'single_tag_done': 'DoneState'
                }
            )
            self.state_machine.add(
                'DoneState', DoneState(self.context),
                transitions={'done': 'DoneState'}
            )

    def run(self):
        self.state_machine.execute()

    def stop(self):
        self.sis.stop()
        # Requests current state to go into 'terminated' to cleanly exit state machine
        self.state_machine.request_preempt()
        # Wait for smach thread to terminate
        self.join()
        self.context.vel_cmd_publisher.publish(Twist())


def main():
    rospy.init_node('navigation')
    context = Context()
    navigation = Navigation(context)

    # Define custom handler for Ctrl-C that shuts down smach properly
    def sigint_handler(sig, frame):
        navigation.stop()
        rospy.signal_shutdown('keyboard interrupt')
        try:
            sys.exit(0)
        except SystemExit:
            # TODO: not really sure why needed bug it is bugging me! >:(
            pass

    signal.signal(signal.SIGINT, sigint_handler)
    navigation.start()


if __name__ == '__main__':
    main()
