#!/usr/bin/env python3
import threading

import rospy
import smach
import smach_ros
from common import DoneState, Context
from tag import SingleTagState
from waypoint import WaypointState


class Navigation:
    state_machine: smach.StateMachine
    context: Context
    thread: threading.Thread
    sis: smach_ros.IntrospectionServer

    def __init__(self):
        # rospy.Subscriber('/fiducial_vertices', FiducialArray, self.tag_callback)
        self.state_machine = smach.StateMachine(outcomes=['done'])
        self.context = Context()
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

    def start(self):
        self.thread = threading.Thread(target=self.state_machine.execute)
        self.thread.start()

    def stop(self):
        self.state_machine.request_preempt()
        self.sis.stop()
        self.thread.join()


if __name__ == '__main__':
    rospy.init_node('navigation')
    navigation = Navigation()
    navigation.start()
    rospy.spin()
    navigation.stop()
