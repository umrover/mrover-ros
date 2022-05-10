import rospy
import smach
import smach_ros

from common import DoneState, Context
from tag import SingleTagState
from waypoint import WaypointState


class Navigation:
    state_machine: smach.StateMachine
    context: Context

    def __init__(self):
        # rospy.Subscriber('/fiducial_vertices', FiducialArray, self.tag_callback)
        self.state_machine = smach.StateMachine(outcomes=['done'])
        self.context = Context()
        sis = smach_ros.IntrospectionServer('server_name', self.state_machine, '/SM_ROOT')
        sis.start()
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

    # def tag_callback(self, data):
    #     print(data)

    def run(self):
        return self.state_machine.execute()


if __name__ == '__main__':
    rospy.init_node('nav')
    Navigation().run()
