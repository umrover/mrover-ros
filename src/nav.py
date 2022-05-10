from abc import ABC

import rospy
import smach_ros
import smach
import tf

from geometry_msgs.msg import TwistStamped


class BaseState(smach.State, ABC):
    navigation: 'Navigation'

    def __init__(self, navigation: 'Navigation', *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigation = navigation


class WaypointState(BaseState):
    def __init__(self, navigation: 'Navigation'):
        super().__init__(
            navigation,
            outcomes=['waypoint', 'waypoint_done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'waypoint'


class SingleTagState(BaseState):
    def __init__(self, navigation: 'Navigation'):
        super().__init__(
            navigation,
            outcomes=['single_tag', 'single_tag_done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'single_tag'


class DoneState(BaseState):
    def __init__(self, navigation: 'Navigation'):
        super().__init__(
            navigation,
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'done'


class Navigation:
    state_machine: smach.StateMachine
    vel_cmd_publisher: rospy.Publisher
    tf_listener: tf.TransformListener

    def __init__(self):
        # rospy.Subscriber('/fiducial_vertices', FiducialArray, self.tag_callback)
        self.state_machine = smach.StateMachine(outcomes=['done'])
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()
        sis = smach_ros.IntrospectionServer('server_name', self.state_machine, '/SM_ROOT')
        sis.start()
        with self.state_machine:
            self.state_machine.add(
                'WaypointState', WaypointState(self),
                transitions={
                    'waypoint': 'WaypointState',
                    'waypoint_done': 'DoneState'
                },
            )
            self.state_machine.add(
                'SingleTagState', SingleTagState(self),
                transitions={
                    'single_tag': 'SingleTagState',
                    'single_tag_done': 'DoneState'
                }
            )
            self.state_machine.add(
                'DoneState', DoneState(self),
                transitions={'done': 'DoneState'}
            )

    # def tag_callback(self, data):
    #     print(data)

    def run(self):
        return self.state_machine.execute()


if __name__ == '__main__':
    rospy.init_node('nav')
    Navigation().run()
