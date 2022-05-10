import rospy
import smach_ros
import smach

from geometry_msgs.msg import TwistStamped
from fiducial_msgs.msg import FiducialArray


class DriveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["driving", "drive_done"])

    def execute(self, userdata):
        return "driving"


class TurnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["turning", "turn_done"])

    def execute(self, userdata):
        return "turning"


class Navigation:
    vel_cmd_publisher: rospy.Publisher
    state_machine: smach.StateMachine

    def __init__(self):
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)
        rospy.Subscriber('/fiducial_vertices', FiducialArray, self.tag_callback)
        self.state_machine = smach.StateMachine(outcomes=['done'])
        sis = smach_ros.IntrospectionServer('server_name', self.state_machine, '/SM_ROOT')
        sis.start()
        with self.state_machine:
            self.state_machine.add("DriveState", DriveState(), transitions={
                "driving": "DriveState",
                "drive_done": "TurnState"
            })
            self.state_machine.add("TurnState", TurnState(), transitions={
                "turning": "TurnState",
                "turn_done": "DriveState"
            })

    def tag_callback(self, data):
        print(data)

    def run(self):
        return self.state_machine.execute()


if __name__ == '__main__':
    rospy.init_node("nav")
    navigation = Navigation()
    navigation.run()
