import rospy
import smach_ros
import smach

from geometry_msgs.msg import TwistStamped


class WaypointState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["waypoint", "waypoint_done"])

    def execute(self, userdata):
        return "waypoint"


class SingleTagState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["single_tag", "single_tag_done"])

    def execute(self, userdata):
        return "single_tag"


class DoneState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata):
        return "done"


class Navigation:
    vel_cmd_publisher: rospy.Publisher
    state_machine: smach.StateMachine

    def __init__(self):
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)
        # rospy.Subscriber('/fiducial_vertices', FiducialArray, self.tag_callback)
        self.state_machine = smach.StateMachine(outcomes=['done'])
        sis = smach_ros.IntrospectionServer('server_name', self.state_machine, '/SM_ROOT')
        sis.start()
        with self.state_machine:
            self.state_machine.add("WaypointState", WaypointState(), transitions={
                "waypoint": "WaypointState",
                "waypoint_done": "DoneState"
            })
            self.state_machine.add("SingleTagState", SingleTagState(), transitions={
                "single_tag": "SingleTagState",
                "single_tag_done": "DoneState"
            })

    # def tag_callback(self, data):
    #     print(data)

    def run(self):
        return self.state_machine.execute()


if __name__ == '__main__':
    rospy.init_node("nav")
    navigation = Navigation()
    navigation.run()
