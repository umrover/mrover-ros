import rospy
import smach_ros
import smach


class DriveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["driving", "drive_done"])
        pass

    def execute(self, userdata):

        return "driving"


class TurnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["turning", "turn_done"])
        pass

    def execute(self, userdata):
        return "turning"


def main():
    rospy.init_node("nav")

    state_machine = smach.StateMachine(outcomes=['done'])
    sis = smach_ros.IntrospectionServer('server_name', state_machine, '/SM_ROOT')
    sis.start()
    with state_machine:
        state_machine.add("DriveState", DriveState(), transitions={"driving": "DriveState", "drive_done": "TurnState"})
        state_machine.add("TurnState", TurnState(), transitions={"turning": "TurnState", "turn_done": "DriveState"})

    outcome = state_machine.execute()


if __name__ == '__main__':
    main()
