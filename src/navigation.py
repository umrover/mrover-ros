import rospy
import smach

class DriveState(smach.State):
    def __init__(self, outcomes=["driving", "done"]):
        pass
    

    def execute(self, userdata):
        pass


def main():
    rospy.init_node("nav")
    state_machine = smach.StateMachine(outcomes=['driving'])
    state_machine.add("DriveState", DriveState())
    
    outcome = state_machine.execute()


main()