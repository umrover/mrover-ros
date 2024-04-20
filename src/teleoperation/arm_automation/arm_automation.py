#!/usr/bin/env python3

import rospy

import actionlib

from mrover.msg import ArmActionAction, ArmActionGoal, ArmActionResult, Position

from sensor_msgs.msg import JointState


def arm_automation() -> None:
    rospy.init_node("arm_automation")

    server = None
    joint_state = None

    pos_pub = rospy.Publisher("arm_position_cmd", Position, queue_size=1)

    def joint_state_callback(msg: JointState):
        nonlocal joint_state
        joint_state = msg

    rospy.Subscriber("joint_states", JointState, joint_state_callback)

    def execute_callback(goal: ArmActionGoal):
        success = True

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if server.is_preempt_requested():
                server.set_preempted()
                success = False
                break

            if goal.name == "de_home":
                pos_pub.publish(Position(names=["joint_de_pitch", "joint_de_roll"], position=[0, 0]))
                if joint_state is None:
                    success = False
                    break

                if abs(joint_state.position[0]) < 0.1 and abs(joint_state.position[1]) < 0.1:
                    break

            rate.sleep()

        if success:
            server.set_succeeded(ArmActionResult())
        else:
            server.set_aborted(ArmActionResult())

    server = actionlib.SimpleActionServer("arm_action", ArmActionAction, execute_cb=execute_callback, auto_start=False)
    server.start()


if __name__ == "__main__":
    try:
        arm_automation()
    except rospy.ROSInterruptException:
        pass
