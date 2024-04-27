#!/usr/bin/env python3

import rospy

import actionlib

from mrover.msg import ArmActionAction, ArmActionGoal, ArmActionResult, Position

from sensor_msgs.msg import JointState

import numpy as np


def arm_automation() -> None:
    rospy.init_node("arm_automation")

    server = None
    joint_state = None

    pos_pub = rospy.Publisher("arm_position_cmd", Position, queue_size=1)

    def joint_state_callback(msg: JointState):
        nonlocal joint_state
        joint_state = msg

    rospy.Subscriber("arm_joint_data", JointState, joint_state_callback)

    def execute_callback(goal: ArmActionGoal):
        if joint_state is None:
            rospy.logerr("No joint state data available")
            server.set_aborted(ArmActionResult())
            return
        
        if goal.name == "de_home":
            target_names = ["joint_de_pitch", "joint_de_roll"]
            target_positions = [
                joint_state.position[joint_state.name.index('joint_de_pitch')],
                np.pi / 8
            ]
            rospy.loginfo(f"Moving to {target_positions} for {target_names}")
        else:
            rospy.logerr("Invalid goal name")
            server.set_aborted(ArmActionResult())
            return
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if server.is_preempt_requested():
                server.set_preempted()
                rospy.loginfo("Preempted")
                server.set_aborted(ArmActionResult())
                return


            pos_pub.publish(Position(names=target_names, positions=target_positions))

            feedback = [
                joint_state.position[joint_state.name.index('joint_de_pitch')],
                joint_state.position[joint_state.name.index('joint_de_roll')]
            ]
            
            if np.allclose(target_positions, feedback, atol=0.1):
                rospy.loginfo("Reached target")
                break

            rate.sleep()

        server.set_succeeded(ArmActionResult())

    rospy.sleep(1)

    server = actionlib.SimpleActionServer("arm_action", ArmActionAction, execute_cb=execute_callback, auto_start=False)
    server.start()


if __name__ == "__main__":
    try:
        arm_automation()
    except rospy.ROSInterruptException:
        pass
