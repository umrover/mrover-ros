#!/usr/bin/env python3
import rospy
import sys
import math
import time

from mrover.msg import IK
from mrover.msg import Position

LINK_CD_LEN = 0.5531735368
JOINT_B_ANGLE = math.pi / 3 - 0.05
LINK_BC_LEN = 0.5344417294
EE_LENGTH = 0.13
DE_LENGTH = 0.044886000454425812
BC_POS_X = LINK_BC_LEN * math.cos(JOINT_B_ANGLE)
BC_POS_Y = LINK_BC_LEN * math.sin(JOINT_B_ANGLE)
JOINT_C_MIN = 1.35
JOINT_C_MAX = 3.19
THETA_MAX = JOINT_B_ANGLE - 1 * JOINT_C_MIN + 0.16084859151 # this seems to be the correct upper limit
# THETA_MIN = JOINT_B_ANGLE - 1 * JOINT_C_MAX + 0.16084859151 # this seems cooked???
# THETA_MAX = -1 * math.pi / 6
THETA_MIN = JOINT_B_ANGLE - 2.85 + 0.16084859151 # this seems accurate? (c maxes out at like 2.85 for some reason?)
THETA_MIN += 0.6 # safety margin
THETA_MAX -= 0.2 # safety margin
A = (THETA_MIN + THETA_MAX) / 2
B = (THETA_MAX - THETA_MIN) / 2
Y_MIN = 0.0
Y_MAX = 0.4
JOINT_DE_MIN = -0.97
JOINT_DE_MAX = 0.5 # this should be able to go higher???

joint_angles = {
    "joint_a": 0,
    "joint_b": 0,
    "joint_c": 0,
    "joint_de_pitch": 0,
    "joint_de_roll": 0
}

def joint_data_callback(msg: Position):
    for i in range(len(msg.names)):
        joint_angles[msg.names[i]] = msg.positions[i]

def main():
    rospy.init_node("test_ik")
    ik_pub = rospy.Publisher("arm_ik", IK, queue_size=1)
    # angle_sub = rospy.Subscriber("arm_joint_data", Position, joint_data_callback)

    t = math.pi / 2 + 1.2

    while True:
        y = (Y_MAX + Y_MIN) / 2 + (Y_MAX - Y_MIN) / 2 * math.sin(t)
        theta = A + B * math.cos(t)
        ee_theta = theta + (JOINT_DE_MAX + JOINT_DE_MIN) / 2 + (JOINT_DE_MAX - JOINT_DE_MIN) / 2 * math.sin(t)

        target = IK()
        target.target.header.stamp = rospy.Time.now()
        target.target.header.frame_id = "arm_base_link" # maybe make relative to joint c to make sure b doesn't move??
        target.target.pose.position.x = BC_POS_X + LINK_CD_LEN * math.cos(theta)
        target.target.pose.position.y = 0
        target.target.pose.position.z = BC_POS_Y + LINK_CD_LEN * math.sin(theta)
        target.target.pose.position.x += (DE_LENGTH + EE_LENGTH) * math.cos(ee_theta)
        target.target.pose.position.z += (DE_LENGTH + EE_LENGTH) * math.sin(ee_theta)
        target.target.pose.orientation.x = ee_theta


        print(f"{target.target.pose.position.x}, {target.target.pose.position.y}, {target.target.pose.position.z}")
        print(f"pitch: {target.target.pose.orientation.x}")
        print(f"t: {t}")
        input("Press enter to send")
        ik_pub.publish(target)
        print("Sent!")
        # time.sleep(2)
        # actual_x = BC_POS_X + LINK_CD_LEN * math.cos(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151) + (DE_LENGTH + EE_LENGTH) * math.cos(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151 - joint_angles["joint_de_pitch"])
        # actual_z = BC_POS_Y + LINK_CD_LEN * math.sin(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151) + (DE_LENGTH + EE_LENGTH) * math.sin(JOINT_B_ANGLE - joint_angles["joint_c"] + 0.16084859151 - joint_angles["joint_de_pitch"])
        # print(f"Actual: {actual_x}, {actual_z}")

        # t += 0.1


if __name__ == "__main__":
    main()
