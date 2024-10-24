#!/usr/bin/env python3
import rospy
import sys
import math
import time

from mrover.msg import IK

LINK_CD_LEN = 0.5531735368
JOINT_B_ANGLE = math.pi / 8
LINK_BC_LEN = 0.5344417294
EE_LENGTH = 0.13
BC_POS_X = LINK_BC_LEN * math.cos(JOINT_B_ANGLE)
BC_POS_Y = LINK_BC_LEN * math.sin(JOINT_B_ANGLE)
JOINT_C_MIN = 1.35
JOINT_C_MAX = 3.19
# THETA_MAX = JOINT_B_ANGLE - 1 * JOINT_C_MIN + 0.16084859151
# THETA_MIN = JOINT_B_ANGLE - 1 * JOINT_C_MAX + 0.16084859151
THETA_MAX = -1 * math.pi / 6
THETA_MIN = JOINT_B_ANGLE - 2.85
A = (THETA_MIN + THETA_MAX) / 2
B = (THETA_MAX - THETA_MIN) / 2
Y_MIN = 0.0
Y_MAX = 0.4
    

def main():
    rospy.init_node("test_ik")
    ik_pub = rospy.Publisher("arm_ik", IK, queue_size=1)

    t = 0.0

    while True:
        y = (Y_MAX + Y_MIN) / 2 + (Y_MAX - Y_MIN) / 2 * math.sin(t)
        theta = A + B * math.cos(t)

        target = IK()
        target.target.header.stamp = rospy.Time.now()
        target.target.header.frame_id = "arm_base_link" # maybe make relative to joint c to make sure b doesn't move??
        target.target.pose.position.x = BC_POS_X + LINK_CD_LEN * math.cos(theta) + EE_LENGTH
        target.target.pose.position.y = y
        target.target.pose.position.z = BC_POS_Y + LINK_CD_LEN * math.sin(theta)

        print("Sending IK command...")
        ik_pub.publish(target)
        print(f"{target.target.pose.position.x}, {target.target.pose.position.y}, {target.target.pose.position.z}")
        print("Sent!")
        # time.sleep(2)
        input("Press enter to continue")
        t += 0.1


if __name__ == "__main__":
    main()
