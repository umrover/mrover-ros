#!/usr/bin/env python3
import rospy
import serial
import threading

from mrover.srv import AdjustMotor


def adjust_gimbal_client(value):
    rospy.wait_for_service('adjust_mast_gimbal_z')
    try:
        adjust_z = rospy.ServiceProxy('adjust_mast_gimbal_z', AdjustMotor)
        resp = adjust_z("mast_gimbal_z", value)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    adjust_gimbal_client(0.0)


if __name__ == "__main__":
    main()
