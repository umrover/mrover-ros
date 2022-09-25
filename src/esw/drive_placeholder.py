#!/usr/bin/env python3
import rospy
import time as t


def main():
    rospy.init_node(f"brushless_control")

    for i in range(5):
        print("Driving motors")
        t.sleep(1)


if __name__ == "__main__":
    main()
