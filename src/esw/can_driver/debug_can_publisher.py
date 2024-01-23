#!/usr/bin/env python3

import rospy
from mrover.msg import CAN


def callback(msg):
    print("Received message!")
    print("BUS: " + msg.bus)
    print("ID: " + hex(msg.id))
    print("DATA: ")
    for d in msg.data:
        print(hex(d))


def main():
    rospy.init_node("can_commands_listener")
    rospy.Subscriber("can_commands", CAN, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
