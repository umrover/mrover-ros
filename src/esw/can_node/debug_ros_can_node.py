#!usr/bin/env python3

import rospy
from mrover.msg import CAN

def main():
    p = rospy.Publisher("can_requests", CAN)

    rospy.init_node("can_debug_requester")

    can_msg = CAN()
    can_msg.bus = 1
    can_msg.message_id = 135 # 0b10101010101
    can_msg.data = [1, 2, 3, 4, 5]
    p.publish(can_msg)


if __name__ == "__main__":
    main()