#!/usr/bin/env python3

import rospy
from mrover.msg import CAN


def main():
    rospy.init_node("can_debug_requester")
    p = rospy.Publisher("can_requests", CAN, queue_size=1)

    can_msg = CAN(bus=1, message_id=0b10101010101, data=[1, 2, 3, 4, 5])

    while p.get_num_connections() == 0:
        rospy.sleep(0.1)

    p.publish(can_msg)


if __name__ == "__main__":
    main()
