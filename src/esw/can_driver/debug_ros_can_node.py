#!/usr/bin/env python3

import rospy
from mrover.msg import CAN


def main():
    rospy.init_node("can_debug_requester")
    p = rospy.Publisher("can/jetson/out", CAN, queue_size=1)

    can_msg = CAN(source="jetson", destination="devboard", data=[1, 2, 3, 4, 5], reply_required=False)

    while p.get_num_connections() == 0:
        rospy.sleep(0.1)

    p.publish(can_msg)


if __name__ == "__main__":
    main()
