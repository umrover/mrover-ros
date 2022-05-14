#!/usr/bin/env python3

import rospy


def main():
    print('===== localization starting =====')
    rospy.init_node('localization')
    rospy.spin()


if __name__ == '__main__':
    main()
