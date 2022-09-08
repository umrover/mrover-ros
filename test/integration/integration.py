#!/usr/bin/env python3

import unittest

import rospy


class TestIntegration(unittest.TestCase):
    def test_integration(self):
        rospy.init_node("test_integration")
        rospy.spin()


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "integration", TestIntegration)
