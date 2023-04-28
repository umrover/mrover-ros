#!/usr/bin/env python3
# NOTE: the above line is very important
# also make sure to chmod +x any python test files

"""
There are three ways to run this file
- We can launch it as part of the basic_integration-test
with rostest mrover basic_integration test
- We can run it as a unit test with ./python_test.py
- We can run a catkin test which runs all tests in the CMakeLists.txt
"""

import unittest
import numpy as np

# make use of imports relative to the package
from navigation.drive import DriveController
from util.SE3 import SE3


class TestDrive(unittest.TestCase):
    def test_drive_straight(self):
        self.assertEqual(True, True)

    def test_drive_turn(self):
        self.assertEqual(True, True)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "basic_python_test", TestDrive)
