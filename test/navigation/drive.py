#!/usr/bin/env python
import unittest
import numpy as np
from navigation.drive import get_drive_command
from util.SE3 import SE3

class TestDrive(unittest.TestCase):
    def test_drive_straight(self):
        self.assertEqual(
            True,
            True
        )
    def test_drive_turn(self):
        self.assertEqual(
            True,
            True
        )

if __name__ == '__main__':
    import rostest
    rostest.rosrun("mrover", 'basic_python_test', TestDrive)