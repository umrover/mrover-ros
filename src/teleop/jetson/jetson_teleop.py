import asyncio
from math import copysign
from enum import Enum
import rospy as ros
from mrover.msg import Joystick 


def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold)/(1 - threshold)

    return copysign(temp_mag, magnitude)


class Drive:
    def __init__(self, reverse: bool):
        self.auton_enabled = False
        self.teleop_enabled = True
        self.reverse = reverse

    def teleop_drive_callback(self, channel, msg):
        if self.auton_enabled or not self.teleop_enabled:
            return

        input = Joystick.decode(msg)

        # TODO: possibly add quadratic control
        linear = deadzone(input.forward_back, 0.05) * input.dampen
        angular = deadzone(input.left_right, 0.1) * input.dampen

        # Convert arcade drive to tank drive
        angular_op = (angular / 2) / (abs(linear) + 0.5)
        vel_left = linear - angular_op
        vel_right = linear + angular_op

        # Account for reverse
        if self.reverse:
            tmp = vel_left
            vel_left = -1 * vel_right
            vel_right = -1 * tmp

        # Scale to be within [-1, 1], if necessary
        if abs(vel_left) > 1 or abs(vel_right) > 1:
            if abs(vel_left) > abs(vel_right):
                vel_right /= abs(vel_left)
                vel_left /= abs(vel_left)
            else:
                vel_left /= abs(vel_right)
                vel_right /= abs(vel_right)

        command = DriveVelCmd()
        command.left = vel_left
        command.right = vel_right

        lcm_.publish('/drive_vel_cmd', command.encode())


def main():
    drive = Drive(reverse=False)


    lcm_.subscribe("/joystick", drive.teleop_drive_callback)


