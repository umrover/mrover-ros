import asyncio
from math import copysign
from enum import Enum
import rospy as ros
from sensor_msgs.msg import Joy


def quadratic(val):
    return copysign(val**2, val)

def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold)/(1 - threshold)

    return copysign(temp_mag, magnitude)


class Drive:
    class LogitechAxes(Enum):
      left_right: 0
      forward_back: 1
      twist: 2
      dampen: 3
      pan: 4
      tilt: 5

    def __init__(self, reverse: bool):
        self.auton_enabled = False
        self.teleop_enabled = True
        self.reverse = reverse

    def teleop_drive_callback(self, msg):
        if self.auton_enabled or not self.teleop_enabled:
            return
        

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

        drive_vel_pub = ros.Publisher('/drive_vel_cmd', DriveVelCmd)
        drive_vel_pub.publish(command)

class ArmControl:


    def ra_control_callback(self, channel, msg):
        if (self.arm_control_state != "open-loop"):
            return

        self.arm_type = self.ArmType.RA

        xboxData = Xbox.decode(msg)

        motor_speeds = [quadratic(deadzone(xboxData.left_js_x, 0.15)),
                        quadratic(-deadzone(xboxData.left_js_y, 0.15)),
                        quadratic(-deadzone(xboxData.right_js_y, 0.15)),
                        quadratic(deadzone(xboxData.right_js_x, 0.15)),
                        quadratic(xboxData.right_trigger - xboxData.left_trigger),
                        (xboxData.right_bumper - xboxData.left_bumper)]

        if self.slow_mode:
            # slow down joints a, c, e, and f
            motor_speeds[0] *= 0.5
            motor_speeds[2] *= 0.5
            motor_speeds[4] *= 0.5
            motor_speeds[5] *= 0.5

        openloop_msg = RAOpenLoopCmd()
        openloop_msg.throttle = motor_speeds

        lcm_.publish('/ra_open_loop_cmd', openloop_msg.encode())

        hand_msg = HandCmd()
        hand_msg.finger = xboxData.y - xboxData.a
        hand_msg.grip = xboxData.b - xboxData.x

        lcm_.publish('/hand_open_loop_cmd', hand_msg.encode())

    def send_ra_kill(self):
        if self.arm_type != self.ArmType.RA:
            return

        arm_motor = RAOpenLoopCmd()
        arm_motor.throttle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lcm_.publish('/ra_open_loop_cmd', arm_motor.encode())

        hand_msg = HandCmd()
        hand_msg.finger = 0
        hand_msg.grip = 0
        lcm_.publish('/hand_open_loop_cmd', hand_msg.encode())




def main():
    arm = arm
    drive = Drive(reverse=False)

    ros.init_node("jetson_teleop")

    ros.Subscriber("/drive_control",Joy,drive.teleop_drive_callback)



