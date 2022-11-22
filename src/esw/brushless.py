#!/usr/bin/env python3
import threading
import time as t
import asyncio
import rospy
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import numpy as np
import moteus


# import moteus_pi3hat


class CommandData:
    def __init__(self, position: float = math.nan, velocity: float = 0.0, torque: float = 0.5):
        self.position = position
        self.velocity = velocity
        self.torque = max(0, min(0.5, torque))


class MoteusBridge:
    def __init__(self, can_id: int):
        self.start_time = t.process_time()
        self.controller = moteus.Controller(id=can_id)
        self.state = "Disconnected"
        self.command_lock = threading.Lock()
        self.command = CommandData(position=math.nan, velocity=0.0, torque=0.5)
        self.fault_response = 0

    def set_command(self, command: CommandData):
        self.command_lock.acquire()
        self.command = command
        self.command_lock.release()

    async def send_command(self):
        self.command_lock.acquire()
        command = self.command
        self.command_lock.release()
        state = await self.controller.set_position(
            position=command.position,
            velocity=command.velocity,
            velocity_limit=5,
            maximum_torque=command.torque,
            watchdog_timeout=1,
            query=True,
        )
        self.fault_response = state.values[moteus.Register.FAULT]

    async def has_error(self):
        return self.fault_response != 0

    async def connect(self):
        await self.controller.set_stop()

    async def update(self):
        if str(self.state) == "Armed":
            errors = await self.has_error()

            if errors:
                self.state = "Error"
                return

            await self.send_command()

        elif str(self.state) == "Disconnected":
            await self.connect()
            self.state = "Armed"

        elif str(self.state) == "ErrorState":
            await self.clean_error()
            self.state = "Disconnected"

    async def clean_error(self):
        await self.controller.set_stop()


class DriveApp:
    def __init__(self):
        drive_info_by_name = rospy.get_param("brushless/drive/")
        self.drive_bridge_by_name = {}
        for name, info in drive_info_by_name.items():
            self.drive_bridge_by_name[name] = MoteusBridge(info["id"])
        rover_width = rospy.get_param("rover/width")
        rover_length = rospy.get_param("rover/length")
        self._wheel_distance_inner = rover_width / 2.0
        self._wheel_distance_outer = math.sqrt(((rover_width / 2.0) ** 2) + ((rover_length / 2.0) ** 2))

        wheel_radius = rospy.get_param("wheel/radius")
        _ratio_motor_to_wheel = rospy.get_param("wheel/gear_ratio")
        self._wheels_m_s_to_motor_rad_ratio = (1 / wheel_radius) * _ratio_motor_to_wheel

        _max_speed_m_s = rospy.get_param("rover/max_speed")
        assert _max_speed_m_s > 0, "rover/max_speed config must be greater than 0"

        max_motor_rad_s = _max_speed_m_s * self._wheels_m_s_to_motor_rad_ratio
        measured_max_motor_rad_s = 1 * 2 * math.pi  # Should not be changed. Derived from testing.
        self._max_motor_speed_rad_s = min(measured_max_motor_rad_s, max_motor_rad_s)

        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    def _process_twist_message(self, ros_msg: Twist):
        """Converts the twist message into rad/s to turn per wheel.
        Then tells the wheels to move at that speed.
        :param ros_msg: A Twist message of the rover to turn."""
        forward = ros_msg.linear.x
        turn = ros_msg.angular.z

        turn_difference_inner = turn * self._wheel_distance_inner
        turn_difference_outer = turn * self._wheel_distance_outer

        left_rad_inner = (forward - turn_difference_inner) * self._wheels_m_s_to_motor_rad_ratio
        right_rad_inner = (forward + turn_difference_inner) * self._wheels_m_s_to_motor_rad_ratio
        left_rad_outer = (forward - turn_difference_outer) * self._wheels_m_s_to_motor_rad_ratio
        right_rad_outer = (forward + turn_difference_outer) * self._wheels_m_s_to_motor_rad_ratio

        # Ignore inner since outer > inner always
        larger_abs_rad_s = max(abs(left_rad_outer), abs(right_rad_outer))
        if larger_abs_rad_s > self._max_motor_speed_rad_s:
            change_ratio = self._max_motor_speed_rad_s / larger_abs_rad_s
            left_rad_inner *= change_ratio
            right_rad_inner *= change_ratio
            left_rad_outer *= change_ratio
            right_rad_outer *= change_ratio

        for name, bridge in self.drive_bridge_by_name.items():
            bridge.start_time = t.process_time()

            if name == "front_left" or name == "back_left":
                commanded_velocity = left_rad_outer
            elif name == "middle_left":
                commanded_velocity = left_rad_inner
            elif name == "front_right" or name == "back_right":
                commanded_velocity = right_rad_outer
            elif name == "middle_right":
                commanded_velocity = right_rad_inner
            else:
                rospy.logerr(f"Invalid name {name}")
                continue

            if bridge.state == "Armed":
                bridge.set_command(CommandData(position=math.nan, velocity=commanded_velocity, torque=0.5))

    async def run(self):
        previously_lost_communication = True
        while not rospy.is_shutdown():
            for name, bridge in self.drive_bridge_by_name.items():
                # if name != "middle_left" and name != "front_right":
                #     continue
                watchdog = t.process_time() - bridge.start_time
                lost_communication = watchdog > 1.0
                if lost_communication:
                    if not previously_lost_communication:
                        rospy.loginfo("Lost communication")
                        previously_lost_communication = True
                    bridge.command = CommandData(position=math.nan, velocity=0.0, torque=0.5)
                elif previously_lost_communication:
                    previously_lost_communication = False
                    rospy.loginfo("Regained communication")
                await bridge.update()


class Application:
    def __init__(self):
        rospy.init_node(f"brushless")
        self.drive_app = DriveApp()

    def run(self):
        """Creates MoteusBridge objects and starts the task associated with each one.
        Tasks run concurrently according to asyncio.gather."""
        asyncio.run(self.drive_app.run())


def main():
    app = Application()
    app.run()


if __name__ == "__main__":
    main()
