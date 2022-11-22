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
    MAX_TORQUE = 0.5
    VELOCITY_LIMIT = 5  # TODO - Change after regenerative braking is solved

    def __init__(self, position: float = math.nan, velocity: float = 0.0, torque: float = MAX_TORQUE):
        self.position = position
        self.velocity = max(-CommandData.VELOCITY_LIMIT, min(CommandData.VELOCITY_LIMIT, velocity))
        self.torque = max(0, min(CommandData.MAX_TORQUE, torque))


class MoteusBridge:
    def __init__(self, can_id: int):
        self.CAN_ID = can_id
        self.last_updated_time = t.time()
        self.controller = moteus.Controller(id=can_id)
        self.state = "Disconnected"
        self.command_lock = threading.Lock()
        self._command = CommandData(position=math.nan, velocity=0.0, torque=CommandData.MAX_TORQUE)
        self._fault_response = 0
        self.TIME_INDICATING_DISCONNECTED = 0.01
        self.WATCHDOG_LIMIT = 0.1

    def set_command(self, command: CommandData) -> None:
        self.command_lock.acquire()
        self._command = command
        self.command_lock.release()

    async def _send_command(self) -> None:
        self.command_lock.acquire()
        command = self._command
        self.command_lock.release()
        try:
            state = await asyncio.wait_for(
                self.controller.set_position(
                    position=command.position,
                    velocity=command.velocity,
                    velocity_limit=CommandData.VELOCITY_LIMIT,
                    maximum_torque=command.torque,
                    watchdog_timeout=self.WATCHDOG_LIMIT,
                    query=True,
                ),
                timeout=self.TIME_INDICATING_DISCONNECTED,
            )
            self._fault_response = state.values[moteus.Register.FAULT]
        except asyncio.TimeoutError:
            if self.state != "Disconnected":
                rospy.logerr(f"CAN ID {self.CAN_ID} disconnected when trying to send command")
            self.state = "Disconnected"

    def _has_error(self) -> bool:
        return self._fault_response != 0

    async def _connect(self) -> None:
        try:
            await asyncio.wait_for(self.controller.set_stop(), timeout=0.1)
            await asyncio.wait_for(
                self.controller.set_position(
                    position=math.nan,
                    velocity=0,
                    velocity_limit=CommandData.VELOCITY_LIMIT,
                    maximum_torque=CommandData.MAX_TORQUE,
                    watchdog_timeout=self.WATCHDOG_LIMIT,
                    query=True,
                ),
                timeout=self.TIME_INDICATING_DISCONNECTED,
            )
        except asyncio.TimeoutError:
            if self.state != "Disconnected":
                rospy.logerr(f"CAN ID {self.CAN_ID} disconnected when trying to connect")
            self.state = "Disconnected"
            return
        self.state = "Armed"

    async def update(self) -> None:
        if self.state == "Armed":
            errors = self._has_error()

            if errors:
                rospy.logerr(f"CAN ID {self.CAN_ID} has error {self._fault_response}")
                self.state = "Error"
                return

            await self._send_command()

        elif self.state == "Disconnected":
            await self._connect()

        elif self.state == "Error":
            await self.clean_error()

    async def clean_error(self) -> None:
        try:
            await asyncio.wait_for(self.controller.set_stop(), timeout=self.TIME_INDICATING_DISCONNECTED)
            state = await asyncio.wait_for(
                self.controller.set_position(
                    position=math.nan,
                    velocity=0,
                    velocity_limit=CommandData.VELOCITY_LIMIT,
                    maximum_torque=CommandData.MAX_TORQUE,
                    watchdog_timeout=self.WATCHDOG_LIMIT,
                    query=True,
                ),
                timeout=self.TIME_INDICATING_DISCONNECTED,
            )
        except asyncio.TimeoutError:
            if self.state != "Disconnected":
                rospy.logerr(f"CAN ID {self.CAN_ID} disconnected when trying to clean error")
            self.state = "Disconnected"
            return
        self._fault_response = state.values[moteus.Register.FAULT]
        if self._has_error():
            self.state = "Error"
        else:
            self.state = "Armed"


class DriveApp:
    def __init__(self):
        drive_info_by_name = rospy.get_param("brushless/drive/")
        self.drive_bridge_by_name = {}
        for name, info in drive_info_by_name.items():
            self.drive_bridge_by_name[name] = MoteusBridge(info["id"])
        rover_width = rospy.get_param("rover/width")
        rover_length = rospy.get_param("rover/length")
        self.WHEEL_DISTANCE_INNER = rover_width / 2.0
        self.WHEEL_DISTANCE_OUTER = math.sqrt(((rover_width / 2.0) ** 2) + ((rover_length / 2.0) ** 2))

        _ratio_motor_to_wheel = rospy.get_param("wheel/gear_ratio")
        self.WHEELS_M_S_TO_MOTOR_RAD_RATIO = (1 / rospy.get_param("wheel/radius")) * rospy.get_param("wheel/gear_ratio")

        _max_speed_m_s = rospy.get_param("rover/max_speed")
        assert _max_speed_m_s > 0, "rover/max_speed config must be greater than 0"

        max_motor_rad_s = _max_speed_m_s * self.WHEELS_M_S_TO_MOTOR_RAD_RATIO
        self.MEASURED_MAX_MOTOR_RAD_S = 1 * 2 * math.pi  # Should not be changed. Derived from testing.
        self._max_motor_speed_rad_s = min(self.MEASURED_MAX_MOTOR_RAD_S, max_motor_rad_s)
        self.WATCHDOG_TIMEOUT = 1
        self.DEFAULT_TORQUE = 0.3

        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    def _process_twist_message(self, ros_msg: Twist) -> None:
        forward = ros_msg.linear.x
        turn = ros_msg.angular.z

        turn_difference_inner = turn * self.WHEEL_DISTANCE_INNER
        turn_difference_outer = turn * self.WHEEL_DISTANCE_OUTER

        left_rad_inner = (forward - turn_difference_inner) * self.WHEELS_M_S_TO_MOTOR_RAD_RATIO
        right_rad_inner = (forward + turn_difference_inner) * self.WHEELS_M_S_TO_MOTOR_RAD_RATIO
        left_rad_outer = (forward - turn_difference_outer) * self.WHEELS_M_S_TO_MOTOR_RAD_RATIO
        right_rad_outer = (forward + turn_difference_outer) * self.WHEELS_M_S_TO_MOTOR_RAD_RATIO

        # Ignore inner since outer > inner always
        larger_abs_rad_s = max(abs(left_rad_outer), abs(right_rad_outer))
        if larger_abs_rad_s > self._max_motor_speed_rad_s:
            change_ratio = self._max_motor_speed_rad_s / larger_abs_rad_s
            left_rad_inner *= change_ratio
            right_rad_inner *= change_ratio
            left_rad_outer *= change_ratio
            right_rad_outer *= change_ratio

        for name, bridge in self.drive_bridge_by_name.items():
            bridge.last_updated_time = t.time()

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
                bridge.set_command(
                    CommandData(position=math.nan, velocity=commanded_velocity, torque=self.DEFAULT_TORQUE)
                )

    async def run(self) -> None:
        previously_lost_communication = True
        while not rospy.is_shutdown():
            for name, bridge in self.drive_bridge_by_name.items():
                time_diff_since_updated = t.time() - bridge.last_updated_time
                lost_communication = time_diff_since_updated > self.WATCHDOG_TIMEOUT
                if lost_communication:
                    if not previously_lost_communication:
                        rospy.loginfo("Lost communication")
                        previously_lost_communication = True
                    bridge.set_command(CommandData(position=math.nan, velocity=0.0, torque=self.DEFAULT_TORQUE))
                elif previously_lost_communication:
                    previously_lost_communication = False
                    rospy.loginfo("Regained communication")
                await bridge.update()


class Application:
    def __init__(self):
        rospy.init_node(f"brushless")
        self.drive_app = DriveApp()

    def run(self) -> None:
        asyncio.run(self.drive_app.run())


def main():
    app = Application()
    app.run()


if __name__ == "__main__":
    main()
