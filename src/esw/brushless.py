#!/usr/bin/env python3
import threading
import time as t
import asyncio
import rospy
import math
from geometry_msgs.msg import Twist
from typing import NoReturn
import moteus


class CommandData:
    DEFAULT_TORQUE = 0.3
    MAX_TORQUE = 0.5
    POSITION_FOR_VELOCITY_CONTROL = math.nan
    VELOCITY_LIMIT = 5  # TODO - Change after regenerative braking is solved
    ZERO_VELOCITY = 0.0

    def __init__(
        self,
        position: float = POSITION_FOR_VELOCITY_CONTROL,
        velocity: float = ZERO_VELOCITY,
        torque: float = DEFAULT_TORQUE,
    ):
        self.position = position
        self.velocity = max(-CommandData.VELOCITY_LIMIT, min(CommandData.VELOCITY_LIMIT, velocity))
        self.torque = max(0, min(CommandData.MAX_TORQUE, torque))


class MoteusBridge:

    ERROR_CODE_DICTIONARY = {
        1: "DmaStreamTransferError",
        2: "DmaStreamFifiError",
        3: "UartOverrunError",
        4: "UartFramingError",
        5: "UartNoiseError",
        6: "UartBufferOverrunError",
        7: "UartParityError",
        32: "CalibrationFault",
        33: "MotorDriverFault",
        34: "OverVoltage",
        35: "EncoderFault",
        36: "MotorNotConfigured",
        37: "PwmCycleOverrun",
        38: "OverTemperature",
        39: "StartOutsideLimit",
        40: "UnderVoltage",
        41: "ConfigChanged",
        42: "ThetaInvalid",
        43: "PositionInvalid",
    }
    UNKNOWN_ERROR_NAME = "Unknown Error"

    MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S = 0.01
    ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.1

    DISCONNECTED_STATE = "Disconnected"
    ARMED_STATE = "Armed"
    ERROR_STATE = "Error"

    def __init__(self, can_id: int, transport):

        self._can_id = can_id
        self.controller = moteus.Controller(id=can_id, transport=transport)
        self.state = MoteusBridge.DISCONNECTED_STATE
        self.command_lock = threading.Lock()
        self._command = CommandData(
            position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
            velocity=CommandData.ZERO_VELOCITY,
            torque=CommandData.MAX_TORQUE,
        )
        self._prev_error_name = MoteusBridge.UNKNOWN_ERROR_NAME

    def set_command(self, command: CommandData) -> None:
        """
        Changes the values of the arguments of set_position for when it is next called.
        :param command: contains arguments that are used in the moteus controller.set_position function
        :return: none
        """
        self.command_lock.acquire()
        self._command = command
        self.command_lock.release()

    async def _send_command(self) -> None:
        """
        Calls controller.set_position (which controls the moteus over CAN) with previously the most recent requested
        commands. If the message has not been received within a certain amount of time, disconnected state is entered.
        Otherwise, error state is entered. State must be in armed state.
        :return: none
        """
        assert self.state == MoteusBridge.ARMED_STATE
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
                    watchdog_timeout=MoteusBridge.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
                    query=True,
                ),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
        except asyncio.TimeoutError:
            if self.state != MoteusBridge.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to send command")
            self.state = MoteusBridge.DISCONNECTED_STATE
        self._check_has_error(state.values[moteus.Register.FAULT])

    def _check_has_error(self, fault_response: int) -> None:
        """
        Checks if the controller has encountered an error. If there is an error, error state is set.
        :param fault_response: the value read in from the fault register of the moteus CAN message
        """
        has_error = fault_response != 0
        if has_error:
            try:
                error_description = MoteusBridge.ERROR_CODE_DICTIONARY[fault_response]
            except KeyError:
                error_description = MoteusBridge.UNKNOWN_ERROR_NAME

            if self._prev_error_name != error_description:
                rospy.logerr(f"CAN ID {self._can_id} has encountered an error: {error_description}")
                self._prev_error_name = error_description

            self.state = MoteusBridge.ERROR_STATE
        else:
            self.state = MoteusBridge.ARMED_STATE

    async def _connect(self) -> None:
        """
        Attempts to establish a connection to the moteus. State must be in error or disconnected state.
        """
        try:
            assert self.state != MoteusBridge.ARMED_STATE
            await asyncio.wait_for(
                self.controller.set_stop(), timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S
            )
            # TODO - verify that this will timeout if controller is disconnected
            state = await asyncio.wait_for(
                self.controller.query(), timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S
            )
            # state = await asyncio.wait_for(
            #     self.controller.set_position(
            #         position=math.nan,
            #         velocity=CommandData.ZERO_VELOCITY,
            #         velocity_limit=CommandData.VELOCITY_LIMIT,
            #         maximum_torque=CommandData.MAX_TORQUE,
            #         watchdog_timeout=MoteusBridge.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
            #         query=True,
            #     ),
            #     timeout=self.TIME_INDICATING_DISCONNECTED,
            # )
        except asyncio.TimeoutError:
            if self.state != MoteusBridge.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to connect")
            self.state = MoteusBridge.DISCONNECTED_STATE
            return
        self._check_has_error(state.values[moteus.Register.FAULT])

    async def update(self) -> None:
        """
        Determines actions based on state. If in armed state, commands will be sent to the moteus.
        If in disconnected or error state, attempts will be made to return to armed state.
        """
        if self.state == MoteusBridge.ARMED_STATE:
            await self._send_command()
        elif self.state == MoteusBridge.DISCONNECTED_STATE or self.state == MoteusBridge.ERROR_STATE:
            await self._connect()


class DriveApp:
    BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S = 1

    def __init__(self):
        drive_info_by_name = rospy.get_param("brushless/drive")
        self.drive_bridge_by_name = {}
        using_pi3_hat = rospy.get_param("brushless/using_pi3_hat")
        if using_pi3_hat:
            # import moteus_pi3hat
            # transport = moteus_pi3hat.Pi3HatRouter(
            #     servo_bus_map={
            #         1: [11],
            #         2: [12],
            #         3: [13],
            #         4: [14],
            #     },
            # )
            pass
        else:
            transport = moteus.Fdcanusb()

        for name, info in drive_info_by_name.items():
            self.drive_bridge_by_name[name] = MoteusBridge(info["id"], transport)
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
        self._last_updated_time = t.time()

        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    def _process_twist_message(self, ros_msg: Twist) -> None:
        """
        Converts a Twist message to individual motor speeds.
        :param ros_msg: Has linear x and angular z velocity components.
        """
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

        self._last_updated_time = t.time()
        for name, bridge in self.drive_bridge_by_name.items():

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

            if bridge.state == MoteusBridge.ARMED_STATE:
                bridge.set_command(
                    CommandData(
                        position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
                        velocity=commanded_velocity,
                        torque=CommandData.DEFAULT_TORQUE,
                    )
                )

    async def run(self) -> NoReturn:
        """
        Runs an infinite loop and only gives commands to the moteus (by updating the bridge) if communication is still
        maintained between the basestation and the rover node.
        """
        previously_lost_communication = True
        while not rospy.is_shutdown():
            for name, bridge in self.drive_bridge_by_name.items():
                time_diff_since_updated = t.time() - self._last_updated_time
                lost_communication = time_diff_since_updated > DriveApp.BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S
                if lost_communication:
                    if not previously_lost_communication:
                        rospy.loginfo("Lost communication")
                        previously_lost_communication = True
                    bridge.set_command(
                        CommandData(
                            position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
                            velocity=CommandData.ZERO_VELOCITY,
                            torque=CommandData.DEFAULT_TORQUE,
                        )
                    )
                elif previously_lost_communication:
                    previously_lost_communication = False
                    rospy.loginfo("Regained communication")
                await bridge.update()
        assert False


class Application:
    def __init__(self):
        rospy.init_node(f"brushless")
        self.drive_app = DriveApp()

    def run(self) -> NoReturn:
        """
        Allows the functions to run.
        :return:
        """
        asyncio.run(self.drive_app.run())


def main():
    app = Application()
    app.run()


if __name__ == "__main__":
    main()
