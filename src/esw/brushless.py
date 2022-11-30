#!/usr/bin/env python3
import threading
import time as t
import asyncio
import rospy
import math
from geometry_msgs.msg import Twist
from typing import List
from sensor_msgs.msg import JointState
from mrover.msg import MoteusState as MoteusStateMsg, MotorsStatus
import moteus


class CommandData:
    DEFAULT_TORQUE = 0.3
    MAX_TORQUE = 0.5
    POSITION_FOR_VELOCITY_CONTROL = math.nan
    VELOCITY_LIMIT = 5
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


class MoteusData:
    def __init__(self, position: float, velocity: float, torque: float):
        self.position = position
        self.velocity = velocity
        self.torque = torque


class MoteusState:
    DISCONNECTED_STATE = "Disconnected"
    ARMED_STATE = "Armed"
    ERROR_STATE = "Error"

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
    NO_ERROR_NAME = "No Error"

    def __init__(self, state: str, error_name: str):
        self.state = state
        self.error_name = error_name


class MoteusBridge:

    MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S = 0.01
    ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.1

    def __init__(self, can_id: int, transport):

        self._can_id = can_id
        self.controller = moteus.Controller(id=can_id, transport=transport)
        self.command_lock = threading.Lock()
        self._command = CommandData(
            position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
            velocity=CommandData.ZERO_VELOCITY,
            torque=CommandData.DEFAULT_TORQUE,
        )
        self.moteus_state = MoteusState(state=MoteusState.DISCONNECTED_STATE, error_name=MoteusState.NO_ERROR_NAME)
        self.moteus_data = MoteusData(
            position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
            velocity=CommandData.ZERO_VELOCITY,
            torque=CommandData.DEFAULT_TORQUE,
        )

    def set_command(self, command: CommandData) -> None:
        """
        Changes the values of the arguments of set_position for when it is next called.
        :param command: contains arguments that are used in the moteus controller.set_position function
        """
        self.command_lock.acquire()
        self._command = command
        self.command_lock.release()

    async def _send_command(self) -> None:
        """
        Calls controller.set_position (which controls the moteus over CAN) with previously the most recent requested
        commands. If the message has not been received within a certain amount of time, disconnected state is entered.
        Otherwise, error state is entered. State must be in armed state.
        """
        assert self.moteus_state.state == MoteusState.ARMED_STATE
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
                timeout=MoteusBridge.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
            self.moteus_data = MoteusData(
                position=state.values[moteus.Register.POSITION],
                velocity=state.values[moteus.Register.VELOCITY],
                torque=state.values[moteus.Register.TORQUE],
            )
            self._check_has_error(state.values[moteus.Register.FAULT])
        except asyncio.TimeoutError:
            if self.moteus_state.state != MoteusState.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to send command")
            self._change_state(MoteusState.DISCONNECTED_STATE)
            return

    def _check_has_error(self, fault_response: int) -> None:
        """
        Checks if the controller has encountered an error. If there is an error, error state is set.
        :param fault_response: the value read in from the fault register of the moteus CAN message
        """
        has_error = fault_response != 0
        if has_error:

            self._change_state(MoteusState.ERROR_STATE)

            try:
                error_description = MoteusState.ERROR_CODE_DICTIONARY[fault_response]
            except KeyError:
                error_description = MoteusState.NO_ERROR_NAME

            if self.moteus_state.error_name != error_description:
                rospy.logerr(f"CAN ID {self._can_id} has encountered an error: {error_description}")
                self.moteus_state.error_name = error_description

        else:
            self._change_state(MoteusState.ARMED_STATE)

    async def _connect(self) -> None:
        """
        Attempts to establish a connection to the moteus. State must be in error or disconnected state.
        """
        try:
            assert self.moteus_state.state != MoteusState.ARMED_STATE
            await asyncio.wait_for(
                self.controller.set_stop(), timeout=MoteusBridge.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S
            )
            state = await asyncio.wait_for(
                self.controller.set_position(
                    position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
                    velocity=CommandData.ZERO_VELOCITY,
                    velocity_limit=CommandData.VELOCITY_LIMIT,
                    maximum_torque=CommandData.MAX_TORQUE,
                    watchdog_timeout=MoteusBridge.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
                    query=True,
                ),
                timeout=MoteusBridge.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
            self._check_has_error(state.values[moteus.Register.FAULT])
        except asyncio.TimeoutError:
            if self.moteus_state.state != MoteusState.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to connect")
            self._change_state(MoteusState.DISCONNECTED_STATE)
            return

    def _change_state(self, state: str) -> None:
        """
        Changes the state as needed. If no longer armed, then the data will be set to
        CommandData.POSITION_FOR_VELOCITY_CONTROL.
        If no longer in an error state, error name is set to no error.
        :param state: Must be disconnected, armed, or error
        """

        if state == MoteusState.ARMED_STATE:
            self.moteus_state.state = MoteusState.ARMED_STATE
            self.moteus_state.error_name = MoteusState.NO_ERROR_NAME
        elif state == MoteusState.DISCONNECTED_STATE:
            self.moteus_state.state = MoteusState.DISCONNECTED_STATE
            self.moteus_state.error_name = MoteusState.NO_ERROR_NAME
            self.moteus_data = MoteusData(position=math.nan, velocity=math.nan, torque=math.nan)
        else:
            self.moteus_state.state = MoteusState.ERROR_STATE
            self.moteus_data = MoteusData(position=math.nan, velocity=math.nan, torque=math.nan)

    async def update(self) -> None:
        """
        Determines actions based on state. If in armed state, commands will be sent to the moteus.
        If in disconnected or error state, attempts will be made to return to armed state.
        """
        if self.moteus_state.state == MoteusState.ARMED_STATE:
            await self._send_command()
        elif (
            self.moteus_state.state == MoteusState.DISCONNECTED_STATE
            or self.moteus_state.state == MoteusState.ERROR_STATE
        ):
            await self._connect()


class MotorsManager:
    BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S = 1

    def __init__(self, motor_info_by_name, motor_names: List[str], transport, publish_topic: str):
        self._motor_bridge_by_name = {}
        for name, info in motor_info_by_name.items():
            self._motor_bridge_by_name[name] = MoteusBridge(info["id"], transport)

        self._last_updated_time = t.time()
        self._motor_names = motor_names

        self._motors_status_publisher = rospy.Publisher(publish_topic, MotorsStatus, queue_size=1)
        self._motors_status = MotorsStatus(
            name=[name for name in self._motor_names],
            joint_states=JointState(
                name=[name for name in self._motor_names],
                position=[self._motor_bridge_by_name[name].moteus_data.position for name in self._motor_names],
                velocity=[self._motor_bridge_by_name[name].moteus_data.velocity for name in self._motor_names],
                effort=[self._motor_bridge_by_name[name].moteus_data.torque for name in self._motor_names],
            ),
            moteus_states=MoteusStateMsg(
                name=[name for name in self._motor_names],
                state=[self._motor_bridge_by_name[name].moteus_state.state for name in self._motor_names],
                error=[self._motor_bridge_by_name[name].moteus_state.error_name for name in self._motor_names],
            ),
        )

    def update_time(self) -> None:
        """
        Updates the last updated time
        """
        self._last_updated_time = t.time()

    async def run(self) -> None:
        """
        Constantly updates the bridges.
        """
        previously_lost_communication = True
        while not rospy.is_shutdown():
            for name, bridge in self._motor_bridge_by_name.items():
                time_diff_since_updated = t.time() - self._last_updated_time
                lost_communication = (
                    time_diff_since_updated > MotorsManager.BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S
                )
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

    def update_command_data(self, command_data_list: List[CommandData]) -> None:
        """
        Updates the commands
        :param command_data_list: Has an array of command data that represents the commands for each motor.
        """
        assert len(command_data_list) == len(self._motor_names)
        for i, command_data in enumerate(command_data_list):
            motor_name = self._motor_names[i]
            if self._motor_bridge_by_name[motor_name].moteus_state.state == MoteusState.ARMED_STATE:
                self._motor_bridge_by_name[motor_name].set_command(command_data)


class ArmManager:
    ARM_NAMES = ["joint_a", "joint_c", "joint_d", "joint_e", "joint_f", "gripper"]

    def __init__(self):
        self._arm_command_data_list = [
            CommandData(
                CommandData.POSITION_FOR_VELOCITY_CONTROL, CommandData.ZERO_VELOCITY, CommandData.DEFAULT_TORQUE
            )
            for name in ArmManager.ARM_NAMES
        ]

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
            transport = None

        arm_info_by_name = rospy.get_param("brushless/arm")
        self._motors_manager = MotorsManager(arm_info_by_name, ArmManager.ARM_NAMES, transport, "arm_status")
        rospy.Subscriber("ra_cmd", JointState, self._process_ra_cmd)

    def _process_ra_cmd(self, ros_msg: JointState) -> None:
        """
        Process a JointState command controls individual motor speeds.
        :param ros_msg: Has information to control the joints controlled by both brushed and brushless motors.
        We only care about the joints controlled by brushless motors.
        """
        self._motors_manager.update_time()

        for i, name in enumerate(ros_msg.name):
            if name in ArmManager.ARM_NAMES:
                position, velocity, torque = (
                    ros_msg.position[i],
                    ros_msg.velocity[i],
                    ros_msg.effort[i],
                )
                # TODO - For now, ignore position and torque since we don't trust teleop.
                # We can assume that the ros_msg sends velocity commands from -1 to 1.
                # self._arm_command_data_list[ArmManager.ARM_NAMES.index(name)].position = position
                self._arm_command_data_list[ArmManager.ARM_NAMES.index(name)].velocity = velocity
                # self._arm_command_data_list[ArmManager.ARM_NAMES.index(name)].torque = torque

        self._motors_manager.update_command_data(self._arm_command_data_list)

    async def run(self) -> None:
        await self._motors_manager.run()


class DriveManager:
    DRIVE_NAMES = ["FrontLeft", "FrontRight", "MiddleLeft", "MiddleRight", "BackLeft", "BackRight"]

    def __init__(self):

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

        self._drive_command_data_list = [
            CommandData(
                CommandData.POSITION_FOR_VELOCITY_CONTROL, CommandData.ZERO_VELOCITY, CommandData.DEFAULT_TORQUE
            )
            for name in DriveManager.DRIVE_NAMES
        ]

        using_pi3_hat = rospy.get_param("brushless/using_pi3_hat")
        if using_pi3_hat:
            pass
            # import moteus_pi3hat
            # transport = moteus_pi3hat.Pi3HatRouter(
            #     servo_bus_map={
            #         1: [11],
            #         2: [12],
            #         3: [13],
            #         4: [14],
            #     },
            # )
        else:
            transport = None

        drive_info_by_name = rospy.get_param("brushless/drive")
        self._motors_manager = MotorsManager(drive_info_by_name, DriveManager.DRIVE_NAMES, transport, "drive_status")
        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    def _process_twist_message(self, ros_msg: Twist) -> None:
        """
        Processes a Twist message and controls individual motor speeds.
        :param ros_msg: Has linear x and angular z velocity components.
        """
        self._motors_manager.update_time()

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

        drive_command_velocities = [
            left_rad_outer,
            right_rad_outer,
            left_rad_inner,
            right_rad_inner,
            left_rad_outer,
            right_rad_outer,
        ]

        for i, new_velocity in enumerate(drive_command_velocities):
            self._drive_command_data_list[i].velocity = new_velocity

        self._motors_manager.update_command_data(self._drive_command_data_list)

    async def run(self) -> None:
        """
        Runs an infinite loop and only gives commands to the moteus (by updating the bridge) if communication is still
        maintained between the basestation and the rover node.
        """
        await self._motors_manager.run()


class Application:
    def __init__(self):
        rospy.init_node(f"brushless")
        self._drive_manager = DriveManager()
        self._arm_manager = ArmManager()

    def run(self) -> None:
        """
        Allows the functions to run.
        """
        asyncio.run(self.run_tasks())

    async def run_tasks(self) -> None:
        """
        Creates an async function to run both drive and arm tasks
        """
        coroutines = [self._drive_manager.run(), self._arm_manager.run()]
        await asyncio.gather(*coroutines, return_exceptions=True)


def main():
    app = Application()
    app.run()


if __name__ == "__main__":
    main()
