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
    VELOCITY_LIMIT = 5  # TODO - KEEP FOR NOW. DO NOT CHANGE UNTIL WE ARE FINE WITH CHANGING MAX SPEED
    ZERO_VELOCITY = 0.0

    def __init__(
        self,
        position: float,
        velocity: float,
        torque: float,
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
        99: "Moteus Unpowered or Not Found",
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
        # self._command is the next command that this ROS node will send to the Moteus
        # once self._send_command is called.
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
        Changes the values of the arguments of set_position for when it is used in self._send_command.
        :param command: contains arguments that are used in the moteus controller.set_position function
        """
        with self.command_lock:
            self._command = command

    async def _send_desired_command(self) -> None:
        """
        Calls controller.set_position (which controls the moteus over CAN) with previously the most recent requested
        commands. If the message has not been received within a certain amount of time, disconnected state is entered.
        Otherwise, error state is entered. State must be in armed state.
        """
        assert self.moteus_state.state == MoteusState.ARMED_STATE
        with self.command_lock:
            command = self._command
        try:
            await self._send_command(command)
        except asyncio.TimeoutError:
            if self.moteus_state.state != MoteusState.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to send command")
            self._change_state(MoteusState.DISCONNECTED_STATE)

    async def _send_command(self, command: CommandData) -> None:
        """
        Calls controller.set_position (which controls the moteus over CAN) with the requested command.
        If the message has not been received within a certain amount of time, disconnected state is entered.
        Otherwise, error state is entered.

        It is expected that this may throw an error if there is a timeout.
        """
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

        moteus_not_found = state is None or not hasattr(state, "values") or moteus.Register.FAULT not in state.values
        if moteus_not_found:
            self._check_has_error(99)
        else:
            self._check_has_error(state.values[moteus.Register.FAULT])

            self.moteus_data = MoteusData(
                position=state.values[moteus.Register.POSITION],
                velocity=state.values[moteus.Register.VELOCITY],
                torque=state.values[moteus.Register.TORQUE],
            )

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

            command = CommandData(
                position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
                velocity=CommandData.ZERO_VELOCITY,
                torque=CommandData.DEFAULT_TORQUE,
            )
            await self._send_command(command)

        except asyncio.TimeoutError:
            if self.moteus_state.state != MoteusState.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to connect")
            self._change_state(MoteusState.DISCONNECTED_STATE)

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
            await self._send_desired_command()
        elif (
            self.moteus_state.state == MoteusState.DISCONNECTED_STATE
            or self.moteus_state.state == MoteusState.ERROR_STATE
        ):
            await self._connect()


class MotorsManager:
    BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S = 1

    def __init__(self, motor_controller_info_by_name, transport, publish_topic: str):
        self._motor_bridge_by_name = {}
        self._motor_names = []
        for name, info in motor_controller_info_by_name.items():
            self._motor_names.append(name)
            self._motor_bridge_by_name[name] = MoteusBridge(info["id"], transport)

        self._last_updated_time = t.time()

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

    async def run(self) -> None:
        """
        Constantly updates the bridges.
        """
        previously_lost_communication = True
        while not rospy.is_shutdown():
            await asyncio.sleep(0)  # Causes a task switch
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

                index = self._motor_names.index(name)

                self._motors_status.joint_states.position[index] = bridge.moteus_data.position
                self._motors_status.joint_states.velocity[index] = bridge.moteus_data.velocity
                self._motors_status.joint_states.effort[index] = bridge.moteus_data.torque
                self._motors_status.moteus_states.state[index] = bridge.moteus_state.state
                self._motors_status.moteus_states.error[index] = bridge.moteus_state.error_name

            self._motors_status_publisher.publish(self._motors_status)

    def update_command_data(self, command_data_list: List[CommandData]) -> None:
        """
        Updates the commands
        :param command_data_list: Has an array of command data that represents the commands for each motor.
        """
        self._last_updated_time = t.time()

        assert len(command_data_list) == len(self._motor_names)
        for i, command_data in enumerate(command_data_list):
            motor_name = self._motor_names[i]
            if self._motor_bridge_by_name[motor_name].moteus_state.state == MoteusState.ARMED_STATE:
                self._motor_bridge_by_name[motor_name].set_command(command_data)


class ArmManager:
    def __init__(self):

        arm_controller_info_by_name = rospy.get_param("brushless/arm/controllers")

        self._arm_names = []
        self._arm_command_data_list = []
        self._max_rps_by_name = {}
        for name, info in arm_controller_info_by_name.items():
            self._arm_names.append(name)
            self._max_rps_by_name[name] = info["max_rps"]
            self._arm_command_data_list.append(
                CommandData(
                    CommandData.POSITION_FOR_VELOCITY_CONTROL, CommandData.ZERO_VELOCITY, CommandData.DEFAULT_TORQUE
                )
            )

        using_pi3_hat = rospy.get_param("brushless/using_pi3_hat")
        transport = None
        if using_pi3_hat:
            import moteus_pi3hat

            transport = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map={1: [info["id"] for name, info in arm_controller_info_by_name.items()]}
            )
        else:
            transport = None

        self._motors_manager = MotorsManager(arm_controller_info_by_name, transport, "arm_status")
        rospy.Subscriber("ra_cmd", JointState, self._process_ra_cmd)

    def _process_ra_cmd(self, ros_msg: JointState) -> None:
        """
        Process a JointState command controls individual motor speeds.
        :param ros_msg: Has information to control the joints controlled by both brushed and brushless motors.
        We only care about the joints controlled by brushless motors.
        """

        for i, name in enumerate(ros_msg.name):
            if name in self._arm_names:
                position, velocity, torque = (
                    ros_msg.position[i],
                    ros_msg.velocity[i],
                    ros_msg.effort[i],
                )

                # We usually assume that the ros_msg sends velocity commands from -1 to 1,
                # but change it just in case.
                if abs(velocity) > 1:
                    rospy.logerr("Commanded arm velocity is too low or high (should be [-1, 1]")
                    velocity = max(-1, min(1, velocity))
                velocity *= self._max_rps_by_name[name]
                # self._arm_command_data_list[self._arm_names.index(name)].position = position
                self._arm_command_data_list[self._arm_names.index(name)].velocity = velocity
                # self._arm_command_data_list[self._arm_names.index(name)].torque = torque

        self._motors_manager.update_command_data(self._arm_command_data_list)

    async def run(self) -> None:
        await self._motors_manager.run()


class DriveManager:
    def __init__(self):

        rover_width = rospy.get_param("rover/width")
        rover_length = rospy.get_param("rover/length")
        self.WHEEL_DISTANCE_INNER = rover_width / 2.0
        self.WHEEL_DISTANCE_OUTER = math.sqrt(((rover_width / 2.0) ** 2) + ((rover_length / 2.0) ** 2))

        _ratio_motor_to_wheel = rospy.get_param("wheel/gear_ratio")
        self.WHEELS_M_S_TO_MOTOR_RAD_RATIO = (1 / rospy.get_param("wheel/radius")) * rospy.get_param("wheel/gear_ratio")

        self.max_motor_rps = rospy.get_param("brushless/drive/max_motor_rps")
        self._max_motor_speed_rad_s = self.max_motor_rps * 2 * math.pi

        drive_controller_info_by_name = rospy.get_param("brushless/drive/controllers")
        self._drive_names = []
        self._drive_command_data_list = []
        for name, info in drive_controller_info_by_name.items():
            self._drive_names.append(name)
            self._drive_command_data_list.append(
                CommandData(
                    CommandData.POSITION_FOR_VELOCITY_CONTROL, CommandData.ZERO_VELOCITY, CommandData.DEFAULT_TORQUE
                )
            )

        using_pi3_hat = rospy.get_param("brushless/using_pi3_hat")
        transport = None
        if using_pi3_hat:
            # TODO - Uncomment once pi3hat is supported
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

        self._motors_manager = MotorsManager(drive_controller_info_by_name, transport, "drive_status")
        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    def _process_twist_message(self, ros_msg: Twist) -> None:
        """
        Processes a Twist message and controls individual motor speeds.
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

        # This assumes the convention that the names are in the form FrontLeft, FrontRight,
        # MiddleLeft, MiddleRight, BackLeft, then BackRight
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
