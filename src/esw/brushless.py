#!/usr/bin/env python3
import threading
import time as t
import asyncio
import rospy
import math
from abc import ABC, abstractmethod
from geometry_msgs.msg import Twist
from typing import List, Dict
from sensor_msgs.msg import JointState
from mrover.msg import MoteusState as MoteusStateMsg, MotorsStatus
import moteus
import moteus.multiplex as mp
import io


class CommandData:
    MAX_TORQUE = 0.7
    POSITION_FOR_VELOCITY_CONTROL = math.nan
    VELOCITY_LIMIT_REV_S = 80  # DO NOT CHANGE THIS HAPHAZARDLY. DERIVED FROM TESTING.
    ZERO_VELOCITY = 0.0

    def __init__(
        self,
        position: float = POSITION_FOR_VELOCITY_CONTROL,
        velocity: float = ZERO_VELOCITY,
        torque: float = MAX_TORQUE,
    ):
        self.position = position
        self.velocity = max(-CommandData.VELOCITY_LIMIT_REV_S, min(CommandData.VELOCITY_LIMIT_REV_S, velocity))
        self.torque = max(0, min(CommandData.MAX_TORQUE, torque))


class MoteusData:
    def __init__(
        self,
        position: float = CommandData.POSITION_FOR_VELOCITY_CONTROL,
        velocity: float = CommandData.ZERO_VELOCITY,
        torque: float = CommandData.MAX_TORQUE,
    ):
        self.position = position
        self.velocity = velocity
        self.torque = torque


class MoteusState:
    DISCONNECTED_STATE = "Disconnected"
    ARMED_STATE = "Armed"
    ERROR_STATE = "Error"

    # a custom error for when the moteus is unpowered or not found
    MOTEUS_UNPOWERED_OR_NOT_FOUND_ERROR = 99

    MODE_DICTIONARY = {
        0: "stopped",
        1: "fault",
        2: "preparing to operate",
        3: "preparing to operate",
        4: "preparing to operate",
        5: "PWM mode",
        6: "voltage mode",
        7: "voltage FOC",
        8: "voltage DQ",
        9: "current",
        10: "position",
        11: "timeout",
        12: "zero velocity",
        13: "stay within",
        14: "measure inductance",
        15: "brake",
    }

    FAULT_MODE = 1
    """
    The mode number when the moteus is in a fault state.
    """

    TIMEOUT_MODE = 11
    """
    The mode number when the moteus has not received a command recently and has timed out.
    """

    ERROR_MODES_LIST = [FAULT_MODE, TIMEOUT_MODE]
    """
    The list of modes that are reset-able errors.
    """

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
        MOTEUS_UNPOWERED_OR_NOT_FOUND_ERROR: "Moteus Unpowered or Not Found",
    }

    NO_ERROR_NAME = "No Error"

    def __init__(self, state: str, error_name: str):
        self.state = state
        self.error_name = error_name


def is_mode_indicating_error(mode: int) -> bool:
    """
    Returns True if the mode response indicates an error
    :param mode_response: the value read in from the mode register of the moteus CAN message
    :return: True if the fault response is an error
    """
    return mode in MoteusState.ERROR_MODES_LIST


class MoteusBridge:
    MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S = 0.01
    ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.1

    def __init__(self, can_id: int, transport, gear_ratio: int):
        self._can_id = can_id
        self.controller = moteus.Controller(id=can_id, transport=transport)
        self.command_lock = threading.Lock()
        # self._command is the next command that this ROS node will send to the Moteus
        # once self._send_command is called.
        self._gear_ratio = gear_ratio
        self._command = CommandData()
        self.moteus_state = MoteusState(state=MoteusState.DISCONNECTED_STATE, error_name=MoteusState.NO_ERROR_NAME)
        self.moteus_data = MoteusData()

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
        # Check if our commanded velocity is close to zero
        # We can't compare floats directly due to representation so use tolerance
        if abs(command.velocity) > 1e-5:
            state = await asyncio.wait_for(
                self.controller.set_position(
                    position=command.position,
                    velocity=command.velocity,
                    velocity_limit=CommandData.VELOCITY_LIMIT_REV_S,
                    maximum_torque=command.torque,
                    watchdog_timeout=MoteusBridge.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
                    query=True,
                ),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
        else:
            # await self.controller.set_stop()
            await asyncio.wait_for(
                MoteusBridge.set_brake(self.controller),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
            state = await asyncio.wait_for(
                self.controller.query(),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )

        moteus_not_found = (
            state is None
            or not hasattr(state, "values")
            or moteus.Register.FAULT not in state.values
            or moteus.Register.MODE not in state.values
        )

        if moteus_not_found:
            self._handle_error(MoteusState.MOTEUS_UNPOWERED_OR_NOT_FOUND_ERROR, MoteusState.FAULT_MODE)
        else:
            is_error = is_mode_indicating_error(state.values[moteus.Register.MODE])

            if is_error:
                self._handle_error(state.values[moteus.Register.FAULT], state.values[moteus.Register.MODE])
            else:
                self._change_state(MoteusState.ARMED_STATE)

            self.moteus_data = MoteusData(
                position=state.values[moteus.Register.POSITION] * 2 * math.pi / self._gear_ratio,
                velocity=state.values[moteus.Register.VELOCITY] * 2 * math.pi / self._gear_ratio,
                torque=state.values[moteus.Register.TORQUE],
            )

    def _handle_error(self, fault_response: int, mode: int) -> None:
        """
        Handles error by changing the state to error state
        :param fault_response: the value read in from the fault register of the moteus CAN message.
        Or a custom error, 99.
        """
        assert is_mode_indicating_error(mode)

        self._change_state(MoteusState.ERROR_STATE)

        if mode == MoteusState.TIMEOUT_MODE:
            new_error_name = MoteusState.MODE_DICTIONARY[MoteusState.TIMEOUT_MODE]
        else:
            try:
                new_error_name = MoteusState.ERROR_CODE_DICTIONARY[fault_response]
            except KeyError:
                new_error_name = MoteusState.NO_ERROR_NAME

        if self.moteus_state.error_name != new_error_name:
            rospy.logerr(f"CAN ID {self._can_id} has encountered an error: {new_error_name}")
            self.moteus_state.error_name = new_error_name

    async def _connect(self) -> None:
        """
        Attempts to establish a connection to the moteus. State must be in error or disconnected state.
        """
        try:
            assert self.moteus_state.state != MoteusState.ARMED_STATE

            await asyncio.wait_for(
                self.controller.set_stop(), timeout=MoteusBridge.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S
            )

            command = CommandData()

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

    @staticmethod
    def make_brake(controller, *, query=False):
        """
        Temporary fix, taken from https://github.com/mjbots/moteus/blob/335d40ef2b78335be89f27fbb27c94d1a1333b25/lib/python/moteus/moteus.py#L1027
        The problem is the Python 3.7 moteus library does not have set_brake and that is the version the Pi has.
        """
        STOPPED_MODE: int = 15

        result = controller._make_command(query=query)

        data_buf = io.BytesIO()
        writer = mp.WriteFrame(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(moteus.Register.MODE))
        writer.write_int8(STOPPED_MODE)

        if query:
            data_buf.write(controller._query_data)

        result.data = data_buf.getvalue()

        return result

    @staticmethod
    async def set_brake(controller, *args, **kwargs):
        return await controller.execute(MoteusBridge.make_brake(controller, **kwargs))


class MotorsManager(ABC):
    BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S = 1

    _motor_bridges: Dict[str, MoteusBridge]
    _multipliers: Dict[str, int]
    _last_updated_time_s: float
    _motors_status_publisher: rospy.Publisher
    _lost_communication: bool
    _gear_ratios: Dict[str, int]

    def __init__(self, motor_controller_info_by_name: dict, transport):
        self._motor_bridges = {}
        self._multipliers = {}
        self._gear_ratios = {}
        for name, info in motor_controller_info_by_name.items():
            self._motor_bridges[name] = MoteusBridge(info["id"], transport, info["gear_ratio"])
            self._multipliers[name] = info["multiplier"]

        self._last_updated_time_s = t.time()

        self._motors_status_publisher = rospy.Publisher(self.publish_topic, MotorsStatus, queue_size=1)
        self._lost_communication = True

    @property
    @abstractmethod
    def manager_type(self) -> str:
        pass

    @property
    @abstractmethod
    def publish_topic(self) -> str:
        pass

    async def send_command(self) -> None:
        """
        Run one loop and only gives commands to the moteus (by updating the
        bridge) if communication is still maintained between the basestation
        and the rover node.
        """
        time_diff_since_updated = t.time() - self._last_updated_time_s
        lost_communication_now = time_diff_since_updated > MotorsManager.BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S

        for bridge in self._motor_bridges.values():
            if lost_communication_now:
                # If we only just lost communications this iteration.
                if not self._lost_communication:
                    rospy.loginfo(
                        f"Brushless {self.manager_type} Watchdog: Not receiving new messages. Disabling controls."
                    )
                    self._lost_communication = True

                # Set command to 0 rev/s
                bridge.set_command(CommandData())

            # If we just regained communications.
            elif self._lost_communication:
                self._lost_communication = False
                rospy.loginfo(f"Brushless {self.manager_type} Watchdog: Received new messages. Enabling controls.")

            await bridge.update()

        motors_status = MotorsStatus(
            name=list(self._motor_bridges.keys()),
            joint_states=JointState(
                name=list(self._motor_bridges.keys()),
                position=[bridge.moteus_data.position for bridge in self._motor_bridges.values()],
                velocity=[bridge.moteus_data.velocity for bridge in self._motor_bridges.values()],
                effort=[bridge.moteus_data.torque for bridge in self._motor_bridges.values()],
            ),
            moteus_states=MoteusStateMsg(
                name=list(self._motor_bridges.keys()),
                state=[bridge.moteus_state.state for bridge in self._motor_bridges.values()],
                error=[bridge.moteus_state.error_name for bridge in self._motor_bridges.values()],
            ),
        )

        self._motors_status_publisher.publish(motors_status)

    def update_bridge_velocity(self, motor_name: str, velocity: float, torque: float) -> None:
        """
        Updates the command of the specified motor bridge.
        """
        if self._motor_bridges[motor_name].moteus_state.state == MoteusState.ARMED_STATE:
            self._motor_bridges[motor_name].set_command(CommandData(velocity=velocity, torque=torque))


class ArmManager(MotorsManager):
    def __init__(self, arm_controller_info_by_name, transport):
        super().__init__(arm_controller_info_by_name, transport)

        self._max_rps_by_name = {}
        self._torque_limit_by_name = {}
        for name, info in arm_controller_info_by_name.items():
            self._max_rps_by_name[name] = info["max_rps"]
            self._torque_limit_by_name[name] = info["max_torque"]

        rospy.Subscriber("ra_cmd", JointState, self._process_ra_cmd)

    @property
    def manager_type(self) -> str:
        return "Arm"

    @property
    def publish_topic(self) -> str:
        return "brushless_ra_data"

    def _process_ra_cmd(self, ros_msg: JointState) -> None:
        """
        Process a JointState command controls individual motor speeds.
        :param ros_msg: Has information to control the joints controlled by both brushed and brushless motors.
        We only care about the joints controlled by brushless motors.
        """

        for i, name in enumerate(ros_msg.name):
            if name in self._motor_bridges:
                position, velocity, torque = (
                    ros_msg.position[i],
                    ros_msg.velocity[i],
                    ros_msg.effort[i],
                )

                # Ensure command is reasonable and clamp if necessary.
                if abs(velocity) > 1:
                    rospy.logerr("Commanded arm velocity is too low or high (should be [-1, 1]")
                    velocity = max(-1, min(1, velocity))

                velocity *= min(self._max_rps_by_name[name], CommandData.VELOCITY_LIMIT_REV_S)
                velocity *= self._multipliers[name]
                self.update_bridge_velocity(name, velocity, self._torque_limit_by_name[name])

        self._last_updated_time_s = t.time()


class DriveManager(MotorsManager):
    def __init__(self, drive_controller_info_by_name, transport):
        super().__init__(drive_controller_info_by_name, transport)

        rover_width = rospy.get_param("rover/width")
        rover_length = rospy.get_param("rover/length")
        self.WHEEL_DISTANCE_INNER = rover_width / 2.0
        self.WHEEL_DISTANCE_OUTER = math.sqrt(((rover_width / 2.0) ** 2) + ((rover_length / 2.0) ** 2))

        ratio_motor_to_wheel = rospy.get_param("wheel/gear_ratio")

        # To convert m/s to rev/s, multiply by this constant. Divide by circumference, multiply by gear ratio.
        self.WHEELS_M_S_TO_MOTOR_REV_S = (1 / (rospy.get_param("wheel/radius") * 2 * math.pi)) * ratio_motor_to_wheel

        _max_speed_m_s = rospy.get_param("rover/max_speed")
        assert _max_speed_m_s > 0, "rover/max_speed config must be greater than 0"

        self._max_motor_speed_rev_s = _max_speed_m_s * self.WHEELS_M_S_TO_MOTOR_REV_S
        self._max_torque = rospy.get_param("brushless/drive/max_torque")
        self._last_updated_time_s = t.time()

        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    @property
    def manager_type(self) -> str:
        return "Drive"

    @property
    def publish_topic(self) -> str:
        return "drive_status"

    def _process_twist_message(self, ros_msg: Twist) -> None:
        """
        Processes a Twist message and controls individual motor speeds.
        :param ros_msg: Has linear x and angular z velocity components.
        Linear velocity is assumed to be in meters per second.
        Angular velocity is assumed to be in radians per second.
        """

        forward = ros_msg.linear.x
        turn = ros_msg.angular.z

        # Multiply radians per second by the radius and to find arc length (s = r(theta)).
        turn_difference_inner = turn * self.WHEEL_DISTANCE_INNER
        turn_difference_outer = turn * self.WHEEL_DISTANCE_OUTER

        # Scale to get rev / s.
        left_rev_inner = (forward - turn_difference_inner) * self.WHEELS_M_S_TO_MOTOR_REV_S
        right_rev_inner = (forward + turn_difference_inner) * self.WHEELS_M_S_TO_MOTOR_REV_S
        left_rev_outer = (forward - turn_difference_outer) * self.WHEELS_M_S_TO_MOTOR_REV_S
        right_rev_outer = (forward + turn_difference_outer) * self.WHEELS_M_S_TO_MOTOR_REV_S

        # If speed too fast, scale to max speed. Ignore inner for comparison since outer > inner, always.
        larger_abs_rev_s = max(abs(left_rev_outer), abs(right_rev_outer))
        if larger_abs_rev_s > self._max_motor_speed_rev_s:
            change_ratio = self._max_motor_speed_rev_s / larger_abs_rev_s
            left_rev_inner *= change_ratio
            right_rev_inner *= change_ratio
            left_rev_outer *= change_ratio
            right_rev_outer *= change_ratio

        drive_command_velocities = {
            "FrontLeft": left_rev_outer,
            "FrontRight": right_rev_outer,
            "MiddleLeft": left_rev_inner,
            "MiddleRight": right_rev_inner,
            "BackLeft": left_rev_outer,
            "BackRight": right_rev_outer,
        }

        # Update bridges.
        for name in self._motor_bridges.keys():
            velocity = drive_command_velocities[name] * self._multipliers[name]
            self.update_bridge_velocity(name, velocity, self._max_torque)

        self._last_updated_time_s = t.time()


class Application:
    def __init__(self):
        rospy.init_node("brushless")

        arm_controller_info_by_name = rospy.get_param("brushless/arm/controllers")
        drive_controller_info_by_name = rospy.get_param("brushless/drive/controllers")

        using_pi3_hat = rospy.get_param("brushless/using_pi3_hat")

        if using_pi3_hat:
            import moteus_pi3hat

            transport = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map={
                    5: [info["id"] for info in drive_controller_info_by_name.values() if info["bus"] == 5],
                    2: [info["id"] for info in drive_controller_info_by_name.values() if info["bus"] == 2],
                    3: [info["id"] for info in arm_controller_info_by_name.values() if info["bus"] == 3],
                }
            )
        else:
            transport = None

        self._arm_manager = ArmManager(arm_controller_info_by_name, transport)
        self._drive_manager = DriveManager(drive_controller_info_by_name, transport)

    def run(self) -> None:
        """
        Allows the function to run an async function.
        """
        asyncio.run(self.run_tasks())

    async def run_tasks(self) -> None:
        """
        Creates an async function to run both drive and arm tasks.
        """
        while not rospy.is_shutdown():
            await self._arm_manager.send_command()
            await self._drive_manager.send_command()


def main():
    app = Application()
    app.run()


if __name__ == "__main__":
    main()
