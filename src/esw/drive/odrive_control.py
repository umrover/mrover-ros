#!/usr/bin/env python3

""" Controls an ODrive motor controller to drive the wheels.
The ODrives codebase deals with controlling the ODrive motor controllers to
drive the wheels. For the 2022 Mars Rover Rosie, there are 3 ODrive motor
controllers to drive the wheels. Each ODrive connects to the Jetson via USB.
Each ODrive has two axes and each axis is assigned a wheel. To simplify wiring
and code, the axis of each ODrive are either all left or all right. Also, the
wheels that are connected to one singular ODrive are opposite each other (i.e.
an ODrive is assigned either the front, middle, or back wheels).
"""

import math
import threading
import time as t
from enum import Enum
from typing import Any, Dict, List

import odrive
from odrive import find_any
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_VELOCITY_CONTROL
from odrive.utils import dump_errors
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from mrover.msg import ODriveState
from enum import Enum


global_publish_joint_state_data_publisher = rospy.Publisher(f"drive_data", JointState, queue_size=1)
global_publish_joint_state_data = JointState()
global_publish_joint_state_data.name = ["FrontLeft", "FrontRight", "MiddleLeft", "MiddleRight", "BackLeft", "BackRight"]
global_publish_joint_state_data.position = [0, 0, 0, 0, 0, 0]
global_publish_joint_state_data.velocity = [0, 0, 0, 0, 0, 0]
global_publish_joint_state_data.effort = [0, 0, 0, 0, 0, 0]


class Axis(Enum):
    LEFT = 0
    RIGHT = 1


class Speed:
    """Stores speed info.
    :var left: A float that is the speed of the left wheel.
    :var right: A float that is the speed of the right wheel
    """

    left: float
    right: float

    def __init__(self, left: float = 0.0, right: float = 0.0) -> None:
        self.left = left
        self.right = right

    def __eq__(self, other):
        """Overrides the equality comparator operator."""
        return self.left == other.left and self.right == other.right


class DisconnectedError(Exception):
    """An exception thrown when the Jetson is unable to talk with the ODrive."""

    pass


class ODriveEvent(Enum):
    """Dictates the behavior of the changing of States.
    The ODriveEvents reveal the following: When the ODrive is disconnected,
    is trying to arm, or has encountered an error.
    """

    DISCONNECTED_ODRIVE_EVENT = 0
    ARM_CMD_EVENT = 1
    ODRIVE_ERROR_EVENT = 2


class Modrive:
    """Abstracts the behavior of the ODrive.
    The ODriveBridge creates the same Modrive object
    every time it tries to connect to an ODrive. The Modrive object
    has functions that allows it to get encoder data, get state data,
    set velocity, and much more.
    :param _axes: A dictionary that maps left or right to an ODrive axes
        object.
    :param _direction_by_side: A dictionary that maps direction of motor
        to direction of actual wheel.
    :param _odrive: An ODrive object.
    :param _radius: The radius of the wheels.
    :param _usb_lock: A lock that prevents multiple threads from accessing
        ODrive objects simultaneously.
    :param _watchdog_timeout: A float that represents the watchdog timeout.
    :param _motor_turns_to_wheels_m_s_ratio: Multipler to convert from motor turns to wheel m/s

    """

    _axes: Dict[str, Any]
    _direction_by_side: Dict[str, float]
    _odrive: Any
    _radius: float
    _usb_lock: threading.Lock
    _watchdog_timeout: float
    _motor_turns_to_wheels_m_s_ratio: float

    def __init__(self, odr) -> None:
        self._odrive = odr
        _side_by_axis: Dict[int, str] = {
            rospy.get_param("odrive/axis/left"): "left",
            rospy.get_param("odrive/axis/right"): "right",
        }

        self._axes = {_side_by_axis[0]: self._odrive.axis0, _side_by_axis[1]: self._odrive.axis1}
        self._direction_by_side = {
            "left": rospy.get_param("odrive/ratio/direction_left"),
            "right": rospy.get_param("odrive/ratio/direction_right"),
        }
        self._radius = rospy.get_param("wheel/radius")
        self._usb_lock = threading.Lock()
        self._watchdog_timeout = rospy.get_param("odrive/config/watchdog_timeout")

        wheel_radius = rospy.get_param("wheel/radius")
        _ratio_motor_to_wheel = rospy.get_param("wheel/gear_ratio")
        self._motor_turns_to_wheels_m_s_ratio = 2 * math.pi / _ratio_motor_to_wheel * wheel_radius

    def __getattr__(self, attr: Any) -> Any:
        """
        Used as a failsafe in case object does not exist.
        """
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self._odrive, attr)

    def arm(self) -> None:
        """Arms the ODrive by setting it to closed loop control and
        velocity control.
        """
        self._set_requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        self._set_velocity_ctrl()

    def disarm(self) -> None:
        """Disarms the ODrive by setting the velocity to zero and making
        it idle.
        """
        self._set_requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        self._set_velocity_ctrl()
        self.set_vel("left", 0.0)
        self.set_vel("right", 0.0)
        self._set_requested_state(AXIS_STATE_IDLE)

    def dump_errors(self) -> None:
        """Dump errors and prints them out."""
        self._reset_errors()

    def get_measured_current(self, axis: str) -> float:
        """Returns the measured current of the requested axis of the ODrive in
        Amperes.
        :param axis: A string that represents which wheel to read current from.
            The string must be "left" or "right"
        :returns: A float that is the measured current of the corresponding
            ODrive axis in Amperes.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        assert axis == "left" or axis == "right", 'axis must be "left" or "right"'
        try:
            self._usb_lock.acquire()
            measured_current = self._axes[axis].motor.current_control.Iq_measured
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()
        return measured_current

    def get_velocity(self, axis: str) -> float:
        """Returns the estimated velocity of the requested wheel of the ODrive
        in meters per second.
        The raw encoder.vel_estimate returns the estimate in turns per second,
        which must then be converted into meters per second using the map.
        :param axis: A string that represents which wheel to read current from.
            The string must be "left" or "right"
        :returns: A float that is the measured current of the corresponding
            ODrive axis in Amperes.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        assert axis == "left" or axis == "right", 'axis must be "left" or "right"'
        try:
            self._usb_lock.acquire()
            vel_turns = self._axes[axis].encoder.vel_estimate
            vel_wheel_m_s = vel_turns * self._direction_by_side[axis] * self._motor_turns_to_wheels_m_s_ratio
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()
        return vel_wheel_m_s

    def has_errors(self) -> bool:
        """Returns a boolean to show if there are ODrive errors.
        :returns: A boolean that is True if there are errors.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        try:
            self._usb_lock.acquire()
            errors = self._axes["left"].error + self._axes["right"].error != 0
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()
        return errors

    def reset_watchdog(self) -> None:
        """Resets the watchdog of the ODrives.
        This function is usually called if the ODrive has previously errored
        out or was disconnected.
        """
        self._disable_watchdog()
        self._reset_errors()
        self._enable_watchdog()

    def set_current_lim(self, lim: float) -> None:
        """Sets the current limit of each ODrive axis.
        :param lim: A float that is the requested current limit.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.motor.config.current_lim = lim
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def set_vel(self, axis: str, motor_rad_s: float) -> None:
        """Sets the requested ODrive axis to run the motors at the requested
        velocity.
        :param axis: A string that represents which wheel to read current from.
        The string must be "left" or "right".
        :param vel_rad_s: A float that is the requested velocity that is in rad/s.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        assert axis == "left" or axis == "right", 'axis must be "left" or "right"'
        try:
            motor_turns_s = motor_rad_s / (2 * math.pi)
            motor_turns_s *= self._direction_by_side[axis]
            self._usb_lock.acquire()
            self._axes[axis].controller.input_vel = motor_turns_s
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def watchdog_feed(self) -> None:
        """Refreshes the ODrive watchdog feed."""
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.watchdog_feed()
        except Exception:
            rospy.logerr("Failed in watchdog_feed. Unplugged")
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def _disable_watchdog(self) -> None:
        """Disables the ODrive watchdog.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.config.watchdog_timeout = 0
                axis.config.enable_watchdog = False
        except Exception:
            rospy.logerr("Failed in _disable_watchdog. Unplugged")
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def _enable_watchdog(self) -> None:
        """Enables the ODrive watchdog.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.config.watchdog_timeout = self._watchdog_timeout
                axis.watchdog_feed()
                axis.config.enable_watchdog = True
        except Exception:
            rospy.logerr("Failed in _enable_watchdog. Unplugged")
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def _reset_errors(self) -> None:
        """Resets the errors cleanly for all axes."""
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.error = 0
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def _set_control_mode(self, mode: Any) -> None:
        """Sets the ODrive control mode to the requested control mode.
        :param mode: A control mode that is the requested ODrive control mode.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.controller.config.control_mode = mode
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def _set_requested_state(self, state: Any) -> None:
        """Sets the ODrive state to the requested state.
        :param state: A state that is the requested ODrive state.
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.requested_state = state
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()

    def _set_velocity_ctrl(self) -> None:
        """Sets the ODrive to velocity control."""
        self._set_control_mode(CONTROL_MODE_VELOCITY_CONTROL)


class State(object):
    """Provides some utility functions for the individual states within the
    state machine.
    """

    def __init__(self) -> None:
        return

    def on_event(self, event: ODriveEvent, modrive: Modrive):
        """Handles events that are delegated to this State."""

    def __repr__(self) -> str:
        """Makes it so __str__ method can describe the State."""
        return self.__str__()

    def __str__(self) -> str:
        """Returns the name of the State."""
        return self.__class__.__name__


class ArmedState(State):
    """Reveals that the ODrive is armed."""

    def on_event(self, event: ODriveEvent, modrive: Modrive) -> State:
        """Handles events that are delegated to the Armed State."""
        if str(event) == str(ODriveEvent.DISCONNECTED_ODRIVE_EVENT):
            return DisconnectedState()
        elif str(event) == str(ODriveEvent.ODRIVE_ERROR_EVENT):
            return ErrorState()
        return self


class DisconnectedState(State):
    """Reveals that the ODrive has disconnected."""

    def on_event(self, event: ODriveEvent, modrive: Modrive) -> State:
        """Handles events that are delegated to the Disconnected State."""
        if str(event) == str(ODriveEvent.ARM_CMD_EVENT):
            modrive.disarm()
            modrive.reset_watchdog()
            modrive.arm()
            return ArmedState()
        return self


class ErrorState(State):
    """Reveals that the ODrive is receiving errors."""

    def on_event(self, event: ODriveEvent, modrive: Modrive) -> State:
        """Handles events that are delegated to the Error State."""
        modrive.dump_errors()
        if str(event) == str(ODriveEvent.ODRIVE_ERROR_EVENT):
            return DisconnectedState()
        return self


class ODriveBridge(object):
    """Controls the behavior of one ODrive. It manages the ODrive state and
    various other behavior.
    :param start_time: An object that helps keep track of time for the
        watchdog.
    :param _current_lim: A float that is the current limit in Amperes.
    :param _id: A string that is the current ODrive's ID.
    :param _data_publish_idx_by_pair: A dictionary that maps a pair to its publish index id
    :param _modrive: A Modrive object that abstracts away the ODrive functions.
    :param _pair: A string that is front, middle, or back.
    :param _odrive_publisher: Holds the rospy Publisher object for odrive state data
    :param _rate: A Rate object that is used for sleep to make sure publish
        does not spam.
    :param _speed: A Speed object that has requested left and right wheel
        speed.
    :param _speed_lock: A lock that prevents multiple threads from accessing
        self._speed simultaneously.
    :param _state: A State object that keeps track of the current state.
    """

    start_time: float
    _current_lim: float
    _id: str
    _data_publish_idx_by_pair: Dict[str, int]
    _modrive: Modrive
    _pair: str
    _odrive_publisher: rospy.Publisher
    _rate: rospy.Rate
    _speed: Speed
    _speed_lock: threading.Lock
    _state: State

    def __init__(self, pair: str) -> None:
        """Initializes the components, starting with a Disconnected State.
        :param pair: A string that is front, middle, or back"""
        self.start_time = t.process_time()
        self._current_lim = rospy.get_param("odrive/config/current_lim")
        self._speed = Speed()
        self._pair = pair
        self._odrive_publisher = rospy.Publisher(f"drive_data/odrive/{pair}", ODriveState, queue_size=1)
        self._id = rospy.get_param(f"odrive/ids/{pair}")
        self._data_publish_idx_by_pair = {
            "left": rospy.get_param(f"odrive/publish_index/{pair}_left"),
            "right": rospy.get_param(f"odrive/publish_index/{pair}_right"),
        }
        self._rate = rospy.Rate(rospy.get_param("odrive/ros/publish_rate_hz"))
        self._speed_lock = threading.Lock()
        self._state = DisconnectedState()

    def change_axis_speed(self, axis: Axis, speed: float) -> None:
        """Sets the self._speed to the requested speeds. The speeds must be in
        rad/s.
        :param axis: An Axis object that is either Axis.LEFT or Axis.RIGHT
        :param speed: A float that has the requested speed of the axis in m/s.
        """
        self._speed_lock.acquire()
        if axis == Axis.LEFT:
            self._speed = Speed(speed, self._speed.right)
        else:
            self._speed = Speed(self._speed.left, speed)
        self._speed_lock.release()

    def get_state_string(self) -> str:
        """Returns the state of the ODriveBridge as a short string.
        The form is of "[insert_odrive_state]State". e.g. state is ErrorState,
        so ros_msg.state is Error
        :returns: A string that is the state of the ODriveBridge.
        """
        state = str(self._state)
        return state[: len(state) - len("State")]

    def ros_publish_data_loop(self) -> None:
        """Publishes velocity and current data continuously."""
        while not rospy.is_shutdown():
            self._publish_encoder_msg()
            self._rate.sleep()

    def watchdog_while_loop(self) -> None:
        """Calls the update() function continuously and checks if communication has
        been lost.
        A flag is set to keep track fo the state for when we have communication with
        the base station or not.
        """
        previously_lost_comms = True
        while not rospy.is_shutdown():
            watchdog = t.process_time() - self.start_time
            lost_comms = watchdog > 1.0
            if lost_comms:
                if not previously_lost_comms:
                    rospy.loginfo("Lost comms")
                    previously_lost_comms = True
                self.change_axis_speed(Axis.LEFT, 0.0)
                self.change_axis_speed(Axis.RIGHT, 0.0)
            elif previously_lost_comms:
                previously_lost_comms = False
                rospy.loginfo("Regained comms")
            try:
                self._update()
            except DisconnectedError:
                self._bridge_on_event(ODriveEvent.DISCONNECTED_ODRIVE_EVENT)

    def _bridge_on_event(self, event: ODriveEvent) -> None:
        """Delegates incoming events to the given states which then handle
        the event. The result is then assigned as the new state.
        Note that this does NOT actually change speed that the ODrive commands
        the motors at. One must wait for the ODriveBridge._update() function
        to be called for that to happen.
        :param event: An ODriveEvent that determines the behavior of the
            switching of states.
        """
        try:
            self._state = self._state.on_event(event, self._modrive)
            rospy.loginfo("Current state for %s ODrive is now: %s", self._pair, self.get_state_string())
            self._publish_state_msg(self.get_state_string())
        except DisconnectedError:
            return

    def _connect(self) -> None:
        """Connects to an ODrive if one is connected.
        This uses the ODrive library to look for an ODrive by its ID.
        """
        odrive_id = self._id
        rospy.loginfo("Looking for %s ODrive with ID %s", self._pair, odrive_id)
        odrive_object = odrive.find_any(serial_number=odrive_id)
        rospy.loginfo("Found %s ODrive with ID %s", self._pair, odrive_id)
        self._modrive = Modrive(odrive_object)
        self._modrive.set_current_lim(self._current_lim)

    def _publish_encoder_helper(self, axis: str) -> None:
        """Publishes the velocity and current data of the requested axis to a
        ROS topic. Published in m/s and Amps.
        :param axis: A string that represents which wheel to read current from.
            The string must be "left" or "right"
        """
        assert axis == "left" or axis == "right", 'axis must be "left" or "right"'
        try:
            publish_index = self._data_publish_idx_by_pair[axis]

            global_publish_joint_state_data.velocity[publish_index] = self._modrive.get_velocity(axis)
            global_publish_joint_state_data.effort[publish_index] = self._modrive.get_measured_current(axis)

            global_publish_joint_state_data_publisher.publish(global_publish_joint_state_data)
        except DisconnectedError:
            return
        except AttributeError:
            # since self._modrive may be None if not connected yet
            return

    def _publish_encoder_msg(self) -> None:
        """Publishes the velocity and current data of all the axes to a ROS
        topic. Published in m/s and Amps.
        """
        self._publish_encoder_helper("left")
        self._publish_encoder_helper("right")

    def _publish_state_msg(self, state: str) -> None:
        """Publishes the ODrive state message over ROS to a topic.
        :param state: A string that is the current state.
        """
        ros_msg = ODriveState()
        ros_msg.state = state
        self._odrive_publisher.publish(ros_msg)

    def _update(self) -> None:
        """Updates based on the current state.
        In the armed state, it will first check for ODrive errors. Then, it
        will _update the speed. In the disconnected state, it will create an
        arm event. In the error state, it will create an error event.
        """
        if str(self._state) == "ArmedState":
            try:
                errors = self._modrive.has_errors()
                self._modrive.watchdog_feed()
            except DisconnectedError:
                self._bridge_on_event(ODriveEvent.DISCONNECTED_ODRIVE_EVENT)
                return

            if errors:
                self._bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)
                return

            self._speed_lock.acquire()
            left_speed = self._speed.left
            right_speed = self._speed.right
            self._speed_lock.release()
            self._modrive.set_vel("left", left_speed)
            self._modrive.set_vel("right", right_speed)

        elif str(self._state) == "DisconnectedState":
            self._connect()
            self._bridge_on_event(ODriveEvent.ARM_CMD_EVENT)

        elif str(self._state) == "ErrorState":
            self._bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)


class Application(object):
    """Manages the ODriveBridge objects and keeps track of communication.
    :param _bridges: A list of bridges that are contain data for the ODrives controlling
        the front, middle, and back wheels.
    :param _max_motor_speed_rad_s: Max permitted speed of the motor in rad/s.
    :param _wheel_distance_inner: Distance of inner wheels (middle) from center in meters
    :param _wheel_distance_outer: Distance of outer wheels (front/back) from center in meters
    :param _wheels_m_s_to_motor_rad_ratio: Multipler to convert from wheel m/s to motor rad"""

    _bridges: List[ODriveBridge]
    _max_motor_speed_rad_s: float
    _wheel_distance_inner: float
    _wheel_distance_outer: float
    _wheels_m_s_to_motor_rad_ratio: float

    def __init__(self):

        rospy.init_node("odrive_control")
        self._bridges = [ODriveBridge("front"), ODriveBridge("middle"), ODriveBridge("back")]

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
        measured_max_motor_rad_s = 50 * 2 * math.pi  # Should not be changed. Derived from testing.
        self._max_motor_speed_rad_s = min(measured_max_motor_rad_s, max_motor_rad_s)

        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

    def run(self):
        """Runs the publish data loop and watchdog loops for all bridges"""

        threads_dict = {}
        threads_dict["publish_data_threads"] = [threading.Thread()] * len(self._bridges)
        threads_dict["watchdog_while_threads"] = [threading.Thread()] * len(self._bridges)
        for i, bridge in enumerate(self._bridges):
            threads_dict["publish_data_threads"][i] = threading.Thread(target=bridge.ros_publish_data_loop)
            threads_dict["watchdog_while_threads"][i] = threading.Thread(target=bridge.watchdog_while_loop)

        for i in range(len(self._bridges)):
            threads_dict["publish_data_threads"][i].start()
            threads_dict["watchdog_while_threads"][i].start()

        rospy.spin()

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

        for bridge in self._bridges:
            bridge.start_time = t.process_time()

        # Handle inner wheels
        if self._bridges[1].get_state_string() == "Armed":
            self._bridges[1].change_axis_speed(Axis.LEFT, left_rad_inner)
            self._bridges[1].change_axis_speed(Axis.RIGHT, right_rad_inner)

        # Handle outer wheels
        if self._bridges[0].get_state_string() == "Armed":
            self._bridges[0].change_axis_speed(Axis.LEFT, left_rad_outer)
            self._bridges[0].change_axis_speed(Axis.RIGHT, right_rad_outer)

        if self._bridges[2].get_state_string() == "Armed":
            self._bridges[2].change_axis_speed(Axis.LEFT, left_rad_outer)
            self._bridges[2].change_axis_speed(Axis.RIGHT, right_rad_outer)


def main():
    """Creates the Application object and runs the program."""

    app = Application()
    app.run()
    exit()


if __name__ == "__main__":
    main()
