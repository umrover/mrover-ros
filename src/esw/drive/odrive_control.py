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
import sys
import threading
import time as t
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict

import odrive
from odrive import find_any
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_VELOCITY_CONTROL
from odrive.utils import dump_errors
import rospy
from sensor_msgs.msg import JointState
from mrover.msg import ODriveState, WheelData
from enum import Enum


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
    :param _meters_to_turns_ratio_by_side: A dictionary that maps left or
        right to a multiplier that can be used to convert turns into meters.
    :param _odrive: An ODrive object.
    :param _turns_to_raw_ratio_by_side: A dictionary that maps left or right
        to the vel cmd multiplier that can be used to convert a raw [0, 1]
        command to desired turns.
    :param _usb_lock: A lock that prevents multiple threads from accessing
        ODrive objects simultaneously.
    :param _watchdog_timeout: A float that represents the watchdog timeout.
    """

    _axes: Dict[str, Any]
    _meters_to_turns_ratio_by_side: Dict[str, float]
    _odrive: Any
    _turns_to_raw_ratio_by_side: Dict[str, float]
    _usb_lock: threading.Lock
    _watchdog_timeout: float

    def __init__(self, odr) -> None:
        self._odrive = odr
        _side_by_axis: Dict[int, str] = {
            rospy.get_param("odrive/axis/left"): "left",
            rospy.get_param("odrive/axis/right"): "right",
        }

        self._axes = {_side_by_axis[0]: self._odrive.axis0, _side_by_axis[1]: self._odrive.axis1}
        self._meters_to_turns_ratio_by_side = {
            "left": rospy.get_param("odrive/ratio/meters_to_turns_ratio_left"),
            "right": rospy.get_param("odrive/ratio/meters_to_turns_ratio_right"),
        }
        self._turns_to_raw_ratio_by_side = {
            "left": rospy.get_param("odrive/ratio/turns_to_raw_ratio_left"),
            "right": rospy.get_param("odrive/ratio/turns_to_raw_ratio_right"),
        }
        self._usb_lock = threading.Lock()
        self._watchdog_timeout = rospy.get_param("odrive/config/watchdog_timeout")

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
            velocity = self._axes[axis].encoder.vel_estimate / self._meters_to_turns_ratio_by_side[axis]
        except Exception:
            raise DisconnectedError
        finally:
            self._usb_lock.release()
        return velocity

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

    def set_vel(self, axis: str, vel: float) -> None:
        """Sets the requested ODrive axis to run the motors at the requested
        velocity.
        :param axis: A string that represents which wheel to read current from.
        The string must be "left" or "right".
        :param vel: A float that is the requested velocity that is in range
            [-1.0, 1.0].
        :raises DisconnectedError: If Jetson is unable to communicate with
            ODrive.
        """
        assert -1.0 <= vel and vel <= 1.0, "vel must be in range [-1.0, 1.0]"
        assert axis == "left" or axis == "right", 'axis must be "left" or "right"'
        try:
            desired_input_vel_turns_s = vel * self._turns_to_raw_ratio_by_side[axis]
            assert (
                -50 <= desired_input_vel_turns_s and desired_input_vel_turns_s <= 50
            ), "magnitude of desired_input_vel_turns_sec is dangerously high"
            self._usb_lock.acquire()
            self._axes[axis].controller.input_vel = desired_input_vel_turns_s
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
    :param _modrive: A Modrive object that abstracts away the ODrive functions.
    :param _pair: A string that is front, middle, or back.
    :param _publishers: A dictionary that holds rospy Publisher object
        for odrive state data and drive joint velocity and current data.
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
    _modrive: Modrive
    _pair: str
    _publishers: Dict[str, rospy.Publisher]
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
        _odrive_ids = {
            "front": rospy.get_param("odrive/ids/front"),
            "middle": rospy.get_param("odrive/ids/middle"),
            "back": rospy.get_param("odrive/ids/back"),
        }
        self._pair = pair
        self._publishers = {
            "odrive": rospy.Publisher(f"drive_data/odrive/{pair}", ODriveState, queue_size=1),
            "left_wheel": rospy.Publisher(f"drive_data/joint/{pair}/left", WheelData, queue_size=1),
            "right_wheel": rospy.Publisher(f"drive_data/joint/{pair}/right", WheelData, queue_size=1),
        }
        self._id = _odrive_ids[pair]
        self._rate = rospy.Rate(rospy.get_param("odrive/ros/publish_rate_hz"))
        self._speed_lock = threading.Lock()
        self._state = DisconnectedState()

    def change_axis_speed(self, axis: Axis, speed: float) -> None:
        """Sets the self._speed to the requested speeds. The speeds must be in
        the range [-1.0, 1.0].
        :param axis: An Axis object that is either Axis.LEFT or Axis.RIGHT
        :param speed: A float that has the requested speed of the axis.
        """
        assert -1.0 <= speed <= 1.0, "speed must be in range[-1.0, 1.0]"
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
        """Calls the update() function continuously and checks if comms has
        been lost.
        A flag is set to keep track fo the state for when we have comms with
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
        Note that this does NOT actually change speed that the ODrive comands
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
        ros_msg = WheelData()
        try:
            ros_msg.current = self._modrive.get_measured_current(axis)
            ros_msg.velocity = self._modrive.get_velocity(axis)
            self._publishers[f"{axis}_wheel"].publish(ros_msg)
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
        self._publishers["odrive"].publish(ros_msg)

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
        the front, middle, and back wheels."""

    def __init__(self):

        rospy.init_node(f"odrive_control")
        self._bridges = [ODriveBridge("front"), ODriveBridge("middle"), ODriveBridge("back")]
        rospy.Subscriber("drive_cmd/wheels/left", JointState, self._drive_vel_cmd_callback, Axis.LEFT)
        rospy.Subscriber("drive_cmd/wheels/right", JointState, self._drive_vel_cmd_callback, Axis.RIGHT)

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

        for i in range(len(self._bridges)):
            threads_dict["publish_data_threads"][i].join()
            threads_dict["watchdog_while_threads"][i].join()

        rospy.spin()

    def _drive_vel_cmd_callback(self, ros_msg: JointState, axis: Axis) -> None:
        """Calls the change_axis_speed function.
        Note that this does NOT actually change speed that the ODrive comands
        the motors at. One must wait for the ODriveBridge._update() function
        to be called for that to happen.
        :param ros_msg: A ROS message that is JointState.
        :param axis: An Axis enum that is either Axis.LEFT or Axis.RIGHT
        """
        velocity = ros_msg.velocity[0]
        velocity = self._throttle(velocity)
        for bridge in self._bridges:
            bridge.start_time = t.process_time()
            if bridge.get_state_string() == "Armed":
                bridge.change_axis_speed(axis, velocity)

    def _throttle(self, speed: float) -> float:
        """Throttles the speed to a range of [-1.0, 1.0].
        :param speed: A float that is the input speed.
        :returns: A float that limits the input to the range [-1.0, 1.0].
        """
        return max(min(1.0, speed), -1.0)


def main():
    """Creates the Application object and runs the program."""

    app = Application()
    app.run()
    exit()


if __name__ == "__main__":
    main()
