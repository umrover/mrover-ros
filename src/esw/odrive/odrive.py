#!/usr/bin/env python3

"""This code controls one ODrive.
It takes in a command line argument (0, 1, or 2) to see which
ODrive it is controlling. The numbers determine whether it is front, middle,
or back, as indicated in the config/odrive.yaml file.
This means that to control 3 separate ODrives,
3 of these programs must be running simultaneously.
Configuration variables can be changed in the config/odrive.yaml file
and are initialized by the program in the config/odrive.py file.
The ODriveBridge object controls
the behavior of the ODrive.
The ODriveBridge object keeps track of a state that it is in.
A State may change to a different state depending on an event.
The ODrive may either be in an error, disconnected, or armed state.
The Modrive object controls the ODrive itself.
"""
import sys
import threading
import time as t
from enum import Enum
from typing import Any

import fibre
import odrive as odv
import rospy
from mrover.msg import DriveStateData, DriveVelCmd, DriveVelData
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                          CONTROL_MODE_VELOCITY_CONTROL)
from odrive.utils import dump_errors


class DisconnectedError(Exception):
    pass


class ODriveEvent(Enum):
    """These are the the possible ODrive events.
    The ODriveBridge keeps track of a State. This
    State will change depending on the current ODrive event."""
    DISCONNECTED_ODRIVE_EVENT = 0
    ARM_CMD_EVENT = 1
    ODRIVE_ERROR_EVENT = 2


class Modrive:
    """This has the functions that are used to control
    and command the ODrive."""
    _axes: Any
    _axis_vel_cmd_mult_map: dict[str, float]
    _axis_vel_est_mult_map: dict[str, float]
    _odrive: Any
    _usb_lock = threading.Lock
    _watchdog_timeout: float

    def __init__(self, odr, axis_0_str: str, axis_1_str: str):

        self._odrive = odr
        axis_map: dict[int, str] = {
            rospy.get_param("/odrive/axis/left"): 'left',
            rospy.get_param("/odrive/axis/right"): 'right'}

        self._axes = {
            axis_map[0]: self._odrive.axis0,
            axis_map[1]: self._odrive.axis1}

        # This scales [0, 1] to [0, vel_cmd_mult] turns per second
        self._axis_vel_cmd_mult_map = {
            'left': rospy.get_param(
                "/odrive/multiplier/vel_cmd_multiplier_left"),
            'right': rospy.get_param(
                "/odrive/multiplier/vel_cmd_multiplier_right")}

        # This scales turns/sec to m/sec
        self._axis_vel_est_mult_map = {
            'left': rospy.get_param(
                "/odrive/multiplier/vel_est_multiplier_left"),
            'right': rospy.get_param(
                "/odrive/multiplier/vel_est_multiplier_right")}
        self._usb_lock = threading.Lock()
        self._watchdog_timeout = rospy.get_param(
            "/odrive/config/watchdog_timeout")

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self._odrive, attr)

    def arm(self) -> None:
        """Arms the ODrive"""
        self._set_closed_loop_ctrl()
        self._set_velocity_ctrl()

    def check_errors(self) -> bool:
        """Returns value of sum of errors"""
        try:
            self._usb_lock.acquire()
            errors = self._axes['left'].error + self._axes['right'].error != 0
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError
        return errors

    def disarm(self) -> None:
        """Disarms the ODrive and sets the velocity to 0"""
        self._set_closed_loop_ctrl()
        self._set_velocity_ctrl()
        self.set_vel('left', 0.0)
        self.set_vel('right', 0.0)
        self._idle()

    def get_measured_current(self, axis: str) -> float:
        """Returns the measured current of
        the requested axis of the ODrive"""
        # measured current [Amps]
        try:
            self._usb_lock.acquire()
            measured_current = \
                self._axes[axis].motor.current_control.Iq_measured
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError
        return measured_current

    def get_vel_estimate_m_s(self, axis: str) -> float:
        """Returns the estimated velocity of
        the requested axis of the ODrive"""
        try:
            self._usb_lock.acquire()
            vel_est_m_s = (
                self._axes[axis].encoder.vel_estimate
                * self._axis_vel_est_mult_map[axis]
            )
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError
        return vel_est_m_s

    def reset_watchdog(self) -> None:
        """This resets the watchdog of the ODrives.
        This is done in case there was previously an error
        caused by the watchdog."""
        try:
            print("Resetting watchdog")
            self._disable_watchdog()
            # clears errors cleanly
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.error = 0
            self._usb_lock.release()
            self._enable_watchdog()
        except fibre.protocol.ChannelBrokenException:
            self._release_if_locked()
            print("Failed in _disable_watchdog. Unplugged")
        except Exception:
            raise DisconnectedError

    def set_current_lim(self, lim: float):
        """Sets the current limit of both ODrive axes"""
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.motor.config.current_lim = lim
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError

    def set_vel(self, axis: str, vel: float) -> None:
        """Sets the requested ODrive axis to run the
        motors at the requested velocity"""
        try:
            desired_input_vel_turns_s = vel * self._axis_vel_cmd_mult_map[axis]
            self._usb_lock.acquire()
            self._axes[axis].controller.input_vel = desired_input_vel_turns_s
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError

    def watchdog_feed(self) -> None:
        """Refreshes the watchdog feed."""
        try:
            for axis in self._axes.values():
                axis.watchdog_feed()
        except fibre.protocol.ChannelBrokenException:
            self._release_if_locked()
            print("Failed in watchdog_feed. Unplugged")

    def _enable_watchdog(self) -> None:
        """This enables the watchdog of the ODrives."""
        try:
            print("Enabling watchdog")
            for axis in self._axes.values():
                self._usb_lock.acquire()
                axis.config.watchdog_timeout = self._watchdog_timeout
                self._usb_lock.release()
                self.watchdog_feed()
                self._usb_lock.acquire()
                axis.config.enable_watchdog = True
                self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError

    def _disable_watchdog(self) -> None:
        """This disables the watchdog of the ODrives"""
        try:
            print("Disabling watchdog")
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.config.watchdog_timeout = 0
                axis.config.enable_watchdog = False
            self._usb_lock.release()
        except fibre.protocol.ChannelBrokenException:
            self._release_if_locked()
            print("Failed in _disable_watchdog. Unplugged")
        except Exception:
            self._release_if_locked()
            raise DisconnectedError

    def _idle(self) -> None:
        """Sets the ODrive state to idle"""
        self._set_requested_state(AXIS_STATE_IDLE)

    def _release_if_locked(self) -> None:
        "Release USB if locked"
        if self._usb_lock.locked():
            self._usb_lock.release()

    def _set_closed_loop_ctrl(self) -> None:
        """Sets the ODrive state to closed loop control"""
        self._set_requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def _set_control_mode(self, mode) -> None:
        """Sets the control mode of the ODrive"""
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.controller.config.control_mode = mode
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError

    def _set_requested_state(self, state) -> None:
        """Sets each ODrive axis state to the requested state"""
        try:
            self._usb_lock.acquire()
            for axis in self._axes.values():
                axis.requested_state = state
            self._usb_lock.release()
        except Exception:
            self._release_if_locked()
            raise DisconnectedError

    def _set_velocity_ctrl(self) -> None:
        """Sets the ODrive to velocity control"""
        self._set_control_mode(CONTROL_MODE_VELOCITY_CONTROL)


class State(object):
    """
    State object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print('Processing current state:', str(self))

    def on_event(self, event: ODriveEvent, modrive: Modrive) -> None:
        """
        Handle events that are delegated to this State.
        """

    def __repr__(self):
        """
        Make it so __str__ method can describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        State state
        str(state) = State
        """
        return self.__class__.__name__


class ArmedState(State):
    """This is the State for when the ODrive is armed"""
    def on_event(self, event: ODriveEvent, modrive: Modrive) -> None:
        """
        Handle events that are delegated to the Armed State.
        """
        if event == ODriveEvent.DISCONNECTED_ODRIVE_EVENT:
            return DisconnectedState()
        elif event == ODriveEvent.ODRIVE_ERROR_EVENT:
            return ErrorState()
        return self


class DisconnectedState(State):
    """This is the State for when the ODrive has disconnected"""
    def on_event(self, event: ODriveEvent, modrive: Modrive) -> None:
        """
        Handle events that are delegated to the Disconnected State.
        """
        if event == ODriveEvent.ARM_CMD_EVENT:
            modrive.disarm()
            modrive.reset_watchdog()
            modrive.arm()
            return ArmedState()
        return self


class ErrorState(State):
    """This is the State for when the ODrive is receiving errors"""
    def on_event(self, event: ODriveEvent, modrive: Modrive) -> None:
        """
        Handle events that are delegated to the Error State.
        """
        print(dump_errors(modrive.odrive, True))
        if event == ODriveEvent.ODRIVE_ERROR_EVENT:
            return DisconnectedState()
        return self


class ODriveBridge(object):
    """This object controls the behavior
    of one ODrive. It manages the ODrive
    state and various other behavior."""
    _current_lim: float
    _left_speed: float
    _modrive: Modrive
    _odrive_ids: dict[str, str]
    _pair: str
    _right_speed: float
    _speed_lock: threading.Lock
    _start_time: t.clock
    _state: State
    _state_pub: rospy.Publisher
    _vel_pub: rospy.Publisher

    def __init__(self) -> None:
        """
        Initialize the components.
        Start with a Default State
        """
        self._current_lim = rospy.get_param("/odrive/config/current_lim")
        self._left_speed = 0.0
        self._modrive = None
        self._odrive_ids = {
            'front': rospy.get_param("/odrive/ids/front"),
            'middle': rospy.get_param("/odrive/ids/middle"),
            'back': rospy.get_param("/odrive/ids/back")}
        self._right_speed = 0.0
        self._pair = sys.argv[1]
        self._speed_lock = threading.Lock()
        self._start_time = t.clock()
        self._state = DisconnectedState()
        self._state_pub = rospy.Publisher(
            'drive_state_data', DriveStateData, queue_size=1)
        self._vel_pub = rospy.Publisher(
            'drive_vel_data', DriveVelData, queue_size=1)

    def drive_vel_cmd_callback(self, ros_msg: DriveVelCmd) -> None:
        """Set the global speed to the requested speed in the ROS message.
        Note that this does NOT actually change speed that the ODrive comands
        the motors at. One must wait for the ODriveBridge._update() function
        to be called for that to happen."""
        try:
            if self._get_state_string() == "Armed":
                self._change_left_and_right_speed(ros_msg.left, ros_msg.right)
        except DisconnectedError:
            return

    def ros_publish_data_loop(self) -> None:
        """This loop continuously publishes
        velocity and current data."""
        while not rospy.is_shutdown():
            self._start_time = t.clock()
            self._publish_encoder_msg()

    def watchdog_while_loop(self):
        """Watchdog while loop"""
        # flag for state when we have comms with base_station vs not
        previously_lost_comms = True
        while not rospy.is_shutdown():
            watchdog = t.clock() - self._start_time
            lost_comms = watchdog > 1.0
            if lost_comms:
                if not previously_lost_comms:
                    # done to print "loss of comms" once
                    print("loss of comms")
                    previously_lost_comms = True

                self._change_left_and_right_speed(0.0, 0.0)
            elif previously_lost_comms:
                previously_lost_comms = False
                print("regained comms")

            try:
                self._update()
            except DisconnectedError:
                self._bridge_on_event(
                    ODriveEvent.DISCONNECTED_ODRIVE_EVENT
                )

    def _bridge_on_event(self, event: ODriveEvent) -> None:
        """
        Incoming events are
        delegated to the given states which then handle the event.
        The result is then assigned as the new state.
        """
        try:
            print("on event called, ODrive event:", event)
            self._state = self._state.on_event(event, self._modrive)
            self._publish_state_msg(self._get_state_string())
        except Exception:
            pass

    def _change_left_and_right_speed(self, left: float, right: float) -> None:
        self._speed_lock.acquire()
        self._left_speed = left
        self._right_speed = right
        self._speed_lock.release()

    def _connect(self) -> None:
        """This will attempt to connect to an ODrive.
        This will use the ODrive library to look for an
        ODrive with the specified ID on the Jetson."""
        print("looking for ODrive")

        odrive_id = self._odrive_ids[self._pair]

        print(odrive_id)
        odrive = odv.find_any(serial_number=odrive_id)

        print("found odrive")
        self._modrive = Modrive(
            odrive)
        self._modrive.set_current_lim(self._current_lim)

    def _get_state_string(self) -> str:
        """Returns the state of the ODriveBridge
        as a short string which is of
        the form "[insert_odrive_state]State"
        e.g. state is ErrorState,
        so ros_msg.state is Error"""
        state = str(self._state)
        return state[:len(state) - len("State")]

    def _publish_encoder_helper(self, axis: str) -> None:
        """Publishes the velocity and current
        data message over ROS of the requested axis"""
        ros_msg = DriveVelData()
        try:
            ros_msg.current_amps = self._modrive.get_measured_current(axis)
            ros_msg.vel_m_s = self._modrive.get_vel_estimate_m_s(axis)
            # e.g. "back_left" or "middle_right" or "front_left"
            ros_msg.wheel = f'{self.pair}_{axis}'
            self._vel_pub.publish(ros_msg)
        except DisconnectedError:
            return

    def _publish_encoder_msg(self) -> None:
        """Publishes velocity and current data over ROS
        of both the left and right axes."""
        self._publish_encoder_helper('left')
        self._publish_encoder_helper('right')

    def _publish_state_msg(self, state: str) -> None:
        """Publishes the ODrive state message
        over ROS to a topic"""
        ros_msg = DriveStateData()
        ros_msg.state = state
        ros_msg.pair = self._pair
        self._state_pub.publish(ros_msg)
        print(f'changed state to {state}')

    def _update(self) -> None:
        """Depending on the current state, it will change the action.
        In the armed state, it will first check for ODrive errors.
        Then, it will _update the speed.
        In the disconnected state, it will create an arm event.
        In the error state, it will create an error event."""
        if str(self._state) == "ArmedState":
            try:
                errors = self._modrive.check_errors()
                self._modrive.watchdog_feed()
            except DisconnectedError:
                errors = 0
                self._bridge_on_event(ODriveEvent.DISCONNECTED_ODRIVE_EVENT)
                return

            if errors:
                self._bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)
                return

            self._modrive.set_vel('left', self._left_speed)
            self._modrive.set_vel('right', self._right_speed)

        elif str(self._state) == "DisconnectedState":
            self._connect()
            self._bridge_on_event(ODriveEvent.ARM_CMD_EVENT)

        elif str(self._state) == "ErrorState":
            self._bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)


def main():
    """In main, the ros_publish_data_loop and
    rospy drive_vel_cmd subscriber thread
    and watchdog_while_loop all run simultaneously"""
    rospy.init_node(f"odrive_{int(sys.argv[1])}")

    bridge = ODriveBridge()

    # Starts the thread to listen to drive commands
    rospy.Subscriber("drive_vel_cmd", DriveVelCmd,
                     bridge.drive_vel_cmd_callback)

    # Starts the thread to listen to continuously publish data
    threading._start_new_thread(bridge.ros_publish_data_loop, ())

    # Starts the thread to continuously monitor comms between
    # basestation and jetson to act as watchdog.
    threading._start_new_thread(bridge.watchdog_while_loop, ())
    rospy.spin()
    exit()


if __name__ == "__main__":
    main()
