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

import fibre
import odrive as odv
import rospy
from mrover.msg import DriveStateData, DriveVelCmd, DriveVelData
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                          CONTROL_MODE_VELOCITY_CONTROL)
from odrive.utils import dump_errors


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
    def __init__(self, odr, axis_0_str: str, axis_1_str: str,
                 vel_cmd_mult_left: float, vel_cmd_mult_right: float,
                 vel_est_mult_left: float, vel_est_mult_right: float,
                 watchdog_timeout: float):
        self.odrive = odr
        self.axes = {
            axis_0_str: self.odrive.axis0,
            axis_1_str: self.odrive.axis1}

        # This scales [0, 1] to [0, vel_cmd_mult] turns per second
        self.axis_vel_command_multiplier_map = {
            'left': vel_cmd_mult_left,
            'right': vel_cmd_mult_right}

        # This scales turns/sec to m/sec
        self.axis_vel_estimate_multiplier_map = {
            'left': vel_est_mult_left,
            'right': vel_est_mult_right}
        self.watchdog_timeout = watchdog_timeout

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def arm(self) -> None:
        """Arms the ODrive"""
        self.set_closed_loop_ctrl()
        self.set_velocity_ctrl()

    def check_errors(self) -> bool:
        """Returns value of sum of errors"""
        return self.axes['left'].error + \
            self.axes['right'].error != 0

    def enable_watchdog(self) -> None:
        """This enables the watchdog of the ODrives."""
        try:
            print("Enabling watchdog")
            for axis in self.axes.values():
                axis.config.watchdog_timeout = self.watchdog_timeout
                self.watchdog_feed()
                axis.config.enable_watchdog = True
        except Exception as exc:
            print(exc)

    def disable_watchdog(self) -> None:
        """This disables the watchdog of the ODrives"""
        try:
            print("Disabling watchdog")
            for axis in self.axes.values():
                axis.config.watchdog_timeout = 0
                axis.config.enable_watchdog = False
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def disarm(self) -> None:
        """Disarms the ODrive and sets the velocity to 0"""
        self.set_closed_loop_ctrl()
        self.set_velocity_ctrl()

        self.set_vel('left', 0.0)
        self.set_vel('right', 0.0)

        self.idle()

    def get_measured_current(self, axis: str) -> float:
        """Returns the measured current of
        the requested axis of the ODrive"""
        # measured current [Amps]
        return self.axes[axis].motor.current_control.Iq_measured

    def get_vel_estimate_m_s(self, axis: str) -> float:
        """Returns the estimated velocity of
        the requested axis of the ODrive"""
        return self.axes[axis].encoder.vel_estimate * \
            self.axis_vel_estimate_multiplier_map[axis]

    def idle(self) -> None:
        """Sets the ODrive state to idle"""
        self.set_requested_state(AXIS_STATE_IDLE)

    def reset_watchdog(self) -> None:
        """This resets the watchdog of the ODrives.
        This is done in case there was previously an error
        caused by the watchdog."""
        try:
            print("Resetting watchdog")
            self.disable_watchdog()
            # clears errors cleanly
            for axis in self.axes.values():
                axis.error = 0
            self.enable_watchdog()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def set_closed_loop_ctrl(self) -> None:
        """Sets the ODrive state to closed loop control"""
        self.set_requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def set_control_mode(self, mode) -> None:
        """Sets the control mode of the ODrive"""
        for axis in self.axes.values():
            axis.controller.config.control_mode = mode

    def set_current_lim(self, lim: float):
        """Sets the current limit of both ODrive axes"""
        for axis in self.axes.values():
            axis.motor.config.current_lim = lim

    def set_requested_state(self, state) -> None:
        """Sets each ODrive axis state to the requested state"""
        for axis in self.axes.values():
            axis.requested_state = state

    def set_vel(self, axis: str, vel: float) -> None:
        """Sets the requested ODrive axis to run the
        motors at the requested velocity"""
        desired_input_vel_turns_s = vel * \
            self.axis_vel_command_multiplier_map[axis]
        self.axes[axis].controller.input_vel = desired_input_vel_turns_s

    def set_velocity_ctrl(self) -> None:
        """Sets the ODrive to velocity control"""
        self.set_control_mode(CONTROL_MODE_VELOCITY_CONTROL)

    def watchdog_feed(self) -> None:
        """Refreshes the watchdog feed."""
        try:
            for axis in self.axes.values():
                axis.watchdog_feed()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in watchdog_feed. Unplugged")


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

    def __init__(self) -> None:
        """
        Initialize the components.
        Start with a Default State
        """
        # Maps the ODrive axis to either left or right
        self.axis_map: dict[int, str] = {
            rospy.get_param("/odrive/axis/left"): 'left',
            rospy.get_param("/odrive/axis/right"): 'right'}
        self.current_lim: float = rospy.get_param("/odrive/config/current_lim")
        self.left_speed: float = 0.0
        self.modrive = None
        self.odrive_ids: dict[str, str] = {
            'front': rospy.get_param("/odrive/ids/front"),
            'middle': rospy.get_param("/odrive/ids/middle"),
            'back': rospy.get_param("/odrive/ids/back")}
        self.right_speed: float = 0.0
        self.pair = sys.argv[1]
        self.speed_lock = threading.Lock()
        # This scales [0, 1] to [0, vel_cmd_mult] turns per second
        self.vel_cmd_mult_left: float = \
            rospy.get_param("/odrive/multiplier/vel_cmd_multiplier_left")
        self.vel_cmd_mult_right: float = \
            rospy.get_param("/odrive/multiplier/vel_cmd_multiplier_right")
        self.start_time = t.clock()
        self.state = DisconnectedState()
        self.state_pub = rospy.Publisher(
            'drive_state_data', DriveStateData, queue_size=1)
        self.usb_lock = threading.Lock()
        # This scales turns/sec to m/sec
        self.vel_est_mult_left: float = \
            rospy.get_param("/odrive/multiplier/vel_est_multiplier_left")
        self.vel_est_mult_right: float = \
            rospy.get_param("/odrive/multiplier/vel_est_multiplier_right")
        self.vel_pub = rospy.Publisher(
            'drive_vel_data', DriveVelData, queue_size=1)
        self.watchdog_timeout = \
            rospy.get_param("/odrive/config/watchdog_timeout")

    def bridge_on_event(self, event: ODriveEvent) -> None:
        """
        Incoming events are
        delegated to the given states which then handle the event.
        The result is then assigned as the new state.
        """

        print("on event called, ODrive event:", event)

        self.state = self.state.on_event(event, self.modrive)
        self.publish_state_msg(self.get_state_string())

    def connect(self) -> None:
        """This will attempt to connect to an ODrive.
        This will use the ODrive library to look for an
        ODrive with the specified ID on the Jetson."""
        print("looking for ODrive")

        odrive_id = self.odrive_ids[self.pair]

        print(odrive_id)
        odrive = odv.find_any(serial_number=odrive_id)

        print("found odrive")
        self.usb_lock.acquire()
        self.modrive = Modrive(
            odrive, self.axis_map[0], self.axis_map[1],
            self.vel_cmd_mult_left, self.vel_cmd_mult_right,
            self.vel_est_mult_left, self.vel_est_mult_right,
            self.watchdog_timeout)
        self.modrive.set_current_lim(self.current_lim)
        self.usb_lock.release()

    def drive_vel_cmd_callback(self, ros_msg: DriveVelCmd) -> None:
        """Set the global speed to the requested speed in the ROS message.
        Note that this does NOT actually change speed that the ODrive comands
        the motors at. One must wait for the ODriveBridge.update() function
        to be called for that to happen."""
        try:
            if self.get_state_string() == "Armed":
                self.speed_lock.acquire()
                self.left_speed, self.right_speed = ros_msg.left, ros_msg.right
                self.speed_lock.release()
        except Exception:
            return

    def get_state_string(self) -> str:
        """Returns the state of the ODriveBridge
        as a short string which is of
        the form "[insert_odrive_state]State"
        e.g. state is ErrorState,
        so ros_msg.state is Error"""
        state = str(self.state)
        return state[:len(state) - len("State")]

    def publish_encoder_helper(self, axis: str) -> None:
        """Publishes the velocity and current
        data message over ROS of the requested axis"""
        ros_msg = DriveVelData()
        direction_multiplier = -1

        self.usb_lock.acquire()
        ros_msg.current_amps = self.modrive.get_measured_current(axis) * \
            direction_multiplier
        ros_msg.vel_m_s = self.modrive.get_vel_estimate_m_s(axis) * \
            direction_multiplier
        self.usb_lock.release()

        # e.g. "back_left" or "middle_right" or "front_left"
        ros_msg.wheel = self.pair + "_" + axis

        self.vel_pub.publish(ros_msg)

    def publish_encoder_msg(self) -> None:
        """Publishes velocity and current data over ROS
        of both the left and right axes."""
        self.publish_encoder_helper('left')
        self.publish_encoder_helper('right')

    def publish_state_msg(self, state: str) -> None:
        """Publishes the ODrive state message
        over ROS to a topic"""
        ros_msg = DriveStateData()
        ros_msg.state = state
        ros_msg.pair = self.pair
        self.state_pub.publish(ros_msg)
        print("changed state to " + state)

    def ros_publish_data_loop(self) -> None:
        """This loop continuously publishes
        velocity and current data."""
        while not rospy.is_shutdown():
            self.start_time = t.clock()
            try:
                self.publish_encoder_msg()
            except Exception:
                if self.usb_lock.locked():
                    self.usb_lock.release()

    def update(self) -> None:
        """Depending on the current state, it will change the action.
        In the armed state, it will first check for ODrive errors.
        Then, it will update the speed.
        In the disconnected state, it will create an arm event.
        In the error state, it will create an error event."""
        if str(self.state) == "ArmedState":
            try:
                self.usb_lock.acquire()
                errors = self.modrive.check_errors()
                self.modrive.watchdog_feed()
                self.usb_lock.release()

            except Exception:
                if self.usb_lock.locked():
                    self.usb_lock.release()
                errors = 0
                self.usb_lock.acquire()
                self.bridge_on_event(ODriveEvent.DISCONNECTED_ODRIVE_EVENT)
                self.usb_lock.release()

            if errors:
                self.usb_lock.acquire()
                self.bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)
                self.usb_lock.release()
                return

            self.usb_lock.acquire()
            self.modrive.set_vel('left', self.left_speed)
            self.modrive.set_vel('right', self.right_speed)
            self.usb_lock.release()

        elif str(self.state) == "DisconnectedState":
            self.connect()
            self.usb_lock.acquire()
            self.bridge_on_event(ODriveEvent.ARM_CMD_EVENT)
            self.usb_lock.release()

        elif str(self.state) == "ErrorState":
            self.usb_lock.acquire()
            self.bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)
            self.usb_lock.release()

    def watchdog_while_loop(self):
        """Watchdog while loop"""
        # flag for state when we have comms with base_station vs not
        previously_lost_comms = True
        while not rospy.is_shutdown():
            watchdog = t.clock() - self.start_time
            lost_comms = watchdog > 1.0
            if lost_comms:
                if not previously_lost_comms:
                    # done to print "loss of comms" once
                    print("loss of comms")
                    previously_lost_comms = True

                self.speed_lock.acquire()
                self.left_speed = self.right_speed = 0.0
                self.speed_lock.release()
            elif previously_lost_comms:
                previously_lost_comms = False
                print("regained comms")
            try:
                self.update()
            except Exception:
                if self.usb_lock.locked():
                    self.usb_lock.release()

                self.usb_lock.acquire()
                self.bridge_on_event(
                    ODriveEvent.DISCONNECTED_ODRIVE_EVENT)
                self.usb_lock.release()


def main():
    """In main, the ros_publish_data_loop and
    rospy drive_vel_cmd subscriber thread
    and watchdog_while_loop all run simultaneously"""
    rospy.init_node("odrive_" + str(int(sys.argv[1])))

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
