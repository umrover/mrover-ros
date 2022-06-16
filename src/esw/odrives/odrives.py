"""This code controls one ODrive.
It takes in a command line argument (0, 1, or 2) to see which
ODrive it is controlling. 0 means front, 1 means middle,
2 means back. This means that to contrl 3 separate ODrives,
3 of these programs must be running simultaneously.
Configuration variables are located
in the config.py file. The ODriveBridge object controls
the behavior of the ODrive.
The ODriveBridge object keeps track of a state that it is in.
A State may change to a different state depending on an Event.
The ODrive may either be in an error, disconnected, or armed state.
"""
import sys
import threading
import time as t

import fibre
import odrive as odv
import rospy
from config import (AXIS_SPEED_MULTIPLIER_MAP,
                    AXIS_VEL_ESTIMATE_MULTIPLIER_MAP, CURRENT_LIM, MOTOR_MAP,
                    ODRIVE_IDS, ODRIVE_WATCHDOG_TIMEOUT, PAIR, Axis,
                    ODriveEvent)
from mrover.msg import DriveStateData, DriveVelCmd, DriveVelData
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                          CONTROL_MODE_VELOCITY_CONTROL)
from odrive.utils import dump_errors

MODRIVE, ODRIVE_BRIDGE = None, None
LEFT_SPEED, RIGHT_SPEED = 0.0, 0.0
ODRIVE_PAIR = PAIR(int(sys.argv[1]))
USB_LOCK, SPEED_LOCK = threading.Lock(), threading.Lock()
VEL_PUB = rospy.Publisher('drive_vel_data', DriveVelData, queue_size=1)
STATE_PUB = rospy.Publisher('drive_state_data', DriveStateData, queue_size=1)
START_TIME = t.clock()


def main():
    """In main, the ros_publisher_thread and
    rospy drive_vel_cmd subscriber thread
    runs in the background
    at the same time as the watchdog while loop
    is running."""
    global LEFT_SPEED, RIGHT_SPEED, ODRIVE_BRIDGE
    rospy.init_node("odrives")

    rospy.Subscriber("drive_vel_cmd", DriveVelCmd, drive_vel_cmd_callback)
    threading._start_new_thread(ros_publisher_thread, ())
    ODRIVE_BRIDGE = ODriveBridge()

    # starting state is DisconnectedState()
    # start up sequence is called, disconnected-->arm

    # flag for state when we have comms with base_station vs not
    previously_lost_comms = True

    while not rospy.is_shutdown():
        watchdog = t.clock() - START_TIME

        lost_comms = watchdog > 1.0
        if lost_comms:
            if not previously_lost_comms:
                print("loss of comms")
                previously_lost_comms = True

            SPEED_LOCK.acquire()
            LEFT_SPEED = RIGHT_SPEED = 0
            SPEED_LOCK.release()
        elif previously_lost_comms:
            previously_lost_comms = False
            print("regained comms")

        try:
            ODRIVE_BRIDGE.update()
        except Exception:
            if USB_LOCK.locked():
                USB_LOCK.release()

            USB_LOCK.acquire()
            ODRIVE_BRIDGE.bridge_on_event(
                ODriveEvent.DISCONNECTED_ODRIVE_EVENT)
            USB_LOCK.release()

    exit()


def ros_publisher_thread() -> None:
    """This loop continuously publishes
    velocity and current data."""
    while not rospy.is_shutdown():
        global START_TIME
        START_TIME = t.clock()
        try:
            publish_encoder_msg()
        except Exception:
            if USB_LOCK.locked():
                USB_LOCK.release()


class State(object):
    """
    State object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print('Processing current state:', str(self))

    def on_event(self, event: ODriveEvent) -> None:
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


class DisconnectedState(State):
    """This is the State for when the ODrive has disconnected"""
    def on_event(self, event: ODriveEvent) -> None:
        """
        Handle events that are delegated to the Disconnected State.
        """
        if event == ODriveEvent.ARM_CMD_EVENT:
            MODRIVE.disarm()
            MODRIVE.reset_watchdog()
            MODRIVE.arm()
            return ArmedState()

        return self


class ArmedState(State):
    """This is the State for when the ODrive is armed"""
    def on_event(self, event: ODriveEvent) -> None:
        """
        Handle events that are delegated to the Armed State.
        """
        if event == ODriveEvent.DISCONNECTED_ODRIVE_EVENT:
            return DisconnectedState()

        elif event == ODriveEvent.ODRIVE_ERROR_EVENT:
            return ErrorState()

        return self


class ErrorState(State):
    """This is the State for when the ODrive is receiving errors"""
    def on_event(self, event: ODriveEvent) -> None:
        """
        Handle events that are delegated to the Error State.
        """
        print(dump_errors(MODRIVE.odrive, True))
        if event == ODriveEvent.ODRIVE_ERROR_EVENT:
            try:
                MODRIVE.reboot()  # only runs after initial pairing
            except Exception:
                pass

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
        self.state = DisconnectedState()  # default is disconnected
        self.left_speed = self.right_speed = 0.0

    def connect(self) -> None:
        """This will attempt to connect to an ODrive.
        This will use the ODrive library to look for an
        ODrive with the specified ID on the Jetson."""
        global MODRIVE
        print("looking for odrive")

        odrive_id = ODRIVE_IDS[ODRIVE_PAIR.value]

        print(odrive_id)
        odrive = odv.find_any(serial_number=odrive_id)

        print("found odrive")
        USB_LOCK.acquire()
        MODRIVE = Modrive(odrive)
        MODRIVE.set_current_lim(CURRENT_LIM)
        USB_LOCK.release()

    def bridge_on_event(self, event: ODriveEvent) -> None:
        """
        Incoming events are
        delegated to the given states which then handle the event.
        The result is then assigned as the new state.
        """

        print("on event called, ODrive event:", event)

        self.state = self.state.on_event(event)
        publish_state_msg(ODRIVE_BRIDGE.get_state_string())

    def update(self) -> None:
        """Depending on the current state, it will change the action.
        In the armed state, it will first check for ODrive errors.
        Then, it will update the speed.
        In the disconnected state, it will create an arm event.
        In the error state, it will create an error event."""
        if str(self.state) == "ArmedState":
            try:
                USB_LOCK.acquire()
                errors = MODRIVE.check_errors()
                MODRIVE.watchdog_feed()
                USB_LOCK.release()

            except Exception:
                if USB_LOCK.locked():
                    USB_LOCK.release()
                errors = 0
                USB_LOCK.acquire()
                self.bridge_on_event(ODriveEvent.DISCONNECTED_ODRIVE_EVENT)
                USB_LOCK.release()

            if errors:
                USB_LOCK.acquire()
                self.bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)
                USB_LOCK.release()
                return

            SPEED_LOCK.acquire()
            self.left_speed, self.right_speed = LEFT_SPEED, RIGHT_SPEED
            SPEED_LOCK.release()

            USB_LOCK.acquire()
            MODRIVE.set_vel(Axis.LEFT, self.left_speed)
            MODRIVE.set_vel(Axis.RIGHT, self.right_speed)
            USB_LOCK.release()

        elif str(self.state) == "DisconnectedState":
            self.connect()
            USB_LOCK.acquire()
            self.bridge_on_event(ODriveEvent.ARM_CMD_EVENT)
            USB_LOCK.release()

        elif str(self.state) == "ErrorState":
            USB_LOCK.acquire()
            self.bridge_on_event(ODriveEvent.ODRIVE_ERROR_EVENT)
            USB_LOCK.release()

    def get_state_string(self) -> str:
        """Returns the state of the ODriveBridge as a string"""
        return str(self.state)


def publish_state_msg(state: str) -> None:
    """Publishes the ODrive state message
    over ROS to a topic"""
    ros_msg = DriveStateData()
    # Shortens the state string which is of
    # the form "[insert_odrive_state]State"
    # e.g. state is ErrorState, so ros_msg.state is Error
    ros_msg.state = state[:len(state) - len("State")]
    ros_msg.odrive_index = ODRIVE_PAIR.value
    STATE_PUB.publish(ros_msg)
    print("changed state to " + state)


def publish_encoder_helper(axis: int) -> None:
    """Publishes the velocity and current
    data message over ROS of the requested axis"""
    ros_msg = DriveVelData()
    direction_multiplier = -1

    USB_LOCK.acquire()
    ros_msg.current_amps = MODRIVE.get_measured_current(axis) * \
        direction_multiplier
    ros_msg.vel_m_s = MODRIVE.get_vel_estimate(axis) * direction_multiplier
    USB_LOCK.release()

    ros_msg.axis = MOTOR_MAP[(axis, ODRIVE_PAIR)]

    VEL_PUB.publish(ros_msg)


def publish_encoder_msg() -> None:
    """Publishes velocity and current data over ROS
    of both the left and right axes."""
    publish_encoder_helper(Axis.LEFT)
    publish_encoder_helper(Axis.RIGHT)


def drive_vel_cmd_callback(ros_msg: DriveVelCmd) -> None:
    """Set the global speed to the requested speed in the ROS message.
    Note that this does NOT actually change speed that the ODrive comands
    the motors at. One must wait for the ODriveBridge.update() function
    to be called for that to happen."""

    try:
        if ODRIVE_BRIDGE.get_state_string() == "ArmedState":
            global LEFT_SPEED, RIGHT_SPEED

            SPEED_LOCK.acquire()
            LEFT_SPEED, RIGHT_SPEED = ros_msg.left, ros_msg.right
            SPEED_LOCK.release()
    except Exception:
        return


if __name__ == "__main__":
    main()


class Modrive:
    """This is the class for the ODrives. This object contains
    the ODrive object that is used in the ODrive library in self.odrive"""
    def __init__(self, odr):
        self.odrive = odr
        self.axes = [self.odrive.axis0, self.odrive.axis1]
        self.set_current_lim(CURRENT_LIM)

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def enable_watchdog(self) -> None:
        """This enables the watchdog of the ODrives."""
        try:
            print("Enabling watchdog")
            for axis in self.axes:
                axis.config.watchdog_timeout = ODRIVE_WATCHDOG_TIMEOUT
                self.watchdog_feed()
                axis.config.enable_watchdog = True
        except Exception as exc:
            print(exc)

    def disable_watchdog(self) -> None:
        """This disables the watchdog of the odrives"""
        try:
            print("Disabling watchdog")
            for axis in self.axes:
                axis.config.watchdog_timeout = 0
                axis.config.enable_watchdog = False
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def reset_watchdog(self) -> None:
        """This resets the watchdog of the odrives.
        This is done in case there was previously an error
        caused by the watchdog."""
        try:
            print("Resetting watchdog")
            self.disable_watchdog()
            # clears errors cleanly
            for axis in self.axes:
                axis.error = 0
            self.enable_watchdog()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def watchdog_feed(self) -> None:
        """Refreshes the watchdog feed."""
        try:
            for axis in self.axes:
                axis.watchdog_feed()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in watchdog_feed. Unplugged")

    def disarm(self) -> None:
        """Disarms the ODrive and sets the velocity to 0"""
        self.set_current_lim(CURRENT_LIM)
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

        self.set_vel(Axis.LEFT, 0)
        self.set_vel(Axis.RIGHT, 0)

        self.idle()

    def arm(self) -> None:
        """Arms the ODrive"""
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

    def set_current_lim(self, lim):
        """Sets the current limit of both ODrive axes"""
        for axis in self.axes:
            axis.motor.config.current_lim = lim

    def _set_control_mode(self, mode) -> None:
        """Sets the control mode of the ODrive"""
        for axis in self.axes:
            axis.controller.config.control_mode = mode

    def set_velocity_ctrl(self) -> None:
        """Sets the ODrive to velocity control"""
        self._set_control_mode(CONTROL_MODE_VELOCITY_CONTROL)

    def get_measured_current(self, axis: Axis) -> float:
        """Returns the measured current of
        the requested axis of the ODrive"""
        # measured current [Amps]
        return self.axes[axis.value].motor.current_control.Iq_measured

    def get_vel_estimate(self, axis: Axis) -> float:
        """Returns the estimated velocity of
        the requested axis of the ODrive"""
        return self.axes[axis.value].encoder.vel_estimate * \
            AXIS_VEL_ESTIMATE_MULTIPLIER_MAP[axis.value]

    def idle(self) -> None:
        """Sets the ODrive state to idle"""
        self._requested_state(AXIS_STATE_IDLE)

    def closed_loop_ctrl(self) -> None:
        """Sets the ODrive state to closed loop control"""
        self._requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def _requested_state(self, state) -> None:
        """Sets each ODrive axis state to the requested state"""
        for axis in self.axes:
            axis.requested_state = state

    def set_vel(self, axis: Axis, vel) -> None:
        """Sets the requested ODrive axis to run the
        motors at the requested velocity"""
        desired_input_vel = vel * \
            AXIS_SPEED_MULTIPLIER_MAP[axis.value]

        self.axes[axis.value].controller.input_vel = desired_input_vel

    def check_errors(self) -> bool:
        """Returns value of sum of errors"""
        return self.axes[Axis.LEFT.value].error + \
            self.axes[Axis.RIGHT.value].error != 0
