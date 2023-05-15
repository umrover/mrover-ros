#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import time, sleep

from std_msgs.msg import Bool

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, Trigger, TriggerRequest, TriggerResponse

MOSFET_GATE_PIN = 12  # the pin used as the gate driver is GPIO 12
"""
The GPIO pin used to reset the MCU board.
"""

TIME_TO_RESET_S = 3.0
"""
The amount of time in seconds to hold the reset pin.
"""

MCU_RESET_PERIOD_S = rospy.get_param("science/info/mcu_reset_period_s")
"""
The minimum amount of time between automatic resets in seconds.
"""


class ResetManager:
    mcu_is_active: bool
    """
    Whether the MCU is active, based on data from the science.py node.
    """

    reset_mcu_autonomously: bool
    """
    Whether the MCU should be reset automatically, based on commands from teleop.
    """

    time_of_last_reset_mcu: float
    """
    The time that the MCU was last reset by this node.
    """

    def __init__(self):
        self.mcu_is_active = True
        self.reset_mcu_autonomously = False

        self.time_of_last_reset = time() - MCU_RESET_PERIOD_S

    def update_mcu_active(self, status: Bool) -> None:
        """
        Handle a message from the science node of whether the MCU is still active.
        """
        self.mcu_is_active = status.data

    def reset_board(self) -> None:
        """
        Toggle GPIO pins to reset the MCU board, updating the necessary state.
        """

        rospy.logerr("Resetting the entire MCU Board.")

        self.time_of_last_reset = time()

        GPIO.output(MOSFET_GATE_PIN, GPIO.LOW)
        sleep(TIME_TO_RESET_S)
        GPIO.output(MOSFET_GATE_PIN, GPIO.HIGH)

        # Set mcu_is_active to true even though it might not actually be true.
        # This is because we want to reset the variable in case the science node stops publishing data.
        self.mcu_is_active = True

    def handle_mcu_board_reset(self, req: TriggerRequest) -> TriggerResponse:
        """
        Handle a direct request to reset MCU board, which should only set enable to true.
        """
        self.reset_board()
        return TriggerResponse(success=True, message="")

    def handle_reset_mcu_autonomously(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Handle a request to change whether the automatically reset the MCU.
        """
        self.reset_mcu_autonomously = req.data
        return SetBoolResponse(success=True, message="")

    def check_mcu_disconnected(self, event=None) -> None:
        """This should check if the MCU is disconnected.
        If it is disconnected AND if the MCU board has not been
        reset in the past 10 seconds, then reset it."""

        time_since_last_reset = time() - self.time_of_last_reset

        if self.reset_mcu_autonomously and not self.mcu_is_active and time_since_last_reset >= MCU_RESET_PERIOD_S:
            self.reset_board()


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOSFET_GATE_PIN, GPIO.OUT, initial=GPIO.HIGH)

    rospy.init_node("mcu_reset")

    manager = ResetManager()

    rospy.Subscriber("science_mcu_active", Bool, manager.update_mcu_active)
    rospy.Service("mcu_board_reset", Trigger, manager.handle_mcu_board_reset)
    rospy.Service("reset_mcu_autonomously", SetBool, manager.handle_reset_mcu_autonomously)

    # Check MCU for potential resets once per second.
    rospy.Timer(rospy.Duration(1.0), manager.check_mcu_disconnected)

    rospy.spin()


if __name__ == "__main__":
    main()
