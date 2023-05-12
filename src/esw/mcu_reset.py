#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time as t

from std_msgs.msg import Bool

import rospy

from mrover.srv import (
    EnableDevice,
    EnableDeviceRequest,
    EnableDeviceResponse,
)

MOSFET_GATE_PIN = 4  # the pin used as the gate driver is GPIO 3
TIME_TO_RESET_S = 1.0

reset_mcu_autonomously = True
time_since_last_reset_mcu = t.time()
mcu_is_active = True
MCU_RESET_PERIOD_S = rospy.get_param("science/info/mcu_reset_period_s")


def reset_board() -> None:
    global time_since_last_reset_mcu, mcu_is_active
    
    rospy.logerr("Resetting the entire MCU Board.")
    GPIO.output(MOSFET_GATE_PIN, GPIO.LOW)
    t.sleep(TIME_TO_RESET_S)
    GPIO.output(MOSFET_GATE_PIN, GPIO.HIGH)
    rospy.logerr("Please restart the brushed_motors node!")

    time_since_last_reset_mcu = t.time()
 
    # Set mcu_is_active to true even though it might not actually be true.
    # This is because we want to reset the variable in case the science node stops publishing data.
    mcu_is_active = True


def handle_mcu_board_reset(req: EnableDeviceRequest) -> EnableDeviceResponse:
    if req.enable:
        reset_board()

    return EnableDeviceResponse(True)


def handle_reset_mcu_autonomously(req: EnableDeviceRequest) -> EnableDeviceResponse:
    global reset_mcu_autonomously
    reset_mcu_autonomously = req.enable
    return EnableDeviceResponse(True)


def check_mcu_disconnected(self, event=None) -> bool:
    """This should check if the MCU is disconnected.
    If it is disconnected AND if the MCU board has not been
    reset in the past 10 seconds, then reset it."""
 
    if (
        reset_mcu_autonomously and
        not mcu_is_active and
        t.time() - time_since_last_reset_mcu >= MCU_RESET_PERIOD_S
    ):
        reset_board()

    return True


def update_science_mcu_active(status: Bool) -> None:
    global mcu_is_active
    mcu_is_active = status.data


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOSFET_GATE_PIN, GPIO.OUT, initial=GPIO.HIGH)

    rospy.init_node("mcu_reset")
    rospy.Service("mcu_board_reset", EnableDevice, handle_mcu_board_reset)
    rospy.Service("reset_mcu_autonomously", EnableDevice, handle_reset_mcu_autonomously)
    rospy.Timer(rospy.Duration(1.0), check_mcu_disconnected)
    rospy.Subscriber("science_mcu_active", Bool, update_science_mcu_active)
    rospy.spin()


if __name__ == "__main__":
    main()
