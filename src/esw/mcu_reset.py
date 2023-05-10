import RPi.GPIO as GPIO
from time import sleep

import rospy

from std_msgs.msg import Bool

from mrover.srv import (
    EnableDevice,
    EnableDeviceRequest,
    EnableDeviceResponse,
)

MOSFET_GATE_PIN = 4  # the pin used as the gate driver is GPIO 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOSFET_GATE_PIN, GPIO.OUT, initial=GPIO.HIGH)  # TODO: make sure the initial value is correct


def reset_mcu(req: EnableDeviceRequest) -> EnableDeviceResponse:
    if req.enable:
        GPIO.output(MOSFET_GATE_PIN, GPIO.LOW)
        sleep(0.5)
        GPIO.output(MOSFET_GATE_PIN, GPIO.HIGH)

    return EnableDeviceResponse(True)


def main():
    rospy.init_node("mcu_reset")
    rospy.Service("mcu_board_reset", EnableDevice, reset_mcu)


if __name__ == "__main__":
    main()
