#!/usr/bin/env python3
import rospy
import serial
import time
import threading

from mrover.srv import (
    ChangeAutonLEDState,
    ChangeAutonLEDStateRequest,
    ChangeAutonLEDStateResponse,
)


class LedBridge:
    """
    A class that keeps track of the Auton LED color and updates as necessary over serial.
    """

    GREEN_PERIOD_S = 2
    GREEN_ON_S = 1
    SLEEP_AMOUNT = 1

    _color: str
    _color_lock: threading.Lock

    _green_counter_s: int
    _ser: serial.Serial

    def __init__(self, port: str, baud: int):
        self._color = ""
        self._color_lock = threading.Lock()

        self._green_counter_s = 0

        # create serial connection with Arduino
        self._ser = serial.Serial(port=port, baudrate=baud)

        self.update()

    def handle_change_state(self, req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
        """
        Processes a request to change the auton LED array state by changing the desired color.
        Returns the success of the transaction.

        :param req: A string that is the color of the requested state of the
            auton LED array. Note that green actually means blinking green.
        :returns: A boolean that is always True.
        """

        with self._color_lock:
            self._color = req.color.lower()

        self.update()

        return ChangeAutonLEDStateResponse(True)

    def flash_if_green(self):
        """
        Updates serial and green counter if the requested color is green.
        """
        with self._color_lock:

            if self._color != "green":
                return

            # If requested color is green, then alternate between off and on.
            if self._green_counter_s >= self.GREEN_PERIOD_S:
                self._green_counter_s = 0

            if self._green_counter_s == 0:
                self._ser.write(b"g")
            elif self._green_counter_s >= self.GREEN_ON_S:
                self._ser.write(b"o")

    def sleep(self):
        """
        Sleeps and updates _green_counter_s if necessary.
        """
        time.sleep(self.SLEEP_AMOUNT)

        with self._color_lock:
            # if requested color is green,
            if self._color == "green":
                self._green_counter_s += self.SLEEP_AMOUNT

    def update(self):
        """
        Writes to serial to change LED color.
        """
        with self._color_lock:
            if self._color == "red":
                self._ser.write(b"r")

            elif self._color == "green":
                self._ser.write(b"g")
                self._green_counter_s = 0

            elif self._color == "blue":
                self._ser.write(b"b")

            else:
                self._ser.write(b"o")


def main():
    rospy.init_node("auton_led")

    # read serial connection info from parameter server
    port = rospy.get_param("auton_led_driver/port")
    baud = rospy.get_param("auton_led_driver/baud")

    # Construct the bridge.
    led_bridge = LedBridge(port, baud)

    # Configure request handler.
    rospy.Service("change_auton_led_state", ChangeAutonLEDState, led_bridge.handle_change_state)

    # Sleep indefinitely, flashing if necessary.
    while not rospy.is_shutdown():
        led_bridge.flash_if_green()
        led_bridge.sleep()


if __name__ == "__main__":
    main()
