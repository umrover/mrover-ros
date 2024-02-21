#!/usr/bin/env python3
import rospy
import serial
import threading

from mrover.msg import LED

class LedBridge:
    """
    A class that keeps track of the Auton LED color and updates as necessary over serial.
    """

    # Number of seconds for the entire flashing period.
    BLINKING_PERIOD_S = 2

    # Number of seconds the LED should be flashing for each period.
    BLINKING_ON_S = 1

    # Ideally, SLEEP_AMOUNT divides BLINKING_PERIOD_S and BLINKING_ON_S cleanly to avoid slop.
    SLEEP_AMOUNT = 1

    # The color of the LED.
    _color: str
    _color_lock: threading.Lock

    # A counter for flashing, in seconds.
    _blinking_counter_s: int
    _is_blinking: bool

    # A serial connection to the Arduino.
    _ser: serial.Serial

    def __init__(self, port: str, baud: int):
        """
        :param port: port for the serial connection
        :param baud: baud rate for the serial connection
        """
        self._color = "w"
        self._color_lock = threading.Lock()

        self._blinking_counter_s = 0
        self._is_blinking = True

        self._ser = serial.Serial(port=port, baudrate=baud)

        with self._color_lock:
            self._update()

    def handle_message(self, color: LED) -> None:
        with self._color_lock:
            prev_color = self._color

            my_color = ""

            if color.red:
                if color.green:
                    if color.blue:
                        my_color = "w" # white - ALL COLORS
                    else:
                        my_color = "n" # brown - n since not blue - red and green
                else:
                    if color.blue:
                        my_color = "p" # purple - red and blue
                    else:
                        my_color = "r"
            else:
                if color.green:
                    if color.blue:
                        my_color = "t" # teal - green and blue
                    else:
                        my_color = "g" # green
                else:
                    if color.blue:
                        my_color = "b" # blue
                    else:
                        my_color = "o" # off

            self._color = my_color

            self._is_blinking = color.is_blinking

            # If the desired color is red, blue, or off, then just update:
            if not self._is_blinking:
                self._update()

            # If the desired color is green, and we previously were not green, then update too.
            elif self._is_blinking:
                # Set to self.BLINKING_PERIOD_S instead of 0,
                # so it just automatically resets at 0 once
                # flash_if_blinking() is called.
                self._blinking_counter_s = self.BLINKING_PERIOD_S
                self._update()

    def _update(self):
        """
        Writes to serial to change LED color.
        Note: assumes self._color_lock is held!
        """
        self._ser.write(self._color)

    def flash_if_blinking(self, event=None):
        """
        Flash green if necessary. Function is expected to be called every self.SLEEP_AMOUNT seconds.
        """
        if not self._is_blinking:
            return

        with self._color_lock:

            # Copy the counter.
            prev_counter = self._blinking_counter_s

            # Increment the counter however much was slept.
            self._blinking_counter_s += self.SLEEP_AMOUNT

            # Switch to green at the end of a period.
            if self._blinking_counter_s >= self.BLINKING_PERIOD_S:
                self._blinking_counter_s = 0
                self._ser.write(self._color)

            # If we just passed the threshold of BLINKING_ON_S, turn off.
            elif prev_counter < self.BLINKING_ON_S <= self._blinking_counter_s:
                self._ser.write("o")


def main():
    rospy.init_node("arduino_led_hw_bridge")

    # read serial connection info from parameter server
    port = rospy.get_param("auton_led_driver/port")
    baud = rospy.get_param("auton_led_driver/baud")

    # Construct the bridge.
    led_bridge = LedBridge(port, baud)

    # Configure subscriber.
    rospy.Subscriber("led", LED, led_bridge.handle_message)

    # Sleep indefinitely, flashing if necessary.
    rospy.Timer(rospy.Duration(led_bridge.SLEEP_AMOUNT), led_bridge.flash_if_blinking)

    rospy.spin()


if __name__ == "__main__":
    main()