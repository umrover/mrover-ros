#!/usr/bin/env python3
import rospy
import serial
import threading


from std_msgs.msg import String


class LedBridge:
    """
    A class that keeps track of the Auton LED color and updates as necessary over serial.
    """

    # Number of seconds for the entire flashing period.
    GREEN_PERIOD_S = 2

    # Number of seconds the LED should be green for each period.
    GREEN_ON_S = 1

    # Ideally, SLEEP_AMOUNT divides GREEN_PERIOD_S and GREEN_ON_S cleanly to avoid slop.
    SLEEP_AMOUNT = 1

    # Maps a color to the byte to send over serial.
    SIGNAL_MAP = {"red": b"r", "green": b"g", "blue": b"b", "off": b"o"}

    # The color of the LED.
    _color: str
    _color_lock: threading.Lock

    # A counter for flashing green, in seconds.
    _green_counter_s: int

    # A serial connection to the Arduino.
    _ser: serial.Serial

    def __init__(self, port: str, baud: int):
        """
        :param port: port for the serial connection
        :param baud: baud rate for the serial connection
        """
        self._color = "off"
        self._color_lock = threading.Lock()

        self._green_counter_s = 0

        self._ser = serial.Serial(port=port, baudrate=baud)

        with self._color_lock:
            self._update()

    def handle_message(self, color: String) -> None:
        """
        Processes the current color requested by teleop.

        :param color: A string that is the color of the requested state of the
            auton LED array. Note that green actually means blinking green.
        """
        if color.data.lower() not in self.SIGNAL_MAP:
            rospy.logerr(f"Auton LED node received invalid color: {color.data}")
            return

        with self._color_lock:
            prev_color = self._color
            self._color = color.data.lower()

            # If the desired color is red, blue, or off, then just update:
            if self._color != "green":
                self._update()

            # If the desired color is green, and we previously were not green, then update too.
            elif prev_color != "green":
                # Set to self.GREEN_PERIOD_S instead of 0,
                # so it just automatically resets at 0 once
                # flash_if_green() is called.
                self._green_counter_s = self.GREEN_PERIOD_S
                self._update()

    def _update(self):
        """
        Writes to serial to change LED color.
        Note: assumes self._color_lock is held!
        """
        assert self._color in self.SIGNAL_MAP
        self._ser.write(self.SIGNAL_MAP[self._color])

    def flash_if_green(self, event=None):
        """
        Flash green if necessary. Function is expected to be called every self.SLEEP_AMOUNT seconds.
        """
        # Upon waking up, flash green if necessary
        with self._color_lock:
            if self._color != "green":
                return

            # Copy the counter.
            prev_counter = self._green_counter_s

            # Increment the counter however much was slept.
            self._green_counter_s += self.SLEEP_AMOUNT

            # Switch to green at the end of a period.
            if self._green_counter_s >= self.GREEN_PERIOD_S:
                self._green_counter_s = 0
                self._ser.write(self.SIGNAL_MAP["green"])

            # If we just passed the threshold of GREEN_ON_S, turn off.
            elif prev_counter < self.GREEN_ON_S <= self._green_counter_s:
                self._ser.write(self.SIGNAL_MAP["off"])


def main():
    rospy.init_node("auton_led")

    # read serial connection info from parameter server
    port = rospy.get_param("auton_led_driver/port")
    baud = rospy.get_param("auton_led_driver/baud")

    # Construct the bridge.
    led_bridge = LedBridge(port, baud)

    # Configure subscriber.
    rospy.Subscriber("auton_led_cmd", String, led_bridge.handle_message)

    # Sleep indefinitely, flashing if necessary.
    rospy.Timer(rospy.Duration(led_bridge.SLEEP_AMOUNT), led_bridge.flash_if_green)

    rospy.spin()


if __name__ == "__main__":
    main()
