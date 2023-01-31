#!/usr/bin/env python3
import rospy
import serial
import time

from mrover.srv import (
    ChangeAutonLEDState,
    ChangeAutonLEDStateRequest,
    ChangeAutonLEDStateResponse,
)

desired_color = ""


def handle_change_auton_led_state(req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
    """Processes a request to change the auton LED array state by issuing
    the command to the Arduino via USB. Returns the success of the
    transaction.
    Sends 'r' for red, 'g' for green', 'b' for blue, 'o' for off.

    :param req: A string that is the color of the requested state of the
        auton LED array. Note that green actually means blinking green.
    :returns: A boolean that is the success of sent USB transaction.
    """

    global desired_color

    desired_color = req.color.lower()

    return ChangeAutonLEDStateResponse(True)


def main():
    rospy.init_node("auton_led")

    global desired_color

    # read serial connection info from parameter server
    port = rospy.get_param("auton_led_driver/port")
    baud = rospy.get_param("auton_led_driver/baud")

    # create serial connection with Arduino
    ser = serial.Serial(port, baud)

    rospy.Service("change_auton_led_state", ChangeAutonLEDState, handle_change_auton_led_state)

    color = ""
    green_counter_s = 0

    green_period_s = 2
    green_on_s = 1
    refresh_rate_s = green_on_s / 10  # this should be a factor of green_on_s

    while not rospy.is_shutdown():

        if color != desired_color:
            # send message
            if desired_color == "red":
                ser.write(b"r")
            elif desired_color == "green":
                green_counter_s = 0
                ser.write(b"g")
            elif desired_color == "blue":
                ser.write(b"b")
            else:
                ser.write(b"o")

            color = desired_color

        if color == "green":
            # if requested color is green,
            # then alternate between off and on
            # every second to make it a blinking LED

            if green_counter_s == green_period_s:
                green_counter_s = 0

            if green_counter_s == 0:
                ser.write(b"g")
            elif green_counter_s == green_on_s:
                ser.write(b"o")

            time.sleep(refresh_rate_s)
            green_counter_s += refresh_rate_s


if __name__ == "__main__":
    main()
