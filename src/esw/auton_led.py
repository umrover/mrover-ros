#!/usr/bin/env python3
import rospy
import serial
import time

from mrover.srv import (
    ChangeAutonLEDState,
    ChangeAutonLEDStateRequest,
    ChangeAutonLEDStateResponse,
)

ser = serial.Serial
color = ""
desired_color = ""
green_counter_s = 0


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

    global ser, color, desired_color, green_counter_s

    # read serial connection info from parameter server
    port = rospy.get_param("auton_led_driver/port")
    baud = rospy.get_param("auton_led_driver/baud")

    # create serial connection with Arduino
    ser = serial.Serial(port, baud)

    rospy.Service("change_auton_led_state", ChangeAutonLEDState, handle_change_auton_led_state)

    seconds_to_wait = 1

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

            if green_counter_s == 2:
                green_counter_s = 0

            if green_counter_s == 0:
                ser.write(b"g")
            elif green_counter_s == 1:
                ser.write(b"o")

            time.sleep(seconds_to_wait)
            green_counter_s += seconds_to_wait


if __name__ == "__main__":
    main()
