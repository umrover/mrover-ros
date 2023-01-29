import rospy
import serial
import time

from mrover.srv import (
    ChangeAutonLEDState,
    ChangeAutonLEDStateRequest,
    ChangeAutonLEDStateResponse,
)

ser = serial.Serial
previous_color = ""
green_counter_ms = 0


def handle_change_auton_led_state(req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
    """Processes a request to change the auton LED array state by issuing
    the command to the Arduino via USB. Returns the success of the
    transaction.
    :param req: A string that is the color of the requested state of the
        auton LED array. Note that green actually means blinking green.
    :returns: A boolean that is the success of sent USB transaction.
    """

    global previous_color, green_counter_ms

    color = req.color.lower()

    if previous_color != color:
        # send message
        if color == "red":
            ser.write(b"r")
        elif color == "green":
            green_counter_ms = 0
            ser.write(b"g")
        elif color == "blue":
            ser.write(b"b")
        else:
            ser.write(b"o")

        previous_color = color

    return ChangeAutonLEDStateResponse(True)


def main():
    rospy.init_node("science")

    global ser, previous_color, green_counter_ms

    # read serial connection info from parameter server
    port = rospy.get_param("auton_led_driver/port")
    baud = rospy.get_param("auton_led_driver/baud")

    # create serial connection with Arduino
    ser = serial.Serial(port, baud)

    rospy.Service("change_auton_led_state", ChangeAutonLEDState, handle_change_auton_led_state)

    ms_to_wait = 25  # needs to be a factor of 1000

    while not rospy.is_shutdown():
        if previous_color == "red":
            ser.write(b"r")
        elif previous_color == "green":
            ser.write(b"g")
            if green_counter_ms == 2000:
                green_counter_ms = 0

            if green_counter_ms == 0:
                ser.write(b"g")
            elif green_counter_ms == 1000:
                ser.write(b"o")

            time.sleep(ms_to_wait)
            green_counter_ms += ms_to_wait

        elif previous_color == "blue":
            ser.write(b"b")
        else:
            ser.write(b"o")


if __name__ == "__main__":
    main()
