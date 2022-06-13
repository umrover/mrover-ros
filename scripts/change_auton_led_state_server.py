"""Used for printing"""
from __future__ import print_function

import rospy
from mrover.src.science.sciencecomms import send_msg
from mrover.srv import (ChangeAutonLEDState, ChangeAutonLEDStateRequest,
                        ChangeAutonLEDStateResponse)

led_map = {
    "Red": 0,
    "Blue": 1,
    "Green": 2,
    "Off": 3
}


def auton_led_transmit(color: str) -> None:
    """Sends an auton LED command message via UART"""
    try:
        requested_state = led_map[color]
        print("Received new auton led request: Turning " + color)
    except KeyError:
        requested_state = led_map["Off"]
        print("Received invalid/off auton led request: Turning off all colors")

    msg = f"$LED,{requested_state.value}"
    send_msg(msg)


def handle_change_auton_led_state(req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
    """Handle/callback for changing auton led state service"""
    auton_led_transmit(req.color)
    return ChangeAutonLEDStateResponse(True)


def change_auton_led_state_server() -> None:
    """Starts the server for change auton led state service"""
    rospy.init_node('change_auton_led_state_server')
    service = rospy.Service('change_auton_led_state', ChangeAutonLEDState,
                            handle_change_auton_led_state)
    service.spin()


if __name__ == "__main__":
    change_auton_led_state_server()
