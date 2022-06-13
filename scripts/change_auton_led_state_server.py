#!/usr/bin/env python

from __future__ import print_function

from mrover.srv import ChangeAutonLEDStateRequest, ChangeAutonLEDState, ChangeAutonLEDStateResponse
from mrover.src.science.sciencecomms import msg_send
import rospy

led_map = {
    "Red": 0,
    "Blue": 1,
    "Green": 2,
    "Off": 3
}

def auton_led_transmit(self, color: str) -> None:
        try:
            requested_state = led_map[color]
            print("Received new auton led request: Turning " + color)
        except KeyError:
            requested_state = led_map["Off"]
            print("Received invalid/off auton led request: Turning off all colors")

        msg = "$LED,{led_color}".format(led_color=requested_state.value)
        msg_send(msg)

def handle_change_auton_led_state(req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
    auton_led_transmit(req.color)
    return ChangeAutonLEDStateResponse(True)

def change_auton_led_state_server() -> None:
    rospy.init_node('change_auton_led_state_server')
    s = rospy.Service('change_auton_led_state', ChangeAutonLEDState, handle_change_auton_led_state)
    rospy.spin()

if __name__ == "__main__":
    change_auton_led_state_server()
