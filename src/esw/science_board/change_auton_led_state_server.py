import rospy
from mrover.srv import (ChangeAutonLEDState, ChangeAutonLEDStateRequest,
                        ChangeAutonLEDStateResponse)
from sciencecomms import led_map, send_msg


def auton_led_transmit(color: str) -> bool:
    """Sends an auton LED command message via UART"""
    try:
        requested_state = led_map[color]
    except KeyError:
        # Done if an invalid color is sent
        return False

    msg = f"$LED,{requested_state.value}"
    success = send_msg(msg)
    return success


def handle_change_auton_led_state(
        req: ChangeAutonLEDStateRequest) -> ChangeAutonLEDStateResponse:
    """Handle/callback for changing auton led state service"""
    success = auton_led_transmit(req.color)
    return ChangeAutonLEDStateResponse(success)


def change_auton_led_state_server() -> None:
    """Starts the server for change auton led state service"""
    rospy.init_node('change_auton_led_state_server')
    service = rospy.Service('change_auton_led_state', ChangeAutonLEDState,
                            handle_change_auton_led_state)
    service.spin()


if __name__ == "__main__":
    change_auton_led_state_server()
