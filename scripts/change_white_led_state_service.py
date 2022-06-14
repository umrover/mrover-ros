import rospy
from mrover.src.science.sciencecomms import send_mosfet_msg
from mrover.srv import (ChangeDeviceState, ChangeDeviceStateRequest,
                        ChangeDeviceStateResponse)


def handle_change_white_led_state(
        req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
    """Handle/callback for changing white led state service"""
    send_mosfet_msg("white_led", req.enable)
    return ChangeDeviceStateResponse(True)


def change_white_led_state_server() -> None:
    """Starts the server for change white led state service"""
    rospy.init_node('change_white_led_state_server')
    service = rospy.Service('change_white_led_state', ChangeDeviceState,
                            handle_change_white_led_state)
    service.spin()


if __name__ == "__main__":
    change_white_led_state_server()
