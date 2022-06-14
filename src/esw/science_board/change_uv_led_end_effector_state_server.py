import rospy
from mrover.srv import (ChangeDeviceState, ChangeDeviceStateRequest,
                        ChangeDeviceStateResponse)
from sciencecomms import send_mosfet_msg


def handle_change_uv_led_end_effector_state(
        req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
    """Handle/callback for changing uv led end effector state service"""
    success = send_mosfet_msg("uv_led_end_effector", req.enable)
    return ChangeDeviceStateResponse(success)


def change_uv_led_end_effector_state_server() -> None:
    """Starts the server for change uv led end effector state service"""
    rospy.init_node('change_uv_led_end_effector_state_server')
    service = rospy.Service('change_uv_led_end_effector_state',
                            ChangeDeviceState,
                            handle_change_uv_led_end_effector_state)
    service.spin()


if __name__ == "__main__":
    change_uv_led_end_effector_state_server()
