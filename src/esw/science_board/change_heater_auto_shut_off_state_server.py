import rospy
from mrover.srv import (ChangeDeviceState, ChangeDeviceStateRequest,
                        ChangeDeviceStateResponse)
from sciencecomms import send_msg


def heater_auto_shut_off_transmit(enable: bool) -> bool:
    """Transmits a heater auto shut off command over uart"""
    tx_msg = f"$AUTOSHUTOFF,{enable}"
    success = send_msg(tx_msg)
    return success


def handle_change_heater_auto_shut_off_state(
        req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
    """Handle/callback for changing heater auto shut off state service"""
    success = heater_auto_shut_off_transmit(req.enable)
    return ChangeDeviceStateResponse(success)


def change_heater_auto_shut_off_state_server() -> None:
    """Starts the server for change heater auto shut off state service"""
    rospy.init_node('change_heater_auto_shut_off_state_server')
    service = rospy.Service('change_heater_auto_shut_off_state',
                            ChangeDeviceState,
                            handle_change_heater_auto_shut_off_state)
    service.spin()


if __name__ == "__main__":
    change_heater_auto_shut_off_state_server()
