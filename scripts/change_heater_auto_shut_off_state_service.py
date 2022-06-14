import rospy
from mrover.src.science.sciencecomms import send_msg
from mrover.srv import (ChangeDeviceState, ChangeDeviceStateRequest,
                        ChangeDeviceStateResponse)


def heater_auto_shut_off_transmit(enable: bool) -> None:
    """Transmits a heater auto shut off command over uart"""
    tx_msg = f"$AUTOSHUTOFF,{enable}"
    send_msg(tx_msg)


def handle_change_heater_auto_shut_off_state(
        req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
    """Handle/callback for changing heater auto shut off state service"""
    heater_auto_shut_off_transmit(req.enable)
    return ChangeDeviceStateResponse(True)


def change_heater_auto_shut_off_state_server() -> None:
    """Starts the server for change heater auto shut off state service"""
    rospy.init_node('change_heater_auto_shut_off_state_server')
    service = rospy.Service('change_heater_auto_shut_off_state',
                            ChangeDeviceState,
                            handle_change_heater_auto_shut_off_state)
    service.spin()


if __name__ == "__main__":
    change_heater_auto_shut_off_state_server()
