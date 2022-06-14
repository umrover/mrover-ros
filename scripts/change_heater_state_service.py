import rospy
from mrover.src.science.sciencecomms import (format_mosfet_msg, mosfet_dev_map,
                                             send_msg)
from mrover.srv import (ChangeHeaterState, ChangeHeaterStateRequest,
                        ChangeHeaterStateResponse)

heater_map = {
    0: "heater_0",
    1: "heater_1",
    2: "heater_2"
}


def heater_transmit(device: int, enable: bool) -> None:
    """Sends a heater state command message via UART"""
    heater_device_string = heater_map[device]
    translated_device = mosfet_dev_map[heater_device_string]
    tx_msg = format_mosfet_msg(translated_device, int(enable))
    send_msg(tx_msg)


def handle_change_heater_state(
        req: ChangeHeaterStateRequest) -> ChangeHeaterStateResponse:
    """Handle/callback for changing auton led state service"""
    heater_transmit(req.device, req.color)
    return ChangeHeaterStateResponse(True)


def change_heater_state_server() -> None:
    """Starts the server for change heater state service"""
    rospy.init_node('change_heater_state_server')
    service = rospy.Service('change_heater_state', ChangeHeaterState,
                            handle_change_heater_state)
    service.spin()


if __name__ == "__main__":
    change_heater_state_server()
