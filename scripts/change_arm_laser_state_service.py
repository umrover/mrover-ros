import rospy
from mrover.src.science.sciencecomms import send_mosfet_msg
from mrover.srv import (ChangeDeviceState, ChangeDeviceStateRequest,
                        ChangeDeviceStateResponse)


def handle_change_arm_laser_state(
        req: ChangeDeviceStateRequest) -> ChangeDeviceStateResponse:
    """Handle/callback for changing arm laser state service"""
    send_mosfet_msg("arm_laser", req.enable)
    return ChangeDeviceStateResponse(True)


def change_arm_laser_state_server() -> None:
    """Starts the server for change arm laser state service"""
    rospy.init_node('change_arm_laser_state_server')
    service = rospy.Service('change_arm_laser_state', ChangeDeviceState,
                            handle_change_arm_laser_state)
    service.spin()


if __name__ == "__main__":
    change_arm_laser_state_server()
