import rospy
from mrover.srv import (ChangeServoAngles, ChangeServoAnglesRequest,
                        ChangeServoAnglesResponse)
from sciencecomms import send_msg


def servo_transmit(angle_0: float, angle_1: float, angle_2: float) -> bool:
    """Transmits a servo angle command message"""
    tx_msg = f"$SERVO,{angle_0},{angle_1},{angle_2}"
    success = send_msg(tx_msg)
    return success


def handle_change_servo_angles(
        req: ChangeServoAnglesRequest) -> ChangeServoAnglesResponse:
    """Handle/callback for changing servo angles service"""
    success = servo_transmit(req.angle_0, req.angle_1, req.angle_2)
    return ChangeServoAnglesResponse(success)


def change_servo_angles_server() -> None:
    """Starts the server for change servo angles service"""
    rospy.init_node('change_servo_angles_server')
    service = rospy.Service('change_servo_angles', ChangeServoAngles,
                            handle_change_servo_angles)
    service.spin()


if __name__ == "__main__":
    change_servo_angles_server()
