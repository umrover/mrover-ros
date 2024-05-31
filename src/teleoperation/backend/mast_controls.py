import rospy
from backend.input import DeviceInputs, filter_input, simulated_axis
from backend.mappings import KeyboardButton
from mrover.msg import Throttle

rospy.init_node("teleoperation", disable_signals=True)

throttle_publisher = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)

Y_SCALE = -1.0
Z_SCALE = 1.0


def send_mast_controls(keyboard: DeviceInputs) -> None:
    buttons = keyboard.buttons

    controller_y = filter_input(
        simulated_axis(buttons, KeyboardButton.W, KeyboardButton.S),
        scale=Y_SCALE,
    )
    controller_z = filter_input(
        simulated_axis(buttons, KeyboardButton.D, KeyboardButton.A),
        scale=Z_SCALE,
    )
    throttle_publisher.publish(Throttle(["mast_gimbal_y", "mast_gimbal_z"], [controller_y, controller_z]))
