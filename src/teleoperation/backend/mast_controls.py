import rospy
from backend.input import Inputs, filter_input, simulated_axis
from backend.mappings import KeyboardButton
from mrover.msg import Throttle

rospy.init_node("teleoperation", disable_signals=True)

throttle_publisher = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)

Y_SCALE = 0.5
Z_SCALE = 1.0


def compute_mast_controls(inputs: Inputs) -> None:
    buttons = inputs.joystick.buttons

    controller_y = filter_input(
        simulated_axis(buttons, KeyboardButton.W.value, KeyboardButton.S.value),
        scale=Y_SCALE,
    )
    controller_z = filter_input(
        simulated_axis(buttons, KeyboardButton.D.value, KeyboardButton.S.value),
        scale=Z_SCALE,
    )
    throttle_publisher.publish(Throttle(["mast_gimbal_y", "mast_gimbal_z"], [controller_y, controller_z]))
