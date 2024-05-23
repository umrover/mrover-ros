import rospy
from backend.input import DeviceInputs, filter_input, simulated_axis
from backend.mappings import KeyboardButton
from mrover.msg import Throttle

rospy.init_node("teleoperation", disable_signals=True)

throttle_publisher = rospy.Publisher("/cache_throttle_cmd", Throttle, queue_size=1)

SCALE = 1.0


def send_cache_controls(mode: str, keyboard: DeviceInputs) -> None:
    if mode == "disabled":
        return

    throttle = filter_input(
        simulated_axis(keyboard.buttons, KeyboardButton.D, KeyboardButton.A),
        scale=SCALE,
    )
    throttle_publisher.publish(Throttle(["cache_motor"], [throttle]))
