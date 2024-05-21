import rospy
from backend.input import filter_input, simulated_axis, safe_index, remap, DeviceInputs
from backend.mappings import JoystickAxis, ControllerButton
from geometry_msgs.msg import Twist, Vector3

rospy.init_node("teleoperation", disable_signals=True)

joystick_twist_publisher = rospy.Publisher("/joystick_cmd_vel", Twist, queue_size=1)
controller_twist_publisher = rospy.Publisher("/controller_cmd_vel", Twist, queue_size=1)

JOYSTICK_MICRO_LINEAR = -0.05
JOYSTICK_MICRO_ANGULAR = -0.1
CONTROLLER_LINEAR = 0.05
CONTROLLER_ANGULAR = -0.1

MAX_LINEAR_SPEED = rospy.get_param("rover/max_speed")
WHEEL_RADIUS = rospy.get_param("wheel/radius")
MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / WHEEL_RADIUS

JOYSTICK_LINEAR = -MAX_LINEAR_SPEED
JOYSTICK_ANGULAR = -MAX_ANGULAR_SPEED * 0.3

JOYSTICK_DEADZONE = 0.02


def send_joystick_twist(joystick: DeviceInputs) -> None:
    axes = joystick.axes

    throttle = remap(-safe_index(axes, JoystickAxis.THROTTLE), -1, 1, 0, 1)

    joystick_linear = filter_input(
        safe_index(axes, JoystickAxis.FORWARD_BACK),
        quadratic=True,
        scale=JOYSTICK_LINEAR * throttle,
        deadzone=JOYSTICK_DEADZONE,
    )
    joystick_angular = filter_input(
        safe_index(axes, JoystickAxis.TWIST),
        quadratic=True,
        scale=JOYSTICK_ANGULAR * throttle,
        deadzone=JOYSTICK_DEADZONE,
    )

    joystick_micro_linear = filter_input(safe_index(axes, JoystickAxis.MICRO_FORWARD_BACK), scale=JOYSTICK_MICRO_LINEAR)
    joystick_micro_angular = filter_input(safe_index(axes, JoystickAxis.MICRO_LEFT_RIGHT), scale=JOYSTICK_MICRO_ANGULAR)

    linear = joystick_linear + joystick_micro_linear
    angular = joystick_angular + joystick_micro_angular

    joystick_twist_publisher.publish(
        Twist(
            linear=Vector3(x=linear),
            angular=Vector3(z=angular),
        )
    )


def send_controller_twist(controller: DeviceInputs) -> None:
    buttons = controller.buttons

    controller_linear = filter_input(
        simulated_axis(buttons, ControllerButton.DPAD_UP, ControllerButton.DPAD_DOWN),
        scale=CONTROLLER_LINEAR,
    )
    controller_angular = filter_input(
        simulated_axis(buttons, ControllerButton.DPAD_RIGHT, ControllerButton.DPAD_LEFT),
        scale=CONTROLLER_ANGULAR,
    )
    controller_twist_publisher.publish(
        Twist(
            linear=Vector3(x=controller_linear),
            # angular=Vector3(z=controller_angular),
        )
    )
