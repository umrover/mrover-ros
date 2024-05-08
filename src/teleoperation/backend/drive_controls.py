import rospy
from backend.input import Inputs, filter_input, simulated_axis
from backend.mappings import JoystickAxis, ControllerAxis
from geometry_msgs.msg import Twist, Vector3

rospy.init_node("teleoperation", disable_signals=True)

twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

JOYSTICK_MICRO_LINEAR = 0.1
JOYSTICK_MICRO_ANGULAR = 0.1
CONTROLLER_LINEAR = 0.1
CONTROLLER_ANGULAR = 0.1

MAX_LINEAR_SPEED = rospy.get_param("rover/max_speed")
WHEEL_RADIUS = rospy.get_param("wheel/radius")
MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / WHEEL_RADIUS

JOYSTICK_LINEAR = MAX_LINEAR_SPEED
JOYSTICK_ANGULAR = MAX_ANGULAR_SPEED


def compute_drive_controls(inputs: Inputs) -> None:
    joystick_axes = inputs.joystick.axes
    controller_axes = inputs.controller.axes

    joystick_linear = filter_input(
        joystick_axes[JoystickAxis.FORWARD_BACK.value], quadratic=True, scale=JOYSTICK_LINEAR
    )
    joystick_angular = filter_input(
        joystick_axes[JoystickAxis.LEFT_RIGHT.value], quadratic=True, scale=JOYSTICK_ANGULAR
    )

    joystick_micro_linear = filter_input(
        joystick_axes[JoystickAxis.MICRO_FORWARD_BACK.value], scale=JOYSTICK_MICRO_LINEAR
    )
    joystick_micro_angular = filter_input(
        joystick_axes[JoystickAxis.MICRO_LEFT_RIGHT.value], scale=JOYSTICK_MICRO_ANGULAR
    )

    controller_linear = filter_input(
        simulated_axis(controller_axes, ControllerAxis.DPAD_UP.value, ControllerAxis.DPAD_DOWN.value),
        scale=CONTROLLER_LINEAR,
    )
    controller_angular = filter_input(
        simulated_axis(controller_axes, ControllerAxis.DPAD_RIGHT.value, ControllerAxis.DPAD_LEFT.value),
        scale=CONTROLLER_ANGULAR,
    )

    linear = joystick_linear + joystick_micro_linear + controller_linear
    angular = joystick_angular + joystick_micro_angular + controller_angular

    twist_publisher.publish(
        Twist(
            linear=Vector3(x=linear),
            angular=Vector3(z=angular),
        )
    )
