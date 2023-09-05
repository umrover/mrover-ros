#!/usr/bin/env python3
# Node for teleop-related callback functions

from math import copysign, nan
import typing
from threading import Lock
import rospy as ros
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from mrover.msg import MotorsStatus
from mrover.srv import ChangeArmMode, ChangeArmModeRequest, ChangeArmModeResponse
from typing import List


DEFAULT_ARM_DEADZONE = 0.15

# Whether or not to publish /joint_states feedback for RA MoveIt/Rviz
PUBLISH_JOINT_STATES = False


def quadratic(val: float) -> float:
    return copysign(val**2, val)


# If below threshold, make output zero
def deadzone(magnitude: float, threshold: float) -> float:
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold) / (1 - threshold)

    return copysign(temp_mag, magnitude)


class Drive:
    def __init__(self):
        self.joystick_mappings = ros.get_param("teleop/joystick_mappings")
        self.drive_config = ros.get_param("teleop/drive_controls")

        # Constants for diff drive
        self.max_wheel_speed = ros.get_param("rover/max_speed")
        wheel_radius = ros.get_param("wheel/radius")
        self.max_angular_speed = self.max_wheel_speed / wheel_radius
        self.twist_pub = ros.Publisher("/cmd_vel", Twist, queue_size=100)

    def teleop_drive_callback(self, msg: Joy):
        joints: typing.Dict[str, JointState] = {}

        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg.axes[self.joystick_mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(
            msg.axes[self.joystick_mappings["forward_back"]] * self.drive_config["forward_back"]["multiplier"], 0.05
        )

        # Convert from [0,1] to [0, max_wheel_speed] and apply dampen
        linear *= self.max_wheel_speed * dampen

        # Deadzones for each axis
        left_right = (
            deadzone(
                msg.axes[self.joystick_mappings["left_right"]] * self.drive_config["left_right"]["multiplier"], 0.4
            )
            if self.drive_config["left_right"]["enabled"]
            else 0
        )
        twist = quadratic(
            deadzone(msg.axes[self.joystick_mappings["twist"]] * self.drive_config["twist"]["multiplier"], 0.1)
        )

        angular = twist + left_right

        # Same as linear but for angular speed
        angular *= self.max_angular_speed * dampen
        # Clamp if both twist and left_right are used at the same time
        if abs(angular) > self.max_angular_speed:
            angular = copysign(self.max_angular_speed, angular)
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        self.twist_pub.publish(twist_msg)


# TODO: This class has alot of duplicate logic for RA and SA, should be refactored
class ArmControl:
    def __init__(self):
        self.xbox_mappings = ros.get_param("teleop/xbox_mappings")
        self.ra_config = ros.get_param("teleop/ra_controls")
        self.sa_config = ros.get_param("teleop/sa_controls")

        self._arm_mode = "arm_disabled"
        self._arm_mode_lock = Lock()
        self._sa_arm_mode = "arm_disabled"
        self._sa_arm_mode_lock = Lock()

        self.ra_cmd_pub = ros.Publisher("ra_cmd", JointState, queue_size=100)
        self.sa_cmd_pub = ros.Publisher("sa_cmd", JointState, queue_size=100)
        self.joint_state_pub = ros.Publisher("joint_states", JointState, queue_size=100)

        self.ra_slow_mode = False

        self.RA_NAMES = [
            "joint_a",
            "joint_b",
            "joint_c",
            "joint_d",
            "joint_e",
            "joint_f",
            "finger",
            "gripper",
        ]
        self.SA_NAMES = ["sa_joint_1", "sa_joint_2", "sa_joint_3", "scoop", "microscope"]

        self.ra_cmd = JointState(
            name=[name for name in self.RA_NAMES],
            position=[nan for _ in self.RA_NAMES],
            velocity=[0.0 for _ in self.RA_NAMES],
            effort=[nan for _ in self.RA_NAMES],
        )

        self.sa_cmd = JointState(
            name=[name for name in self.SA_NAMES],
            position=[nan for _ in self.SA_NAMES],
            velocity=[0.0 for _ in self.SA_NAMES],
            effort=[nan for _ in self.SA_NAMES],
        )

        self._joint_state_lock = Lock()
        self.current_ra_joint_states = JointState(
            name=[name for name in self.RA_NAMES],
            position=[nan for _ in self.RA_NAMES],
            velocity=[0.0 for _ in self.RA_NAMES],
            effort=[nan for _ in self.RA_NAMES],
        )

    # Service handler for RA arm control mode
    def ra_mode_service(self, req: ChangeArmModeRequest) -> ChangeArmModeResponse:
        """
        Service handler for arm control mode
        :param req: ChangeArmMode service request name in {arm_disabled, open_loop, servo}
        :return: ChangeArmMode service response
        """
        with self._arm_mode_lock:
            self._arm_mode = req.mode
            if self._arm_mode == "arm_disabled":
                self.send_ra_stop()
        return ChangeArmModeResponse(success=True)

    def send_ra_stop(self) -> None:
        """
        Sends a stop command to the Robotic Arm
        :return:
        """
        self.ra_cmd.position = [nan for _ in self.RA_NAMES]
        self.ra_cmd.velocity = [0.0 for _ in self.RA_NAMES]
        self.ra_cmd.effort = [nan for _ in self.RA_NAMES]
        self.ra_cmd_pub.publish(self.ra_cmd)

    def brushless_encoder_callback(self, msg: MotorsStatus) -> None:
        """
        Callback for brushless encoder values
        :param msg: MotorsStatus message representing the current status of the brushless motors
        :return:
        """
        with self._joint_state_lock:
            for i, name in enumerate(msg.name):
                index_of_joint = self.current_ra_joint_states.name.index(name)
                self.current_ra_joint_states.position[index_of_joint] = msg.joint_states.position[i]
                self.current_ra_joint_states.velocity[index_of_joint] = msg.joint_states.velocity[i]
                self.current_ra_joint_states.effort[index_of_joint] = msg.joint_states.effort[i]

    def brushed_encoder_callback(self, msg: JointState) -> None:
        """
        Callback for brushed encoder values
        :param msg: JointState message representing the current joint states of the brushed motors
        :return:
        """
        with self._joint_state_lock:
            for i, name in enumerate(msg.name):
                index_of_joint = self.current_ra_joint_states.name.index(name)
                self.current_ra_joint_states.position[index_of_joint] = msg.position[i]
                self.current_ra_joint_states.velocity[index_of_joint] = msg.velocity[i]
                self.current_ra_joint_states.effort[index_of_joint] = msg.effort[i]

    def publish_joint_states(self) -> None:
        """
        Publishes the current joint states to the /joint_states topic
        :return:
        """
        with self._joint_state_lock:
            self.joint_state_pub.publish(self.current_ra_joint_states)

    def filter_xbox_axis(
        self,
        axes_array: "List[float]",
        axis_name: str,
        deadzone_threshold: float = DEFAULT_ARM_DEADZONE,
        quad_control: bool = True,
    ) -> float:
        """
        Applies various filtering functions to an axis for controlling the arm
        :param axes_array: Axis array from sensor_msgs/Joy, each value is a float from [-1,1]
        :param axis_name: String representing the axis you are controlling, should match teleop.yaml
        :param deadzone_threshold: Float representing the deadzone of the axis that you would like to use
        :param quad_control: Bool for whether or not we want the axis to follow an x^2 curve instead of a linear one
        Velocities are sent in range [-1,1]
        :return: Returns an output velocity value for the given joint using the given axis_name
        """
        deadzoned_val = deadzone(axes_array[self.xbox_mappings[axis_name]], deadzone_threshold)
        return quadratic(deadzoned_val) if quad_control else deadzoned_val

    def filter_xbox_button(self, button_array: "List[int]", pos_button: str, neg_button: str) -> int:
        """
        Applies various filtering functions to an axis for controlling the arm
        :param button_array: Button array from sensor_msgs/Joy, each value is an int 0 or 1
        :param pos_button: String representing the positive button for controlling a joint
        :param neg_button: String representing the negtaive button for controlling a joint
        :return: Return -1, 0, or 1 depending on what buttons are being pressed
        """
        return button_array[self.xbox_mappings[pos_button]] - button_array[self.xbox_mappings[neg_button]]

    def ra_control_callback(self, msg: Joy) -> None:
        """
        Chooses which RA control function to use based on _arm_mode string
        :param msg: Has axis and buttons array for Xbox controller
        Velocities are sent in range [-1,1]
        :return:
        """
        if self._arm_mode == "open_loop":
            self.ra_open_loop_control(msg)
        elif self._arm_mode == "servo":
            self.ra_servo_control(msg)

    def ra_open_loop_control(self, msg: Joy) -> None:
        """
        Converts a Joy message with the Xbox inputs
        to a JointState to control the RA Arm in open loop
        :param msg: Has axis and buttons array for Xbox controller
        Velocities are sent in range [-1,1]
        :return:
        """

        d_pad_x = msg.axes[self.xbox_mappings["d_pad_x"]]
        if d_pad_x > 0.5:
            self.ra_slow_mode = True
        elif d_pad_x < -0.5:
            self.ra_slow_mode = False

        # Filter for xbox triggers, they are typically [-1,1]
        # Lose [-1,0] range since when joystick is initially plugged in
        # these output 0 instead of -1 when up
        raw_left_trigger = msg.axes[self.xbox_mappings["left_trigger"]]
        left_trigger = raw_left_trigger if raw_left_trigger > 0 else 0
        raw_right_trigger = msg.axes[self.xbox_mappings["right_trigger"]]
        right_trigger = raw_right_trigger if raw_right_trigger > 0 else 0
        self.ra_cmd.velocity = [
            self.ra_config["joint_a"]["multiplier"] * self.filter_xbox_axis(msg.axes, "left_js_x"),
            self.ra_config["joint_b"]["multiplier"] * self.filter_xbox_axis(msg.axes, "left_js_y"),
            self.ra_config["joint_c"]["multiplier"] * self.filter_xbox_axis(msg.axes, "right_js_y"),
            self.ra_config["joint_d"]["multiplier"] * self.filter_xbox_axis(msg.axes, "right_js_x"),
            self.ra_config["joint_e"]["multiplier"] * (right_trigger - left_trigger),
            self.ra_config["joint_f"]["multiplier"]
            * self.filter_xbox_button(msg.buttons, "right_bumper", "left_bumper"),
            self.ra_config["finger"]["multiplier"] * self.filter_xbox_button(msg.buttons, "y", "a"),
            self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(msg.buttons, "b", "x"),
        ]

        for i, name in enumerate(self.RA_NAMES):
            if self.ra_slow_mode:
                self.ra_cmd.velocity[i] *= self.ra_config[name]["slow_mode_multiplier"]
            if self.ra_config[name]["invert"]:
                self.ra_cmd.velocity[i] *= -1

        self.ra_cmd_pub.publish(self.ra_cmd)

    def ra_servo_control(self, msg: Joy) -> None:
        """
        Converts a Joy message with the Xbox inputs
        to a JointState to control the RA Arm with Moveit servo control
        :param msg: Has axis and buttons array for Xbox controller
        Velocities are sent in range [-1,1]
        :return:
        """
        # TODO: Write this function if/when we get moveit_servo working
        return

    def sa_mode_service(self, req: ChangeArmModeRequest) -> ChangeArmModeResponse:
        """
        Service handler for arm control mode
        :param req: ChangeArmMode service request mode in {"sa_disabled", "sa_enabled"}
        :return: ChangeArmMode service response
        """
        with self._sa_arm_mode_lock:
            self._sa_arm_mode = req.mode
            if self._sa_arm_mode == "sa_disabled":
                self.send_sa_stop()
        return ChangeArmModeResponse(success=True)

    def sa_control_callback(self, msg: Joy) -> None:
        """
        Converts a Joy message with the Xbox inputs
        to a JointState to control the SA Arm in open loop
        :param msg: Has axis and buttons array for Xbox controller
        Velocities are sent in range [-1,1]
        :return:
        """
        with self._sa_arm_mode_lock:
            if self._sa_arm_mode == "open_loop":
                # Filter for xbox triggers, they are typically [-1,1]
                raw_left_trigger = msg.axes[self.xbox_mappings["left_trigger"]]
                left_trigger = raw_left_trigger if raw_left_trigger > 0 else 0
                raw_right_trigger = msg.axes[self.xbox_mappings["right_trigger"]]
                right_trigger = raw_right_trigger if raw_right_trigger > 0 else 0

                self.sa_cmd.velocity = [
                    self.sa_config["sa_joint_1"]["multiplier"]
                    * self.filter_xbox_axis(msg.axes, "left_js_x", 0.15, True),
                    self.sa_config["sa_joint_2"]["multiplier"]
                    * self.filter_xbox_axis(msg.axes, "left_js_y", 0.15, True),
                    self.sa_config["sa_joint_3"]["multiplier"]
                    * self.filter_xbox_axis(msg.axes, "right_js_y", 0.15, True),
                    self.sa_config["scoop"]["multiplier"] * (right_trigger - left_trigger),
                    self.sa_config["microscope"]["multiplier"]
                    * self.filter_xbox_button(msg.buttons, "right_bumper", "left_bumper"),
                ]

                fast_mode_activated = msg.buttons[self.xbox_mappings["a"]] or msg.buttons[self.xbox_mappings["b"]]
                if not fast_mode_activated:
                    for i, name in enumerate(self.SA_NAMES):
                        # When going up (vel > 0) with SA joint 2, we DON'T want slow mode.
                        if not (name == "sa_joint_2" and self.sa_cmd.velocity[i] > 0):
                            self.sa_cmd.velocity[i] *= self.sa_config[name]["slow_mode_multiplier"]

                self.sa_cmd_pub.publish(self.sa_cmd)

    def send_sa_stop(self) -> None:
        """
        Send a stop command to the SA arm
        :return:
        """
        self.sa_cmd.position = [nan for _ in self.SA_NAMES]
        self.sa_cmd.velocity = [0.0 for _ in self.SA_NAMES]
        self.sa_cmd.effort = [nan for _ in self.SA_NAMES]
        self.sa_cmd_pub.publish(self.sa_cmd)


def main():
    ros.init_node("teleop")

    arm = ArmControl()
    drive = Drive()

    ros.Subscriber("joystick", Joy, drive.teleop_drive_callback)
    ros.Subscriber("xbox/ra_control", Joy, arm.ra_control_callback)
    ros.Subscriber("xbox/sa_control", Joy, arm.sa_control_callback)
    ros.Subscriber("brushless_ra_data", MotorsStatus, arm.brushless_encoder_callback)
    ros.Subscriber("brushed_ra_data", JointState, arm.brushed_encoder_callback)

    # Arm Mode Services
    ros.Service("change_ra_mode", ChangeArmMode, arm.ra_mode_service)
    ros.Service("change_sa_mode", ChangeArmMode, arm.sa_mode_service)

    # Publish joint states for Moveit at 10Hz
    if PUBLISH_JOINT_STATES:
        while not ros.is_shutdown():
            arm.publish_joint_states()
            ros.sleep(0.1)

    ros.spin()


if __name__ == "__main__":
    main()
