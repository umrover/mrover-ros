import json
from math import nan
from channels.generic.websocket import JsonWebsocketConsumer
import rospy
from std_srvs.srv import SetBool, Trigger
from mrover.msg import PDLB, ControllerState, Calibrated
from mrover.srv import EnableDevice, AdjustMotor
from sensor_msgs.msg import JointState, Joy, NavSatFix
from geometry_msgs.msg import Twist
from math import copysign
import typing

import tf2_ros
from mrover.msg import PDLB, ControllerState, GPSWaypoint, LED, StateMachineStateUpdate, Throttle, Velocity, Position
from mrover.srv import EnableAuton
from std_msgs.msg import String, Bool
from util.SE3 import SE3


DEFAULT_ARM_DEADZONE = 0.15

# If below threshold, make output zero
def deadzone(magnitude: float, threshold: float) -> float:
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold) / (1 - threshold)
    return copysign(temp_mag, magnitude)


def quadratic(val: float) -> float:
    return copysign(val**2, val)


class GUIConsumer(JsonWebsocketConsumer):
    def connect(self):
        self.accept()
        # Publishers
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=1)
        self.teleop_pub = rospy.Publisher("/teleop_enabled", Bool, queue_size=1)
        self.mast_gimbal_pub = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)
        self.arm_throttle_cmd_pub = rospy.Publisher("arm_throttle_cmd", Throttle, queue_size=1)
        self.arm_velocity_cmd_pub = rospy.Publisher("arm_velocity_cmd", Velocity, queue_size=1)
        self.arm_position_cmd_pub = rospy.Publisher("arm_position_cmd", Position, queue_size=1)

        # Subscribers
        self.pdb_sub = rospy.Subscriber("/pdb_data", PDLB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber("/arm_controller_data", ControllerState, self.arm_controller_callback)
        self.drive_moteus_sub = rospy.Subscriber(
            "/drive_controller_data", ControllerState, self.drive_controller_callback
        )
        self.gps_fix = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_fix_callback)
        self.joint_state_sub = rospy.Subscriber("/drive_joint_data", JointState, self.joint_state_callback)
        self.led_sub = rospy.Subscriber("/led", LED, self.led_callback)
        self.nav_state_sub = rospy.Subscriber("/nav_state", StateMachineStateUpdate, self.nav_state_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.enable_auton = rospy.ServiceProxy("enable_auton", EnableAuton)
        self.calibration_checkbox_sub = rospy.Subscriber(
            "/calibration_checkbox", Calibrated, self.calibration_checkbox_callback
        )
        self.joy_sub = rospy.Subscriber("/joystick", Joy, self.handle_joystick_message)
        self.arm_joystick_sub = rospy.Subscriber("/xbox/ra_control", Joy, self.handle_arm_message)
        # Services
        self.laser_service = rospy.ServiceProxy("enable_mosfet_device", SetBool)
        self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)
        self.arm_adjust_service = rospy.ServiceProxy("arm_adjust", AdjustMotor)

        # ROS Parameters
        self.mappings = rospy.get_param("teleop/joystick_mappings")
        self.drive_config = rospy.get_param("teleop/drive_controls")
        self.max_wheel_speed = rospy.get_param("rover/max_speed")
        self.wheel_radius = rospy.get_param("wheel/radius")
        self.max_angular_speed = self.max_wheel_speed / self.wheel_radius

        self.ra_config = rospy.get_param("teleop/ra_controls")
        self.brushless_motors = rospy.get_param("brushless_motors/controllers")
        self.brushed_motors = rospy.get_param("brushed_motors/controllers")
        self.xbox_mappings = rospy.get_param("teleop/xbox_mappings")
        self.sa_config = rospy.get_param("teleop/sa_controls")

    def disconnect(self, close_code):
        self.pdb_sub.unregister()
        self.arm_moteus_sub.unregister()
        self.drive_moteus_sub.unregister()
        self.joint_state_sub.unregister()
        self.gps_fix.unregister()
        self.led_sub.unregister()
        self.nav_state_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
        try:
            if message["type"] == "joystick_values":
                self.handle_joystick_message(message)
            elif message["type"] == "enable_decive_srv":
                self.enable_device_callback(message)
            elif message["type"] == "disable_auton_led":
                self.disable_auton_led()
            elif message["type"] == "laser_service":
                self.enable_laser_callback(message)
            elif message["type"] == "calibrate_motors":
                self.calibrate_motors(message)
            elif message["type"] == "calibrate_service":
                self.calibrate_motors_callback(message)
            elif message["type"] == "arm_adjust":
                self.arm_adjust(message)
            elif message["type"] == "change_ra_mode":
                self.handle_joystick_message(message)
            elif message["type"] == "arm_values":
                self.handle_arm_message(message)
            elif message["type"] == "auton_command":
                self.send_auton_command(message)
            elif message["type"] == "teleop_enabled":
                self.send_teleop_enabled(message)
            elif message["type"] == "auton_tfclient":
                self.auton_bearing()
            elif message["type"] == "mast_gimbal":
                self.mast_gimbal(message)
        except Exception as e:
            rospy.logerr(e)


    def filter_xbox_axis(
        self,
        value: int,
        deadzone_threshold: float = DEFAULT_ARM_DEADZONE,
        quad_control: bool = False,
    ) -> float:
        deadzoned_val = deadzone(value, deadzone_threshold)
        return quadratic(deadzoned_val) if quad_control else deadzoned_val

    def filter_xbox_button(self, button_array: "List[int]", pos_button: str, neg_button: str) -> float:
        """
        Applies various filtering functions to an axis for controlling the arm
        :return: Return -1, 0, or 1 depending on what buttons are being pressed
        """
        return button_array[self.xbox_mappings[pos_button]] - button_array[self.xbox_mappings[neg_button]]

    def to_velocity(self, input: int, joint_name: str, brushless: bool = True) -> float:
        """
        Scales [-1,1] joystick input to min/max of each joint
        """
        if brushless:
            return (self.brushless_motors[joint_name]["min_velocity"] 
                    + (input+1) * (self.brushless_motors[joint_name]["max_velocity"] 
                    - self.brushless_motors[joint_name]["min_velocity"]) / 2)
        else: 
            return (self.brushed_motors[joint_name]["min_velocity"] 
                    + (input+1) * (self.brushed_motors[joint_name]["max_velocity"] 
                    - self.brushed_motors[joint_name]["min_velocity"]) / 2)

    def handle_arm_message(self, msg):
        RA_NAMES = ["joint_a","joint_b","joint_c","joint_de_pitch","joint_de_yaw","allen_key","gripper"]
        ra_slow_mode = False
        if msg["arm_mode"] == "ik":
            x = 1
        elif msg["arm_mode"] == "position":
            arm_position_cmd = Position(
                names=RA_NAMES,
                positions=msg["positions"],
            )
            self.arm_position_cmd_pub.publish(arm_position_cmd)

        elif msg["arm_mode"] == "velocity":
            arm_velocity_cmd = Velocity()
            arm_velocity_cmd.names = RA_NAMES
            arm_velocity_cmd.velocities = [
                self.to_velocity(self.filter_xbox_axis(msg["axes"][self.ra_config["joint_a"]["xbox_index"]]), "joint_a") ,
                self.to_velocity(self.filter_xbox_axis(msg["axes"][self.ra_config["joint_b"]["xbox_index"]]), "joint_b", False),
                self.to_velocity(self.filter_xbox_axis(msg["axes"][self.ra_config["joint_c"]["xbox_index"]]), "joint_c"),
                self.to_velocity(self.filter_xbox_axis(msg["axes"][self.ra_config["joint_de_0"]["xbox_index"]]), "joint_de_0"),
                self.to_velocity(self.filter_xbox_axis(msg["axes"][self.ra_config["joint_de_1"]["xbox_index"]]), "joint_de_1"),
                self.ra_config["allen_key"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "y", "a"),
                self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "b", "x"),
            ]

            self.arm_velocity_cmd_pub.publish(arm_velocity_cmd)

        elif msg["arm_mode"] == "throttle":
            arm_throttle_cmd = Throttle()
            arm_throttle_cmd.names = RA_NAMES
            d_pad_x = msg["axes"][self.xbox_mappings["d_pad_x"]]
            if d_pad_x > 0.5:
                ra_slow_mode = True
            elif d_pad_x < -0.5:
                ra_slow_mode = False

            arm_throttle_cmd.throttles = [self.ra_config[name]["multiplier"] 
                                               * self.filter_xbox_axis(msg["axes"][info["xbox_index"]]) for name, info in self.ra_config.items() if name.startswith("joint")]
            arm_throttle_cmd.throttles.extend([
                self.ra_config["allen_key"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "y", "a"),
                self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "b", "x"),
            ])

            for i, name in enumerate(RA_NAMES):
                if ra_slow_mode:
                    arm_throttle_cmd.throttles[i] *= self.ra_config[name]["slow_mode_multiplier"]
                if self.ra_config[name]["invert"]:
                    arm_throttle_cmd.throttles[i] *= -1

            self.arm_throttle_cmd_pub.publish(arm_throttle_cmd)


    def handle_joystick_message(self, msg):
        # Tiny deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg["axes"][self.mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(
            msg["axes"][self.mappings["forward_back"]] * self.drive_config["forward_back"]["multiplier"], 0.05
        )

        # Convert from [0,1] to [0, self_max_wheel_speed] and apply dampen
        linear *= self.max_wheel_speed * dampen

        # Deadzones for each axis
        left_right = (
            deadzone(msg["axes"][self.mappings["left_right"]] * self.drive_config["left_right"]["multiplier"], 0.4)
            if self.drive_config["left_right"]["enabled"]
            else 0
        )
        twist = quadratic(deadzone(msg["axes"][self.mappings["twist"]] * self.drive_config["twist"]["multiplier"], 0.1))

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

        self.send(
            text_data=json.dumps(
                {
                    "type": "joystick",
                    "left_right": left_right,
                    "forward_back": msg["axes"][self.mappings["forward_back"]],
                    "twist": twist,
                    "dampen": dampen,
                    "pan": msg["axes"][self.mappings["pan"]],
                    "tilt": msg["axes"][self.mappings["tilt"]],
                }
            )
        )

    def pdb_callback(self, msg):
        self.send(text_data=json.dumps({"type": "pdb", "temperatures": msg.temperatures, "currents": msg.currents}))

    def calibration_checkbox_callback(self, msg):
        self.send(
            text_data=json.dumps({"type": "calibration_status", "names": msg.names, "calibrated": msg.calibrated})
        )

    def arm_controller_callback(self, msg):
        self.send(
            text_data=json.dumps({"type": "arm_moteus", "name": msg.name, "state": msg.state, "error": msg.error})
        )

    def drive_controller_callback(self, msg):
        self.send(
            text_data=json.dumps({"type": "drive_moteus", "name": msg.name, "state": msg.state, "error": msg.error})
        )

    def enable_laser_callback(self, msg):
        try:
            result = self.laser_service(data=msg["data"])
            self.send(text_data=json.dumps({"type": "laser_service", "result": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def enable_device_callback(self, msg):
        try:
            result = self.calibrate_service()
            self.send(text_data=json.dumps({"type": "calibrate_service", "result": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def calibrate_motors_callback(self, msg):
        self.send(
            text_data=json.dumps(
                {"type": "calibrate_service", "name": msg["name"], "state": msg["state"], "error": msg["error"]}
            )
        )

    def disable_auton_led(self):
        message = String()
        message.data = "off"
        self.led_pub.publish(message)

    def joint_state_callback(self, msg):
        msg.position = [x * self.wheel_radius for x in msg.position]
        msg.velocity = [x * self.wheel_radius for x in msg.velocity]
        self.send(
            text_data=json.dumps(
                {
                    "type": "joint_state",
                    "name": msg.name,
                    "position": msg.position,
                    "velocity": msg.velocity,
                    "effort": msg.effort,
                }
            )
        )

    def gps_fix_callback(self, msg):
        self.send(
            text_data=json.dumps(
                {"type": "nav_sat_fix", "latitude": msg.latitude, "longitude": msg.longitude, "altitude": msg.altitude}
            )
        )

    def send_auton_command(self, msg):
        self.enable_auton(
            msg["is_enabled"],
            [
                GPSWaypoint(
                    waypoint["latitude_degrees"],
                    waypoint["longitude_degrees"],
                    waypoint["tag_id"],
                    waypoint["type"],
                )
                for waypoint in msg["waypoints"]
            ],
        )

    def send_teleop_enabled(self, msg):
        self.teleop_pub.publish(msg["data"])

    def led_callback(self, msg):
        self.send(
            text_data=json.dumps(
                {"type": "led", "red": msg.red, "green": msg.green, "blue": msg.blue, "is_blinking": msg.is_blinking}
            )
        )

    def nav_state_callback(self, msg):
        self.send(text_data=json.dumps({"type": "nav_state", "state": msg.state}))

    def auton_bearing(self):
        base_link_in_map = SE3.from_tf_tree(self.tf_buffer, "map", "base_link")
        self.send(
            text_data=json.dumps(
                {
                    "type": "auton_tfclient",
                    "rotation": base_link_in_map.rotation.quaternion.tolist(),
                }
            )
        )

    def mast_gimbal(self, msg):
        pwr = rospy.get_param("teleop/mast_gimbal_power")
        rot_pwr = msg["throttles"][0] * pwr["rotation_pwr"]
        up_down_pwr = msg["throttles"][1] * pwr["up_down_pwr"]
        self.mast_gimbal_pub.publish(Throttle(["mast_gimbal_x", "mast_gimbal_y"], [rot_pwr, up_down_pwr]))
