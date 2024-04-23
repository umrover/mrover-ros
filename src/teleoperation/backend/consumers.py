import csv
from datetime import datetime
import json
import os
import pytz
from math import copysign
from math import pi
from tf.transformations import euler_from_quaternion
import threading

from channels.generic.websocket import JsonWebsocketConsumer
import rospy
import tf2_ros
import cv2

# from cv_bridge import CvBridge
from mrover.msg import (
    PDLB,
    ControllerState,
    GPSWaypoint,
    WaypointType,
    LED,
    StateMachineStateUpdate,
    Throttle,
    CalibrationStatus,
    MotorsStatus,
    CameraCmd,
    Velocity,
    Position,
    IK,
    Spectral,
    ScienceThermistors,
    HeaterData,
)
from mrover.srv import EnableAuton, AdjustMotor, ChangeCameras, CapturePanorama
from sensor_msgs.msg import NavSatFix, Temperature, RelativeHumidity, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist, Pose, Point, Quaternion

from util.SE3 import SE3

from backend.models import AutonWaypoint, BasicWaypoint


DEFAULT_ARM_DEADZONE = 0.15


# If below threshold, make output zero
def deadzone(magnitude: float, threshold: float) -> float:
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold)
    return copysign(temp_mag, magnitude)


def quadratic(val: float) -> float:
    return copysign(val**2, val)


class GUIConsumer(JsonWebsocketConsumer):
    def connect(self):
        self.accept()
        try:
            # Publishers
            self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
            self.led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=1)
            self.mast_gimbal_pub = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)
            self.arm_ik_pub = rospy.Publisher("arm_ik", IK, queue_size=1)
            self.arm_throttle_cmd_pub = rospy.Publisher("arm_throttle_cmd", Throttle, queue_size=1)
            self.arm_velocity_cmd_pub = rospy.Publisher("arm_velocity_cmd", Velocity, queue_size=1)
            self.arm_position_cmd_pub = rospy.Publisher("arm_position_cmd", Position, queue_size=1)
            self.sa_throttle_cmd_pub = rospy.Publisher("sa_throttle_cmd", Throttle, queue_size=1)
            self.sa_velocity_cmd_pub = rospy.Publisher("sa_velocity_cmd", Velocity, queue_size=1)
            self.sa_position_cmd_pub = rospy.Publisher("sa_position_cmd", Position, queue_size=1)
            self.cache_throttle_cmd_pub = rospy.Publisher("cache_throttle_cmd", Throttle, queue_size=1)
            self.cache_velocity_cmd_pub = rospy.Publisher("cache_velocity_cmd", Velocity, queue_size=1)
            self.cache_position_cmd_pub = rospy.Publisher("cache_position_cmd", Position, queue_size=1)

            # Subscribers
            self.pdb_sub = rospy.Subscriber("/pdb_data", PDLB, self.pdb_callback)
            self.arm_moteus_sub = rospy.Subscriber(
                "/arm_controller_data", ControllerState, self.arm_controller_callback
            )
            self.drive_moteus_sub = rospy.Subscriber(
                "/drive_controller_data", ControllerState, self.drive_controller_callback
            )
            self.gps_fix = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_fix_callback)
            self.drive_status_sub = rospy.Subscriber("/drive_status", MotorsStatus, self.drive_status_callback)
            self.led_sub = rospy.Subscriber("/led", LED, self.led_callback)
            self.nav_state_sub = rospy.Subscriber("/nav_state", StateMachineStateUpdate, self.nav_state_callback)
            self.imu_calibration = rospy.Subscriber("imu/calibration", CalibrationStatus, self.imu_calibration_callback)
            self.sa_temp_data = rospy.Subscriber("/sa_temp_data", Temperature, self.sa_temp_data_callback)
            self.sa_humidity_data = rospy.Subscriber(
                "/sa_humidity_data", RelativeHumidity, self.sa_humidity_data_callback
            )
            self.ish_thermistor_data = rospy.Subscriber(
                "/science_thermistors", ScienceThermistors, self.ish_thermistor_data_callback
            )
            self.ish_heater_state = rospy.Subscriber(
                "/science_heater_state", HeaterData, self.ish_heater_state_callback
            )
            self.science_spectral = rospy.Subscriber("/science_spectral", Spectral, self.science_spectral_callback)
            self.cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

            # Services
            self.laser_service = rospy.ServiceProxy("enable_arm_laser", SetBool)
            self.enable_auton = rospy.ServiceProxy("enable_auton", EnableAuton)
            self.change_heater_srv = rospy.ServiceProxy("science_change_heater_auto_shutoff_state", SetBool)
            self.calibrate_cache_srv = rospy.ServiceProxy("cache_calibrate", Trigger)
            self.cache_enable_limit = rospy.ServiceProxy("cache_enable_limit_switches", SetBool)
            self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)
            self.change_cameras_srv = rospy.ServiceProxy("change_cameras", ChangeCameras)
            self.capture_panorama_srv = rospy.ServiceProxy("capture_panorama", CapturePanorama)
            self.heater_auto_shutoff_srv = rospy.ServiceProxy("science_change_heater_auto_shutoff_state", SetBool)

            # ROS Parameters
            self.mappings = rospy.get_param("teleop/joystick_mappings")
            self.drive_config = rospy.get_param("teleop/drive_controls")
            self.max_wheel_speed = rospy.get_param("rover/max_speed")
            self.wheel_radius = rospy.get_param("wheel/radius")
            self.max_angular_speed = self.max_wheel_speed / self.wheel_radius
            self.ra_config = rospy.get_param("teleop/ra_controls")
            self.ik_names = rospy.get_param("teleop/ik_multipliers")
            self.RA_NAMES = rospy.get_param("teleop/ra_names")
            self.brushless_motors = rospy.get_param("brushless_motors/controllers")
            self.brushed_motors = rospy.get_param("brushed_motors/controllers")
            self.xbox_mappings = rospy.get_param("teleop/xbox_mappings")
            self.sa_config = rospy.get_param("teleop/sa_controls")

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.flight_thread = threading.Thread(target=self.flight_attitude_listener)
            self.flight_thread.start()

        except rospy.ROSException as e:
            rospy.logerr(e)

    def disconnect(self, close_code) -> None:
        self.pdb_sub.unregister()
        self.arm_moteus_sub.unregister()
        self.drive_moteus_sub.unregister()
        self.drive_status_sub.unregister()
        self.gps_fix.unregister()
        self.led_sub.unregister()
        self.nav_state_sub.unregister()
        self.imu_calibration.unregister()
        self.sa_temp_data.unregister()
        self.sa_humidity_data.unregister()
        self.ish_thermistor_data.unregister()
        self.science_spectral.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
        try:
            if message["type"] == "joystick_values":
                self.handle_joystick_message(message)
            elif message["type"] == "disable_auton_led":
                self.disable_auton_led()
            elif message["type"] == "laser_service":
                self.enable_laser_callback(message)
            elif message["type"] == "calibrate_motors":
                self.calibrate_motors(message)
            elif message["type"] == "arm_adjust":
                self.arm_adjust(message)
            elif (
                message["type"] == "arm_values"
                or message["type"] == "cache_values"
                or message["type"] == "sa_arm_values"
            ):
                self.handle_controls_message(message)
            elif message["type"] == "enable_white_leds":
                self.enable_white_leds_callback(message)
            elif message["type"] == "enable_uv_leds":
                self.enable_uv_leds_callback(message)
            elif message["type"] == "auton_command":
                self.send_auton_command(message)
            elif message["type"] == "teleop_enabled":
                self.send_teleop_enabled(message)
            elif message["type"] == "auton_tfclient":
                self.auton_bearing()
            elif message["type"] == "mast_gimbal":
                self.mast_gimbal(message)
            elif message["type"] == "max_streams":
                self.send_res_streams()
            elif message["type"] == "sendCameras":
                self.change_cameras(message)
            elif message["type"] == "takePanorama":
                self.capture_panorama()
            elif message["type"] == "capturePhoto":
                self.capture_photo()
            elif message["type"] == "heaterEnable":
                self.heater_enable_service(message)
            elif message["type"] == "autoShutoff":
                self.auto_shutoff_toggle(message)
            elif message["type"] == "center_map":
                self.send_center()
            elif message["type"] == "enable_limit_switch":
                self.limit_switch(message)
            elif message["type"] == "save_auton_waypoint_list":
                self.save_auton_waypoint_list(message)
            elif message["type"] == "get_auton_waypoint_list":
                self.get_auton_waypoint_list(message)
            elif message["type"] == "save_basic_waypoint_list":
                self.save_basic_waypoint_list(message)
            elif message["type"] == "get_basic_waypoint_list":
                self.get_basic_waypoint_list(message)
            elif message["type"] == "download_csv":
                self.download_csv(message)
        except Exception as e:
            rospy.logerr(e)

    def filter_xbox_axis(
        self,
        axes: "List[float]",
        axis: str,
        deadzone_threshold: float = DEFAULT_ARM_DEADZONE,
        quad_control: bool = False,
    ) -> float:
        index = self.xbox_mappings[axis]
        value = axes[index]

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
            return (
                self.brushless_motors[joint_name]["min_velocity"]
                + (input + 1)
                * (
                    self.brushless_motors[joint_name]["max_velocity"]
                    - self.brushless_motors[joint_name]["min_velocity"]
                )
                / 2
            )
        else:
            return (
                self.brushed_motors[joint_name]["min_velocity"]
                + (input + 1)
                * (self.brushed_motors[joint_name]["max_velocity"] - self.brushed_motors[joint_name]["min_velocity"])
                / 2
            )

    def handle_controls_message(self, msg):
        CACHE = ["cache"]
        SA_NAMES = ["sa_x", "sa_y", "sa_z", "sampler", "sensor_actuator"]
        RA_NAMES = self.RA_NAMES
        ra_slow_mode = False
        raw_left_trigger = msg["axes"][self.xbox_mappings["left_trigger"]]
        left_trigger = raw_left_trigger if raw_left_trigger > 0 else 0
        raw_right_trigger = msg["axes"][self.xbox_mappings["right_trigger"]]
        right_trigger = raw_right_trigger if raw_right_trigger > 0 else 0
        arm_pubs = [self.arm_position_cmd_pub, self.arm_velocity_cmd_pub, self.arm_throttle_cmd_pub, self.arm_ik_pub]
        sa_pubs = [self.sa_position_cmd_pub, self.sa_velocity_cmd_pub, self.sa_throttle_cmd_pub]
        cache_pubs = [self.cache_position_cmd_pub, self.cache_velocity_cmd_pub, self.cache_throttle_cmd_pub]
        publishers = []
        controls_names = []
        if msg["type"] == "cache_values":
            controls_names = CACHE
            publishers = cache_pubs
        elif msg["type"] == "arm_values":
            controls_names = RA_NAMES
            publishers = arm_pubs
        elif msg["type"] == "sa_arm_values":
            controls_names = SA_NAMES
            publishers = sa_pubs

        if msg["arm_mode"] == "ik":
            ee_in_map = SE3.from_tf_tree(self.tf_buffer, "base_link", "arm_e_link")

            left_trigger = self.filter_xbox_axis(msg["axes"],"left_trigger")
            if left_trigger < 0:
                left_trigger = 0

            right_trigger = self.filter_xbox_axis(msg["axes"],"right_trigger")
            if right_trigger < 0:
                right_trigger = 0
            ee_in_map.position[0] += (
                self.ik_names["x"] * self.filter_xbox_axis(msg["axes"],"left_js_x"),
            )
            ee_in_map.position[1] += (
                self.ik_names["y"] * self.filter_xbox_axis(msg["axes"],"left_js_y"),
            )
            ee_in_map.position[2] -= self.ik_names["z"] * left_trigger + self.ik_names["z"] * right_trigger

            arm_ik_cmd = IK(
                pose=Pose(
                    position=Point(*ee_in_map.position),
                    orientation=Quaternion(*ee_in_map.rotation.quaternion),
                )
            )
            publishers[3].publish(arm_ik_cmd)

        elif msg["arm_mode"] == "position":
            position_names = controls_names
            if msg["type"] == "arm_values":
                position_names = ["joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"]
            position_cmd = Position(
                names=position_names,
                positions=msg["positions"],
            )
            publishers[0].publish(position_cmd)

        elif msg["arm_mode"] == "velocity":
            velocity_cmd = Velocity()
            velocity_cmd.names = controls_names
            if msg["type"] == "cache_values":
                velocity_cmd.velocities = [
                    self.sa_config["cache"]["multiplier"]
                    * self.filter_xbox_button(msg["buttons"], "right_bumper", "left_bumper")
                ]
            elif msg["type"] == "sa_arm_values":
                velocity_cmd.velocities = [
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.sa_config["sa_x"]["xbox_index"]), "sa_x", False
                    ),
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.sa_config["sa_y"]["xbox_index"]), "sa_y", False
                    ),
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.sa_config["sa_z"]["xbox_index"]), "sa_z", True
                    ),
                    self.sa_config["sampler"]["multiplier"] * (right_trigger - left_trigger),
                    self.sa_config["sensor_actuator"]["multiplier"]
                    * self.filter_xbox_button(msg["buttons"], "right_bumper", "left_bumper"),
                ]
            elif msg["type"] == "arm_values":
                velocity_cmd.velocities = [
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.ra_config["joint_a"]["xbox_index"]), "joint_a"
                    ),
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.ra_config["joint_b"]["xbox_index"]), "joint_b", False
                    ),
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.ra_config["joint_c"]["xbox_index"]), "joint_c"
                    ),
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.ra_config["joint_de_pitch"]["xbox_index"]), "joint_de_0"
                    ),
                    self.to_velocity(
                        self.filter_xbox_axis(msg["axes"],self.ra_config["joint_de_roll"]["xbox_index"]), "joint_de_1"
                    ),
                    self.ra_config["allen_key"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "y", "a"),
                    self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "b", "x"),
                ]
            publishers[1].publish(velocity_cmd)

        elif msg["arm_mode"] == "throttle":
            throttle_cmd = Throttle()
            throttle_cmd.names = controls_names
            if msg["type"] == "cache_values":
                throttle_cmd.throttles = [
                    self.sa_config["cache"]["multiplier"]
                    * self.filter_xbox_button(msg["buttons"], "right_bumper", "left_bumper")
                ]
            elif msg["type"] == "arm_values":
                # print(msg["buttons"])

                # d_pad_x = msg["axes"][self.xbox_mappings["d_pad_x"]]
                # if d_pad_x > 0.5:
                #     ra_slow_mode = True
                # elif d_pad_x < -0.5:
                #     ra_slow_mode = False

                throttle_cmd.throttles = [
                    self.filter_xbox_axis(msg["axes"],self.ra_config["joint_a"]["xbox_index"]),
                    self.filter_xbox_axis(msg["axes"], self.ra_config["joint_b"]["xbox_index"]),
                    self.filter_xbox_axis(msg["axes"], self.ra_config["joint_c"]["xbox_index"]),
                    self.filter_xbox_axis(
                        msg["axes"],self.ra_config["joint_de_pitch"]["xbox_index_right"]
                    )
                    - self.filter_xbox_axis(msg["axes"],self.ra_config["joint_de_pitch"]["xbox_index_left"]),
                    self.filter_xbox_axis(
                        msg["axes"],self.ra_config["joint_de_roll"]["xbox_index_right"]
                    )
                    - self.filter_xbox_axis(msg["axes"],self.ra_config["joint_de_roll"]["xbox_index_left"]),
                    self.ra_config["allen_key"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "y", "a"),
                    self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(msg["buttons"], "b", "x"),
                ]

                for i, name in enumerate(self.RA_NAMES):
                    if ra_slow_mode:
                        throttle_cmd.throttles[i] *= self.ra_config[name]["slow_mode_multiplier"]
                    if self.ra_config[name]["invert"]:
                        throttle_cmd.throttles[i] *= -1
            elif msg["type"] == "sa_arm_values":
                throttle_cmd.throttles = [
                    self.filter_xbox_axis(msg["axes"],self.sa_config["sa_x"]["xbox_index"]),
                    self.filter_xbox_axis(msg["axes"],self.sa_config["sa_y"]["xbox_index"]),
                    self.filter_xbox_axis(msg["axes"],self.sa_config["sa_z"]["xbox_index"]),
                    self.sa_config["sampler"]["multiplier"] * (right_trigger - left_trigger),
                    self.sa_config["sensor_actuator"]["multiplier"]
                    * self.filter_xbox_button(msg["buttons"], "right_bumper", "left_bumper"),
                ]

                fast_mode_activated = msg["buttons"][self.xbox_mappings["a"]] or msg["buttons"][self.xbox_mappings["b"]]
                if not fast_mode_activated:
                    for i, name in enumerate(SA_NAMES):
                        # When going up (vel > 0) with SA joint 2, we DON'T want slow mode.
                        if not (name == "sa_y" and throttle_cmd.throttles[i] > 0):
                            throttle_cmd.throttles[i] *= self.sa_config[name]["slow_mode_multiplier"]
            publishers[2].publish(throttle_cmd)

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

    def arm_controller_callback(self, msg):
        hits = []
        for n in msg.limit_hit:
            temp = []
            for i in range(4):
                temp.append((1 if n & (1 << i) != 0 else 0))
            hits.append(temp)
        self.send(
            text_data=json.dumps(
                {"type": "arm_moteus", "name": msg.name, "state": msg.state, "error": msg.error, "limit_hit": hits}
            )
        )

    def drive_controller_callback(self, msg):
        hits = []
        for n in msg.limit_hit:
            temp = []
            for i in range(4):
                temp.append((1 if n & (1 << i) != 0 else 0))
            hits.append(temp)
        self.send(
            text_data=json.dumps(
                {"type": "drive_moteus", "name": msg.name, "state": msg.state, "error": msg.error, "limit_hit": hits}
            )
        )

    def enable_laser_callback(self, msg):
        try:
            result = self.laser_service(data=msg["data"])
            self.send(text_data=json.dumps({"type": "laser_service", "success": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def limit_switch(self, msg):
        fail = []
        if msg["name"] == "all_ra":
            for joint in self.RA_NAMES:
                name = "arm_enable_limit_switch_" + joint
                self.limit_switch_service = rospy.ServiceProxy(name, SetBool)
                try:
                    result = self.limit_switch_service(data=msg["data"])
                    if not result.success:
                        fail.append(joint)
                except rospy.ServiceException as e:
                    print(f"Service call failed: {e}")
        else:
            self.limit_switch_service = rospy.ServiceProxy(msg["name"], SetBool)
            try:
                result = self.limit_switch_service(data=msg["data"])
                if not result.success:
                    fail.append(joint)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

    def enable_white_leds_callback(self, msg):
        for i in range(0, 3):
            white_led_name = f"science_enable_white_led_{i}"
            led_srv = rospy.ServiceProxy(white_led_name, SetBool)
            try:
                result = led_srv(data=msg["data"])
                self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def enable_uv_leds_callback(self, msg):
        for i in range(0, 3):
            uv_led_name = f"science_enable_uv_led_{i}"
            led_srv = rospy.ServiceProxy(uv_led_name, SetBool)
            try:
                result = led_srv(data=msg["data"])
                self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def enable_device_callback(self, msg):
        try:
            result = self.calibrate_service()
            self.send(text_data=json.dumps({"type": "calibrate_service", "result": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(e)

        self.send(text_data=json.dumps({"type": "enable_limit_switch", "result": fail}))

    def calibrate_motors(self, msg):
        fail = []  # if any calibration fails, add joint name to a list to return
        if msg["topic"] == "all_ra":
            for joint in self.RA_NAMES:
                self.calibrate_service = rospy.ServiceProxy("arm_calibrate_" + joint, Trigger)
                try:
                    result = self.calibrate_service()
                    if not result.success:
                        fail.append(joint)
                except rospy.ServiceException as e:
                    print(f"Service call failed: {e}")
        else:
            self.calibrate_service = rospy.ServiceProxy(msg["topic"], Trigger)
            try:
                result = self.calibrate_service()
                if not result.success:
                    fail = msg["topic"]
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

        self.send(text_data=json.dumps({"type": "calibrate_motors", "result": fail}))

    def arm_adjust(self, msg):
        try:
            arm_adjust_srv = rospy.ServiceProxy(msg["name"] + "_adjust", AdjustMotor)
            result = arm_adjust_srv(name=msg["name"], value=msg["value"])
            self.send(text_data=json.dumps({"type": "arm_adjust", "success": result.success}))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def disable_auton_led(self):
        message = String()
        message.data = "off"
        self.led_pub.publish(message)

    def drive_status_callback(self, msg):
        msg.joint_states.position = [x * self.wheel_radius for x in msg.joint_states.position]
        msg.joint_states.velocity = [x * self.wheel_radius for x in msg.joint_states.velocity]
        self.send(
            text_data=json.dumps(
                {
                    "type": "drive_status",
                    "name": msg.name,
                    "position": msg.joint_states.position,
                    "velocity": msg.joint_states.velocity,
                    "effort": msg.joint_states.effort,
                    "state": msg.moteus_states.state,
                    "error": msg.moteus_states.error,
                }
            )
        )

    def cmd_vel_callback(self, msg):
        self.send(text_data=json.dumps({"type": "cmd_vel", "linear_x": msg.linear.x, "angular_z": msg.angular.z}))

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
                    waypoint["tag_id"],
                    waypoint["latitude_degrees"],
                    waypoint["longitude_degrees"],
                    WaypointType(waypoint["type"]),
                )
                for waypoint in msg["waypoints"]
            ],
        )

    def send_teleop_enabled(self, msg):
        rospy.wait_for_service("enable_teleop")
        try:
            enable_teleop = rospy.ServiceProxy("enable_teleop", SetBool)
            enable_teleop(msg["data"])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def led_callback(self, msg):
        self.send(
            text_data=json.dumps(
                {"type": "led", "red": msg.red, "green": msg.green, "blue": msg.blue, "is_blinking": msg.is_blinking}
            )
        )

    def nav_state_callback(self, msg):
        self.send(text_data=json.dumps({"type": "nav_state", "state": msg.state}))

    def sa_temp_data_callback(self, msg):
        self.send(text_data=json.dumps({"type": "temp_data", "temp_data": msg.temperature}))

    def sa_humidity_data_callback(self, msg):
        self.send(text_data=json.dumps({"type": "relative_humidity", "humidity_data": msg.relative_humidity}))

    def ish_thermistor_data_callback(self, msg):
        temps = [x.temperature for x in msg.temps]
        self.send(text_data=json.dumps({"type": "thermistor", "temps": temps}))

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
        self.mast_gimbal_pub.publish(Throttle(["mast_gimbal_y", "mast_gimbal_z"], [rot_pwr, up_down_pwr]))

    def change_cameras(self, msg):
        try:
            camera_cmd = CameraCmd(msg["device"], msg["resolution"])
            result = self.change_cameras_srv(primary=msg["primary"], camera_cmd=camera_cmd)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def send_res_streams(self):
        res = rospy.get_param("cameras/max_num_resolutions")
        streams = rospy.get_param("cameras/max_streams")
        self.send(text_data=json.dumps({"type": "max_resolution", "res": res}))
        self.send(text_data=json.dumps({"type": "max_streams", "streams": streams}))

    def capture_panorama(self) -> None:
        try:
            response = self.capture_panorama_srv()
            image = response.panorama
            # self.image_callback(image)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    # def capture_photo(self):
    #     try:
    #         response = self.capture_photo_srv()
    #         image = response.photo
    #         self.image_callback(image)
    #     except rospy.ServiceException as e:
    #         print(f"Service call failed: {e}")

    # def image_callback(self, msg):
    #     bridge = CvBridge()
    #     try:
    #         # Convert the image to OpenCV standard format
    #         cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #     except Exception as e:
    #         rospy.logerr("Could not convert image message to OpenCV image: " + str(e))
    #         return

    #     # Save the image to a file (you could change 'png' to 'jpg' or other formats)
    #     image_filename = "panorama.png"
    #     try:
    #         cv2.imwrite(image_filename, cv_image)
    #         rospy.loginfo("Saved image to {}".format(image_filename))
    #     except Exception as e:
    #         rospy.logerr("Could not save image: " + str(e))

    def heater_enable_service(self, msg):
        try:
            heater = msg["heater"]
            science_enable = rospy.ServiceProxy("science_enable_heater_" + heater, SetBool)
            result = science_enable(data=msg["enabled"])
        except rospy.ServiceException as e:
            print(f"Service init failed: {e}")

    def ish_heater_state_callback(self, msg):
        self.send(text_data=json.dumps({"type": "heater_states", "states": msg.state}))

    def auto_shutoff_toggle(self, msg):
        try:
            rospy.logerr(msg)
            result = self.heater_auto_shutoff_srv(data=msg["shutoff"])
            self.send(text_data=json.dumps({"type": "auto_shutoff", "success": result.success}))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def send_center(self):
        lat = rospy.get_param("gps_linearization/reference_point_latitude")
        long = rospy.get_param("gps_linearization/reference_point_longitude")
        self.send(text_data=json.dumps({"type": "center_map", "latitude": lat, "longitude": long}))

    def save_auton_waypoint_list(self, msg):
        AutonWaypoint.objects.all().delete()
        waypoints = []
        for w in msg["data"]:
            waypoints.append(
                AutonWaypoint(tag_id=w["id"], type=w["type"], latitude=w["lat"], longitude=w["lon"], name=w["name"])
            )
        AutonWaypoint.objects.bulk_create(waypoints)
        self.send(text_data=json.dumps({"type": "save_auton_waypoint_list", "success": True}))
        # Print out all of the waypoints
        for w in AutonWaypoint.objects.all():
            rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))

    def get_auton_waypoint_list(self, msg):
        waypoints = []
        for w in AutonWaypoint.objects.all():
            waypoints.append({"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type})
        self.send(text_data=json.dumps({"type": "get_auton_waypoint_list", "data": waypoints}))

    def save_basic_waypoint_list(self, msg):
        BasicWaypoint.objects.all().delete()
        waypoints = []
        for w in msg["data"]:
            waypoints.append(BasicWaypoint(drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]))
        BasicWaypoint.objects.bulk_create(waypoints)
        self.send(text_data=json.dumps({"type": "save_basic_waypoint_list", "success": True}))
        # Print out all of the waypoints
        for w in BasicWaypoint.objects.all():
            rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))

    def get_basic_waypoint_list(self, msg):
        waypoints = []
        for w in BasicWaypoint.objects.all():
            waypoints.append({"name": w.name, "drone": w.drone, "lat": w.latitude, "lon": w.longitude})
        self.send(text_data=json.dumps({"type": "get_basic_waypoint_list", "data": waypoints}))

    def imu_calibration_callback(self, msg) -> None:
        self.send(text_data=json.dumps({"type": "calibration_status", "system_calibration": msg.system_calibration}))

    def flight_attitude_listener(self):
        # threshold that must be exceeded to send JSON message
        threshold = 0.1
        map_to_baselink = SE3()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                tf_msg = SE3.from_tf_tree(self.tf_buffer, "map", "base_link")

                if tf_msg.is_approx(map_to_baselink, threshold):
                    rate.sleep()
                    continue
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rate.sleep()
                continue

            map_to_baselink = tf_msg
            rotation = map_to_baselink.rotation
            euler = euler_from_quaternion(rotation.quaternion)
            pitch = euler[0] * 180 / pi
            roll = euler[1] * 180 / pi

            self.send(text_data=json.dumps({"type": "flight_attitude", "pitch": pitch, "roll": roll}))

            rate.sleep()

    def science_spectral_callback(self, msg):
        self.send(
            text_data=json.dumps({"type": "spectral_data", "site": msg.site, "data": msg.data, "error": msg.error})
        )

    def download_csv(self, msg):
        username = os.getenv("USERNAME", "-1")

        now = datetime.now(pytz.timezone("US/Eastern"))
        current_time = now.strftime("%m/%d/%Y-%H:%M")
        spectral_data = msg["data"]

        site_names = ["A", "B", "C"]
        index = 0

        # add site letter in front of data
        for site_data in spectral_data:
            site_data.insert(0, f"Site {site_names[index]}")
            index = index + 1

        time_row = ["Time", current_time]
        spectral_data.insert(0, time_row)

        if username == "-1":
            rospy.logerr("username not found")

        with open(os.path.join(f"/home/{username}/Downloads/spectral_data.csv"), "w") as f:
            csv_writer = csv.writer(f)
            csv_writer.writerows(spectral_data)
