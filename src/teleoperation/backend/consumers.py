import json
import bson
import traceback
from typing import Any, Type

import yaml
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
import tf2_ros
import numpy as np
from backend.cache_controls import send_cache_controls
from backend.drive_controls import send_controller_twist, send_joystick_twist
from backend.input import DeviceInputs
from backend.mast_controls import send_mast_controls
from backend.models import BasicWaypoint, AutonWaypoint
from backend.ra_controls import send_ra_controls
from backend.sa_controls import send_sa_controls
from geometry_msgs.msg import Twist
from mrover.msg import ( CalibrationStatus, ControllerState, StateMachineStateUpdate, LED, GPSWaypoint, WaypointType, 
                        NetworkBandwidth, ScienceThermistors, Spectral, HeaterData )
from mrover.srv import EnableAuton
from sensor_msgs.msg import JointState, NavSatFix, RelativeHumidity, Temperature
from std_srvs.srv import SetBool
from util.SE3 import SE3

LOCALIZATION_INFO_HZ = 10

rospy.init_node("teleoperation", disable_signals=True)

tf2_buffer = tf2_ros.Buffer()
tf2_ros.TransformListener(tf2_buffer)

ra_mode = "disabled"
sa_mode = "disabled"
cache_mode = "disabled"


def ra_timer_expired(_):
    global ra_mode
    rospy.logwarn("RA GUI timed out. Disabling...")
    ra_mode = "disabled"


ra_timer = None


def sa_timer_expired(_):
    global sa_mode
    rospy.logwarn("SA GUI timed out. Disabling...")
    sa_mode = "disabled"


sa_timer = None


def cache_timer_expired(_) -> None:
    global cache_mode
    rospy.logwarn("Cache GUI timed out. Disabling...")


cache_timer = None


class GUIConsumer(JsonWebsocketConsumer):
    subscribers: list[rospy.Subscriber] = []
    timers: list[rospy.Timer] = []
    enable_auton: rospy.ServiceProxy
    enable_teleop: rospy.ServiceProxy

    def connect(self) -> None:
        self.accept()

        self.forward_ros_topic("/imu/calibration", CalibrationStatus, "calibration")
        self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")
        self.forward_ros_topic("/arm_joint_data", JointState, "fk")
        self.forward_ros_topic("/arm_controller_data", ControllerState, "arm_state")
        self.forward_ros_topic("/drive_left_controller_data", ControllerState, "drive_left_state")
        self.forward_ros_topic("/drive_right_controller_data", ControllerState, "drive_right_state")
        self.forward_ros_topic("/nav_state", StateMachineStateUpdate, "nav_state")
        self.forward_ros_topic("/led", LED, "led")
        self.forward_ros_topic("/cmd_vel", Twist, "cmd_vel")
        self.forward_ros_topic("/sa_humidity_data", RelativeHumidity, "soil_humidity")
        self.forward_ros_topic("/sa_temp_data", Temperature, "soil_temp")
        self.forward_ros_topic("/sa_thermistor_data", Temperature, "soil_therm_temp")
        self.forward_ros_topic("/sa_joint_data", JointState, "sa_joint")
        self.forward_ros_topic("/corer_joint_data", JointState, "plunger")
        self.forward_ros_topic("/network_bandwidth", NetworkBandwidth, "network")
        self.forward_ros_topic("/science_thermistors", ScienceThermistors, "thermistor")
        self.forward_ros_topic("science_spectral", Spectral, "spectral_data")
        self.forward_ros_topic("/science_heater_state", HeaterData, "heater_states")

    #     eaterDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::HeaterData>("science_heater_state", 1));
    # spectralDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::Spectral>("science_spectral", 1));
    # thermistorDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::ScienceThermistors>("science_thermistors", 1));

        self.enable_auton = rospy.ServiceProxy("enable_auton", EnableAuton)
        self.enable_teleop = rospy.ServiceProxy("enable_teleop", SetBool)

        self.timers.append(rospy.Timer(rospy.Duration(1 / LOCALIZATION_INFO_HZ), self.send_localization_callback))

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            subscriber.unregister()
        for timer in self.timers:
            timer.shutdown()

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            # Formatting a ROS message as a string outputs YAML
            # Parse it back into a dictionary, so we can send it as JSON
            self.send_message_as_json({"type": gui_msg_type, **yaml.safe_load(str(ros_message))})

        self.subscribers.append(rospy.Subscriber(topic_name, topic_type, callback))

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            rospy.logwarn(f"Failed to send message: {e}")

    def send_localization_callback(self, _):
        try:
            base_link_in_map = SE3.from_tf_tree(tf2_buffer, "map", "base_link")
            self.send_message_as_json(
                {
                    "type": "orientation",
                    "orientation": base_link_in_map.rotation.quaternion.tolist(),
                }
            )
        except Exception as e:
            rospy.logwarn_throttle(5, f"Failed to get bearing: {e} Is localization running?")

    def save_basic_waypoint_list(self, waypoints: list[dict]) -> None:
        BasicWaypoint.objects.all().delete()
        BasicWaypoint.objects.bulk_create(
            [BasicWaypoint(drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]) for w in waypoints]
        )
        self.send_message_as_json({"type": "save_basic_waypoint_list", "success": True})

    def get_basic_waypoint_list(self) -> None:
        self.send_message_as_json(
            {
                "type": "get_basic_waypoint_list",
                "data": [
                    {"name": w.name, "drone": w.drone, "lat": w.latitude, "lon": w.longitude}
                    for w in BasicWaypoint.objects.all()
                ],
            }
        )

    def save_auton_waypoint_list(self, waypoints: list[dict]) -> None:
        AutonWaypoint.objects.all().delete()
        AutonWaypoint.objects.bulk_create(
            [
                AutonWaypoint(
                    tag_id=w["id"],
                    type=w["type"],
                    latitude=w["lat"],
                    longitude=w["lon"],
                    name=w["name"],
                )
                for w in waypoints
            ]
        )
        self.send_message_as_json({"type": "save_auton_waypoint_list", "success": True})

    def get_auton_waypoint_list(self) -> None:
        self.send_message_as_json(
            {
                "type": "get_auton_waypoint_list",
                "data": [
                    {"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type}
                    for w in AutonWaypoint.objects.all()
                ],
            }
        )

    def send_auton_command(self, waypoints: list[dict], enabled: bool) -> None:
        self.enable_auton(
            enabled,
            [
                GPSWaypoint(
                    waypoint["tag_id"],
                    waypoint["latitude_degrees"],
                    waypoint["longitude_degrees"],
                    WaypointType(int(waypoint["type"])),
                )
                for waypoint in waypoints
            ],
        )

    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        global ra_mode, ra_timer, sa_mode, sa_timer, cache_mode, cache_timer

        if text_data is not None:
            try:
                message = json.loads(text_data)
            except json.JSONDecodeError as e:
                rospy.logwarn(f"Failed to decode JSON: {e}")
                return
        elif bytes_data is not None:
            message = bson.loads(bytes_data)

        try:
            match message:
                case {
                    "type": "joystick" | "ra_controller" | "sa_controller" | "mast_keyboard" | "cache_keyboard",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    match message["type"]:
                        case "joystick":
                            send_joystick_twist(device_input)
                        case "ra_controller":
                            send_controller_twist(device_input)
                            send_ra_controls(ra_mode, device_input)
                        case "sa_controller":
                            send_sa_controls(device_input)
                        case "mast_keyboard":
                            send_mast_controls(device_input)
                        case "cache_keyboard":
                            send_cache_controls(cache_mode, device_input)

                case {"type": "ra_mode", "mode": new_ra_mode}:
                    ra_mode = new_ra_mode
                    if ra_timer:
                        ra_timer.shutdown()
                    ra_timer = rospy.Timer(rospy.Duration(1), ra_timer_expired, oneshot=True)
                case {"type": "sa_mode", "mode": new_sa_mode}:
                    sa_mode = new_sa_mode
                    if sa_timer:
                        sa_timer.shutdown()
                    sa_timer = rospy.Timer(rospy.Duration(1), sa_timer_expired, oneshot=True)
                case {"type": "cache_mode", "mode": new_cache_mode}:
                    cache_mode = new_cache_mode
                    if cache_timer:
                        cache_timer.shutdown()
                    cache_timer = rospy.Timer(rospy.Duration(1), cache_timer_expired, oneshot=True)
                case {"type": "auton_enable", "enabled": enabled, "waypoints": waypoints}:
                    self.send_auton_command(waypoints, enabled)
                case {"type": "teleop_enable", "enabled": enabled}:
                    self.enable_teleop(enabled)
                case {"type": "enable_white_leds", "data": data}:
                    for i in range(3):
                        led_service = rospy.ServiceProxy(f"science_enable_white_led_{i}", SetBool)
                        led_service(data=data)
                        led_service.close()
                case {"type": "enable_uv_leds", "data": data}:
                    for i in range(3):
                        led_service = rospy.ServiceProxy(f"science_enable_uv_led_{i}", SetBool)
                        led_service(data=data)
                        led_service.close()
                case {"type": "heater_enable", "heater": heater, "enabled": enabled}:
                    heater_service = rospy.ServiceProxy(f"science_enable_heater_{heater}", SetBool)
                    heater_service(data=enabled)
                    heater_service.close()
                case {"type": "auto_shutoff", "shutoff": shutoff}:
                    auto_shutoff_service = rospy.ServiceProxy("heater_auto_shutoff", SetBool)
                    auto_shutoff_service(data=shutoff)
                    auto_shutoff_service.close()
                case {"type": "save_basic_waypoint_list", "data": waypoints}:
                    self.save_basic_waypoint_list(waypoints)
                case {"type": "save_auton_waypoint_list", "data": waypoints}:
                    self.save_auton_waypoint_list(waypoints)
                case {"type": "poly_fit", "temperatures": temperatures, "timestamps": timestamps}:
                    y_log = np.log(temperatures)
                    exponents = np.polyfit(timestamps, y_log, 1)
                    exponents = list(exponents)
                    self.send(text_data=json.dumps({"type": "poly_fit", "exponents": exponents}))
                case _:
                    match message["type"]:
                        case "get_basic_waypoint_list":
                            self.get_basic_waypoint_list()
                        case "get_auton_waypoint_list":
                            self.get_auton_waypoint_list()
                        case _:
                            rospy.logwarn(f"Unhandled message: {message}")

        except:
            rospy.logerr(f"Failed to handle message: {message}")
            rospy.logerr(traceback.format_exc())

        # try:
        #     if message["type"] == "joystick_values":
        #         update_joystick(message)
        #     elif message["type"] == "disable_auton_led":
        #         self.disable_auton_led()
        #     elif message["type"] == "calibrate_motors":
        #         self.calibrate_motors(message)
        #     elif message["type"] == "arm_adjust":
        #         self.arm_adjust(message)
        #     elif message["type"] in {"arm_values", "cache_values", "sa_arm_values"}:
        #         self.handle_controls_message(message)
        #     elif message["type"] == "enable_white_leds":
        #         self.enable_white_leds_callback(message)
        #     elif message["type"] == "enable_uv_leds":
        #         self.enable_uv_leds_callback(message)
        #     elif message["type"] == "auton_command":
        #         self.send_auton_command(message)
        #     elif message["type"] == "teleop_enabled":
        #         self.send_teleop_enabled(message)
        #     elif message["type"] == "bearing":
        #         self.bearing()
        #     elif message["type"] == "mast_gimbal":
        #         self.mast_gimbal(message)
        #     elif message["type"] == "takePanorama":
        #         self.capture_panorama()
        #     elif message["type"] == "heaterEnable":
        #         self.heater_enable_service(message)
        #     elif message["type"] == "autoShutoff":
        #         self.auto_shutoff_toggle(message)
        #     elif message["type"] == "center_map":
        #         self.send_center()
        #     elif message["type"] == "save_auton_waypoint_list":
        #         self.save_auton_waypoint_list(message)
        #     elif message["type"] == "get_auton_waypoint_list":
        #         self.get_auton_waypoint_list(message)
        #     elif message["type"] == "save_basic_waypoint_list":
        #         self.save_basic_waypoint_list(message)
        #     elif message["type"] == "get_basic_waypoint_list":
        #         self.get_basic_waypoint_list(message)
        #     elif message["type"] == "download_csv":
        #         self.download_csv(message)
        #     elif message["type"] == "reset_gimbal":
        #         self.reset_gimbal()
        # except Exception as e:
        #     rospy.logerr(e)

    # def pdb_callback(self, msg):
    #     self.send(text_data=json.dumps({"type": "pdb", "temperatures": msg.temperatures, "currents": msg.currents}))
    #
    # def arm_controller_callback(self, msg):
    #     hits = []
    #     for n in msg.limit_hit:
    #         temp = []
    #         for i in range(4):
    #             temp.append((1 if n & (1 << i) != 0 else 0))
    #         hits.append(temp)
    #     self.send(
    #         text_data=json.dumps(
    #             {"type": "arm_moteus", "name": msg.name, "state": msg.state, "error": msg.error, "limit_hit": hits}
    #         )
    #     )
    #
    # def sa_joint_callback(self, msg):
    #     names = msg.name
    #     z = msg.position[names.index("sa_z")]
    #     self.send(text_data=json.dumps({"type": "sa_z", "sa_z": z}))
    #
    # def drive_controller_callback(self, msg):
    #     hits = []
    #     for n in msg.limit_hit:
    #         temp = []
    #         for i in range(4):
    #             temp.append((1 if n & (1 << i) != 0 else 0))
    #         hits.append(temp)
    #     self.send(
    #         text_data=json.dumps(
    #             {"type": "drive_moteus", "name": msg.name, "state": msg.state, "error": msg.error, "limit_hit": hits}
    #         )
    #     )
    #
    # def enable_white_leds_callback(self, msg):
    #     for i in range(0, 3):
    #         white_led_name = f"science_enable_white_led_{i}"
    #         led_srv = rospy.ServiceProxy(white_led_name, SetBool)
    #         try:
    #             result = led_srv(data=msg["data"])
    #             self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call failed: {e}")
    #
    # def enable_uv_leds_callback(self, msg):
    #     for i in range(0, 3):
    #         uv_led_name = f"science_enable_uv_led_{i}"
    #         led_srv = rospy.ServiceProxy(uv_led_name, SetBool)
    #         try:
    #             result = led_srv(data=msg["data"])
    #             self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call failed: {e}")
    #
    # def enable_device_callback(self, msg):
    #     try:
    #         result = self.calibrate_service()
    #         self.send(text_data=json.dumps({"type": "calibrate_service", "result": result.success}))
    #     except rospy.ServiceException as e:
    #         rospy.logerr(e)
    #
    # def calibrate_motors(self, msg):
    #     fail = []  # if any calibration fails, add joint name to a list to return
    #     if msg["topic"] == "all_ra":
    #         for joint in self.RA_NAMES:
    #             self.calibrate_service = rospy.ServiceProxy("arm_calibrate_" + joint, Trigger)
    #             try:
    #                 result = self.calibrate_service()
    #                 if not result.success:
    #                     fail.append(joint)
    #             except rospy.ServiceException as e:
    #                 print(f"Service call failed: {e}")
    #     else:
    #         self.calibrate_service = rospy.ServiceProxy(msg["topic"], Trigger)
    #         try:
    #             result = self.calibrate_service()
    #             if not result.success:
    #                 fail = msg["topic"]
    #         except rospy.ServiceException as e:
    #             print(f"Service call failed: {e}")
    #
    #     self.send(text_data=json.dumps({"type": "calibrate_motors", "result": fail}))
    #
    # def arm_adjust(self, msg):
    #     try:
    #         arm_adjust_srv = rospy.ServiceProxy(msg["name"] + "_adjust", AdjustMotor)
    #         result = arm_adjust_srv(name=msg["name"], value=msg["value"])
    #         self.send(text_data=json.dumps({"type": "arm_adjust", "success": result.success}))
    #     except rospy.ServiceException as e:
    #         print(f"Service call failed: {e}")
    #
    # def reset_gimbal(self):
    #     try:
    #         adjust_srv = rospy.ServiceProxy("mast_gimbal_z_adjust", AdjustMotor)
    #         result = adjust_srv(name="mast_gimbal_z", value=0)
    #         self.send(text_data=json.dumps({"type": "arm_adjust", "success": result.success}))
    #     except rospy.ServiceException as e:
    #         print(f"Service call failed: {e}")
    #
    # def disable_auton_led(self):
    #     message = String()
    #     message.data = "off"
    #     self.led_pub.publish(message)
    #
    # def drive_status_callback(self, msg):
    #     msg.joint_states.position = [x * self.wheel_radius for x in msg.joint_states.position]
    #     msg.joint_states.velocity = [x * self.wheel_radius for x in msg.joint_states.velocity]
    #     self.send(
    #         text_data=json.dumps(
    #             {
    #                 "type": "drive_status",
    #                 "name": msg.name,
    #                 "position": msg.joint_states.position,
    #                 "velocity": msg.joint_states.velocity,
    #                 "effort": msg.joint_states.effort,
    #                 "state": msg.moteus_states.state,
    #                 "error": msg.moteus_states.error,
    #             }
    #         )
    #     )
    #
    # def cmd_vel_callback(self, msg):
    #     self.send(text_data=json.dumps({"type": "cmd_vel", "linear_x": msg.linear.x, "angular_z": msg.angular.z}))
    #
    # def gps_fix_callback(self, msg):
    #     self.send(
    #         text_data=json.dumps(
    #             {"type": "nav_sat_fix", "latitude": msg.latitude, "longitude": msg.longitude, "altitude": msg.altitude}
    #         )
    #     )
    #
    # def send_auton_command(self, msg):
    #     self.enable_auton(
    #         msg["is_enabled"],
    #         [
    #             GPSWaypoint(
    #                 waypoint["tag_id"],
    #                 waypoint["latitude_degrees"],
    #                 waypoint["longitude_degrees"],
    #                 WaypointType(waypoint["type"]),
    #             )
    #             for waypoint in msg["waypoints"]
    #         ],
    #     )
    #
    # def send_teleop_enabled(self, msg):
    #     rospy.wait_for_service("enable_teleop")
    #     try:
    #         enable_teleop = rospy.ServiceProxy("enable_teleop", SetBool)
    #         enable_teleop(msg["data"])
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")
    #
    # def led_callback(self, msg):
    #     self.send(
    #         text_data=json.dumps(
    #             {"type": "led", "red": msg.red, "green": msg.green, "blue": msg.blue, "is_blinking": msg.is_blinking}
    #         )
    #     )
    #
    # def nav_state_callback(self, msg):
    #     self.send(text_data=json.dumps({"type": "nav_state", "state": msg.state}))
    #
    # def sa_temp_data_callback(self, msg):
    #     self.send(text_data=json.dumps({"type": "temp_data", "temp_data": msg.temperature}))
    #
    # def sa_humidity_data_callback(self, msg):
    #     self.send(text_data=json.dumps({"type": "relative_humidity", "humidity_data": msg.relative_humidity}))
    #
    # def ish_thermistor_data_callback(self, msg):
    #     temps = [x.temperature for x in msg.temps]
    #     self.send(text_data=json.dumps({"type": "thermistor", "temps": temps}))

    # def mast_gimbal(self, msg):
    #     pwr = rospy.get_param("teleop/mast_gimbal_power")
    #     rot_pwr = msg["throttles"][0] * pwr["rotation_pwr"]
    #     up_down_pwr = msg["throttles"][1] * pwr["up_down_pwr"]
    #     self.mast_gimbal_pub.publish(Throttle(["mast_gimbal_y", "mast_gimbal_z"], [rot_pwr, up_down_pwr]))
    #
    #
    # def capture_panorama(self) -> None:
    #     rospy.logerr("Capturing panorama")
    #     goal = CapturePanoramaGoal()
    #     goal.angle = pi / 2
    #
    #     def feedback_cb(feedback: CapturePanoramaGoal) -> None:
    #         self.send(text_data=json.dumps({"type": "pano_feedback", "percent": feedback.percent_done}))
    #
    #     self.pano_client.send_goal(goal, feedback_cb=feedback_cb)
    #     finished = self.pano_client.wait_for_result(timeout=rospy.Duration(30))  # timeouts after 30 seconds
    #     if finished:
    #         rospy.logerr("Finished!")
    #         image = self.pano_client.get_result().panorama
    #         self.image_callback(image)
    #     else:
    #         rospy.logerr("CapturePanorama took too long!")
    #
    # def image_callback(self, msg):
    #     bridge = CvBridge()
    #     try:
    #         # Convert the image to OpenCV standard format
    #         cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #     except Exception as e:
    #         rospy.logerr("Could not convert image message to OpenCV image: " + str(e))
    #         return
    #
    #     # Save the image to a file (you could change 'png' to 'jpg' or other formats)
    #     image_filename = "~/Downloads/panorama.png"
    #     try:
    #         cv2.imwrite(os.path.expanduser(image_filename), cv_image)
    #         rospy.logerr("Saved image to {}".format(image_filename))
    #     except Exception as e:
    #         rospy.logerr("Could not save image: " + str(e))
    #
    # def heater_enable_service(self, msg):
    #     try:
    #         heater = msg["heater"]
    #         science_enable = rospy.ServiceProxy("science_enable_heater_" + heater, SetBool)
    #         result = science_enable(data=msg["enabled"])
    #     except rospy.ServiceException as e:
    #         print(f"Service init failed: {e}")
    #
    # def ish_heater_state_callback(self, msg):
    #     self.send(text_data=json.dumps({"type": "heater_states", "states": msg.state}))
    #
    # def auto_shutoff_toggle(self, msg):
    #     try:
    #         rospy.logerr(msg)
    #         result = self.heater_auto_shutoff_srv(data=msg["shutoff"])
    #         self.send(text_data=json.dumps({"type": "auto_shutoff", "success": result.success}))
    #     except rospy.ServiceException as e:
    #         print(f"Service call failed: {e}")
    #
    # def send_center(self):
    #     lat = rospy.get_param("gps_linearization/reference_point_latitude")
    #     long = rospy.get_param("gps_linearization/reference_point_longitude")
    #     self.send(text_data=json.dumps({"type": "center_map", "latitude": lat, "longitude": long}))
    #
    # def save_auton_waypoint_list(self, msg):
    #     AutonWaypoint.objects.all().delete()
    #     waypoints = []
    #     for w in msg["data"]:
    #         waypoints.append(
    #             AutonWaypoint(tag_id=w["id"], type=w["type"], latitude=w["lat"], longitude=w["lon"], name=w["name"])
    #         )
    #     AutonWaypoint.objects.bulk_create(waypoints)
    #     self.send(text_data=json.dumps({"type": "save_auton_waypoint_list", "success": True}))
    #     # Print out all of the waypoints
    #     for w in AutonWaypoint.objects.all():
    #         rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))
    #
    # def get_auton_waypoint_list(self, msg):
    #     waypoints = []
    #     for w in AutonWaypoint.objects.all():
    #         waypoints.append({"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type})
    #     self.send(text_data=json.dumps({"type": "get_auton_waypoint_list", "data": waypoints}))
    #
    # def save_basic_waypoint_list(self, msg):
    #     BasicWaypoint.objects.all().delete()
    #     waypoints = []
    #     for w in msg["data"]:
    #         waypoints.append(BasicWaypoint(drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]))
    #     BasicWaypoint.objects.bulk_create(waypoints)
    #     self.send(text_data=json.dumps({"type": "save_basic_waypoint_list", "success": True}))
    #     # Print out all of the waypoints
    #     for w in BasicWaypoint.objects.all():
    #         rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))
    #
    # def get_basic_waypoint_list(self, msg):
    #     waypoints = []
    #     for w in BasicWaypoint.objects.all():
    #         waypoints.append({"name": w.name, "drone": w.drone, "lat": w.latitude, "lon": w.longitude})
    #     self.send(text_data=json.dumps({"type": "get_basic_waypoint_list", "data": waypoints}))
    #
    # def imu_calibration_callback(self, msg) -> None:
    #     self.send(text_data=json.dumps({"type": "calibration_status", "system_calibration": msg.system_calibration}))
    #
    # def flight_attitude_listener(self):
    #     # threshold that must be exceeded to send JSON message
    #     threshold = 0.1
    #     map_to_baselink = SE3()
    #
    #     rate = rospy.Rate(10.0)
    #     while not rospy.is_shutdown():
    #         try:
    #             tf_msg = SE3.from_tf_tree(TF_BUFFER, "map", "base_link")
    #
    #             if tf_msg.is_approx(map_to_baselink, threshold):
    #                 rate.sleep()
    #                 continue
    #         except (
    #             tf2_ros.LookupException,
    #             tf2_ros.ConnectivityException,
    #             tf2_ros.ExtrapolationException,
    #         ):
    #             rate.sleep()
    #             continue
    #
    #         map_to_baselink = tf_msg
    #         rotation = map_to_baselink.rotation
    #         euler = euler_from_quaternion(rotation.quaternion)
    #         pitch = euler[0] * 180 / pi
    #         roll = euler[1] * 180 / pi
    #
    #         self.send(text_data=json.dumps({"type": "flight_attitude", "pitch": pitch, "roll": roll}))
    #
    #         rate.sleep()
    #
    # def arm_joint_callback(self, msg: JointState) -> None:
    #     # Set non-finite values to zero
    #     msg.position = [x if isfinite(x) else 0 for x in msg.position]
    #     self.send(text_data=json.dumps({"type": "fk", "positions": msg.position}))
    #
    # def science_spectral_callback(self, msg):
    #     self.send(
    #         text_data=json.dumps({"type": "spectral_data", "site": msg.site, "data": msg.data, "error": msg.error})
    #     )
    #
    # def download_csv(self, msg):
    #     username = os.getenv("USERNAME", "-1")
    #
    #     now = datetime.now(pytz.timezone("US/Eastern"))
    #     current_time = now.strftime("%m-%d-%Y_%H:%M:%S")
    #     spectral_data = msg["data"]
    #
    #     site_names = ["0", "1", "2"]
    #     index = 0
    #
    #     # add site letter in front of data
    #     for site_data in spectral_data:
    #         site_data.insert(0, f"Site {site_names[index]}")
    #         index = index + 1
    #
    #     time_row = ["Time", current_time]
    #     spectral_data.insert(0, time_row)
    #
    #     if username == "-1":
    #         rospy.logerr("username not found")
    #
    #     with open(os.path.join(f"/home/{username}/Downloads/spectral_data_{current_time}.csv"), "w") as f:
    #         csv_writer = csv.writer(f)
    #         csv_writer.writerows(spectral_data)


# def download_csv(msg: dict) -> None:
#     now = datetime.now(pytz.timezone("US/Eastern"))
#     current_time = now.strftime("%m-%d-%Y_%H:%M:%S")
#     spectral_data = msg["data"]
#
#     site_names = ["0", "1", "2"]
#     index = 0
#
#     for site_data in spectral_data:
#         site_data.insert(0, f"Site {site_names[index]}")
#         index = index + 1
#
#     time_row = ["Time", current_time]
#     spectral_data.insert(0, time_row)
#
#     downloads_path = Path.home() / "Downloads"
#     downloads_path.mkdir(exist_ok=True)
#
#     report_path = downloads_path / f"spectral_data_{current_time}.csv"
#     with report_path.open("w") as f:
#         csv_writer = csv.writer(f)
#         csv_writer.writerows(spectral_data)
