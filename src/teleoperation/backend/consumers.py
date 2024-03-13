import csv
from datetime import datetime, timezone
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
from geometry_msgs.msg import Twist
from mrover.msg import PDLB, ControllerState, GPSWaypoint, WaypointType, LED, StateMachineStateUpdate, Throttle, CalibrationStatus, SpectralGroup
from mrover.srv import EnableAuton
from sensor_msgs.msg import JointState, NavSatFix
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from util.SE3 import SE3

from backend.models import Waypoint

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
        self.imu_calibration = rospy.Subscriber('imu/calibration', CalibrationStatus, self.imu_calibration_callback)
        self.science_spectral = rospy.Subscriber('/science_spectral',SpectralGroup, self.science_spectral_callback)


        # Services
        self.laser_service = rospy.ServiceProxy("enable_mosfet_device", SetBool)
        self.toggle_uv = rospy.ServiceProxy("sa_enable_uv_bulb",SetBool)
        self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)
        self.enable_auton = rospy.ServiceProxy("enable_auton", EnableAuton)

        # ROS Parameters
        self.mappings = rospy.get_param("teleop/joystick_mappings")
        self.drive_config = rospy.get_param("teleop/drive_controls")
        self.max_wheel_speed = rospy.get_param("rover/max_speed")
        self.wheel_radius = rospy.get_param("wheel/radius")
        self.max_angular_speed = self.max_wheel_speed / self.wheel_radius

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.flight_thread = threading.Thread(target=self.flight_attitude_listener)
        self.flight_thread.start()
 



    def disconnect(self, close_code):
        self.pdb_sub.unregister()
        self.arm_moteus_sub.unregister()
        self.drive_moteus_sub.unregister()
        self.joint_state_sub.unregister()
        self.gps_fix.unregister()
        self.led_sub.unregister()
        self.nav_state_sub.unregister()
        self.imu_calibration.unregister()

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
            elif message["type"] == "sa_enable_uv_bulb":
                self.sa_uv_bulb_callback(message)
            elif message["type"] == "enable_white_leds":
                self.enable_white_leds_callback(message)
            elif message["type"] == "calibrate_service":
                self.calibrate_motors_callback(message)
            elif message["type"] == "auton_command":
                self.send_auton_command(message)
            elif message["type"] == "teleop_enabled":
                self.send_teleop_enabled(message)
            elif message["type"] == "auton_tfclient":
                self.auton_bearing()
            elif message["type"] == "mast_gimbal":
                self.mast_gimbal(message)
            elif message["type"] == "save_waypoint_list":
                self.save_waypoint_list(message)
            elif message["type"] == "get_waypoint_list":
                self.get_waypoint_list(message)
            elif message["type"] == "download_csv":
                self.download_csv(message)
        except Exception as e:
            rospy.logerr(e)

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

    def sa_uv_bulb_callback(self, msg):
        try:
            result = self.toggle_uv(data=msg["data"])
            self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def enable_white_leds_callback(self, msg):
        for i in range(0, 3):
            white_led_name = f"science_enable_white_led_{i}"
            led_srv = rospy.ServiceProxy(white_led_name, SetBool)
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
                    waypoint["tag_id"],
                    waypoint["latitude_degrees"],
                    waypoint["longitude_degrees"],
                    WaypointType(waypoint["type"]),
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

    def save_waypoint_list(self, msg):
        Waypoint.objects.all().delete()
        waypoints = []
        for w in msg["data"]:
            waypoints.append(
                Waypoint(tag_id=w["id"], type=w["type"], latitude=w["lat"], longitude=w["lon"], name=w["name"])
            )
        Waypoint.objects.bulk_create(waypoints)
        self.send(text_data=json.dumps({"type": "save_waypoint_list", "success": True}))
        # Print out all of the waypoints
        for w in Waypoint.objects.all():
            rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))

    def get_waypoint_list(self, msg):
        waypoints = []
        for w in Waypoint.objects.all():
            waypoints.append({"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type})
        self.send(text_data=json.dumps({"type": "get_waypoint_list", "data": waypoints}))


    def imu_calibration_callback(self, msg) -> None:
        self.send(text_data=json.dumps({
            'type': 'calibration_status',
            'system_calibration': msg.system_calibration
        }))

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

            self.send(text_data=json.dumps({
                'type': 'flight_attitude',
                'pitch': pitch,
                'roll': roll
            }))

            rate.sleep()

    def science_spectral_callback(self, msg):
        data = []
        error = []
        for spectral in msg.spectrals:
            data.append(spectral.data)
            error.append(spectral.error)

        self.send(text_data=json.dumps({
            'type': 'spectral_data',
            'data': data,
            'error': error
        }))

    def download_csv(self, msg):
        username = os.getenv("USERNAME", "-1")

        now = datetime.now(pytz.timezone('US/Eastern'))
        # now = now.astimezone()
        current_time = now.strftime("%m/%d/%Y-%H:%M")
        spectral_data = msg['data']

        site_names = ['A', 'B', 'C']
        index = 0

        # add site letter in front of data
        for site_data in spectral_data:
            site_data.insert(0, f'Site {site_names[index]}')
            index = index + 1 #use key value pair, there's no need to create a var

        time_row = ['Time', current_time]
        spectral_data.insert(0, time_row)

        if username == "-1":
            rospy.logerr("username not found")

        with open(os.path.join(f'/home/{username}/Downloads/spectral_data.csv'), 'w') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerows(spectral_data)        
