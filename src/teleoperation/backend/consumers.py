import json
from channels.generic.websocket import JsonWebsocketConsumer
import rospy
from std_srvs.srv import SetBool, Trigger
from mrover.msg import PDLB, ControllerState, AutonCommand, GPSWaypoint, LED, StateMachineStateUpdate, Throttle
from mrover.srv import EnableDevice
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, NavSatFix
from geometry_msgs.msg import Twist
from math import copysign
import typing
import tf


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
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        self.led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=1)
        self.auton_cmd_pub = rospy.Publisher("/auton/command", AutonCommand, queue_size=100)
        self.teleop_pub = rospy.Publisher("/teleop_enabled", Bool, queue_size=1)
        self.mast_gimbal_pub = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)

        # Subscribers
        self.pdb_sub = rospy.Subscriber("/pdb_data", PDLB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber("/arm_controller_data", ControllerState, self.arm_controller_callback)
        self.drive_moteus_sub = rospy.Subscriber( "/drive_controller_data", ControllerState, self.drive_controller_callback)
        self.gps_fix = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_fix_callback)
        self.joint_state_sub = rospy.Subscriber("/drive_joint_data", JointState, self.joint_state_callback)
        self.led_sub = rospy.Subscriber("/led", LED, self.led_callback)
        self.nav_state_sub = rospy.Subscriber("/nav_state", StateMachineStateUpdate, self.nav_state_callback)

        # Services
        self.laser_service = rospy.ServiceProxy("enable_mosfet_device", SetBool)
        self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)

        # ROS Parameters
        self.mappings = rospy.get_param("teleop/joystick_mappings")
        self.drive_config = rospy.get_param("teleop/drive_controls")
        self.max_wheel_speed = rospy.get_param("rover/max_speed")
        self.wheel_radius = rospy.get_param("wheel/radius")
        self.max_angular_speed = self.max_wheel_speed / self.wheel_radius

        

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
            elif message["type"] == "calibrate_service":
                self.calibrate_motors_callback(message)
            elif message["type"] == "auton_command":
                self.send_auton_command(message)
            elif message["type"] == "teleop_enabled":
                self.send_teleop_enabled(message)
            elif message["type"] == "mast_gimbal":
                self.mast_gimbal(message)
        except rospy.ROSException as e:
            rospy.logerr(e)

    def handle_joystick_message(self, msg):
     
        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg["axes"][self.mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(msg["axes"][self.mappings["forward_back"]] * self.drive_config["forward_back"]["multiplier"], 0.05)

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
        msg.position = [x*self.wheel_radius for x in msg.position]
        msg.velocity = [x*self.wheel_radius for x in msg.velocity]
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
        waypoints = []
        for w in msg["waypoints"]:
            waypoints.append(GPSWaypoint(w["latitude_degrees"], w["longitude_degrees"], w["tag_id"], w["type"]))

        message = AutonCommand(msg["is_enabled"], waypoints)
        self.auton_cmd_pub.publish(message)

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

    def auton_bearing(self, msg):
        listener = tf.TransformListener()
        trans, rot = listener.lookupTransform("map", "base_link", rospy.Time(O))
        self.send(
            text_data=json.dumps(
                {
                    "type": "auton_tfclient",
                    "rotation": rot,
                }
            )
        )

    def mast_gimbal(self, msg):
        pwr = rospy.get_param("teleop/mast_gimbal_power")
        rot_pwr = msg["throttles"][0] * pwr["rotation_pwr"]
        up_down_pwr = msg["throttles"][1] * pwr["up_down_pwr"]
        self.mast_gimbal_pub.publish(Throttle(["mast_gimbal_x", "mast_gimbal_y"], [rot_pwr, up_down_pwr]))
