import json
from math import nan
from channels.generic.websocket import JsonWebsocketConsumer
import rospy
from std_srvs.srv import SetBool, Trigger
from mrover.msg import PDB, ControllerState, Calibrated
from mrover.srv import EnableDevice, AdjustMotor
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Joy, NavSatFix
from geometry_msgs.msg import Twist
from math import copysign
import typing

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
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        self.calibration_checkbox_sub = rospy.Subscriber('/calibration_checkbox', Calibrated, self.calibration_checkbox_callback)
        self.laser_service = rospy.ServiceProxy("laser_service",SetBool)
        # rospy.wait_for_service("enable_limit_switches")
        # self.limit_switch_service = rospy.ServiceProxy("enable_limit_switches", EnableDevice)
        # self.ra_mode_service = rospy.ServiceProxy("change_ra_mode", )
        self.arm_adjust_service = rospy.ServiceProxy("arm_adjust",AdjustMotor )
        self.gps_fix = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_fix_callback)
        self.joint_state_sub = rospy.Subscriber('/drive_joint_data', JointState, self.joint_state_callback)
        self.joy_sub = rospy.Subscriber('/joystick', Joy, self.handle_joystick_message)
        self.arm_joystick_sub = rospy.Subscriber('/xbox/ra_control', Joy, self.handle_arm_message)

    def disconnect(self, close_code):
        self.pdb_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """

        message = json.loads(text_data)
        try:
            if message["type"] == "enable_decive_srv":
                self.enable_device(message)
            elif message["type"] == "disable_auton_led":
                self.disable_auton_led(message)
            elif message["type"] == "laser_service":
                self.enable_laser(message)
            elif message["type"] == "calibrate_motors":
                self.calibrate_motors(message)
            elif message["type"] == "arm_adjust":
                self.arm_adjust(message)
            elif message['type'] == "joystick_values":
                self.handle_joystick_message(message)
            elif message['type'] == "change_ra_mode":
                self.handle_joystick_message(message)
            elif message['type'] == "arm_values":
                self.handle_arm_message(message)
            elif message['type'] == 'enable_device_srv':
                self.limit_switch(message)
        except Exception as e:
            rospy.logerr(e)

    def handle_arm_message(self,msg):
        self.ra_config = rospy.get_param("teleop/ra_controls")  #could be different name
        self.xbox_mappings = rospy.get_param("teleop/xbox_mappings")
        self.sa_config = rospy.get_param("teleop/sa_controls")
        self.RA_NAMES = [
            "joint_a",
            "joint_b",
            "joint_c",
            "joint_de_pitch",
            "joint_de_yaw",
            "allen_key",
            "gripper"
        ]
        

        self.ra_slow_mode = False
        self.ra_cmd_pub = rospy.Publisher("arm_controller", JointState, queue_size=100)
        if msg.arm_mode == "arm_disabled":
            x= 1
        elif msg.arm_mode == "ik":
            x = 1
        elif msg.arm_mode == "position":
            self.arm_position_cmd = JointState(
 
             name=[name for name in self.RA_NAMES],
            position=[nan for _ in self.RA_NAMES],
            )
            self.ra_cmd_pub.publish(self.arm_position_cmd)
        

        elif msg.arm_mode == "velocity":
            self.arm_velocity_cmd = JointState(
 
                name=[name for name in self.RA_NAMES],
                velocity=[0.0 for _ in self.RA_NAMES],
            )
            self.ra_cmd_pub.publish(self.arm_velocity_cmd)
        
        
        elif msg.arm_mode == "throttle":
            self.arm_throttle_cmd = JointState(
 
             name=[name for name in self.RA_NAMES],
            throttle=[0.0 for _ in self.RA_NAMES],
        
        )
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
            self.arm_throttle_cmd.throttle = [
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
                    self.arm_throttle_cmd.velocity[i] *= self.ra_config[name]["slow_mode_multiplier"]
                if self.ra_config[name]["invert"]:
                    self.arm_throttle_cmd.velocity[i] *= -1

            self.ra_cmd_pub.publish(self.arm_throttle_cmd)


    def handle_joystick_message(self, msg):
        mappings = rospy.get_param("teleop/joystick_mappings")
        drive_config = rospy.get_param("teleop/drive_controls")
        max_wheel_speed = rospy.get_param("rover/max_speed")
        wheel_radius = rospy.get_param("wheel/radius")
        max_angular_speed = max_wheel_speed / wheel_radius

        twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg.axes[mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(
            msg.axes[mappings["forward_back"]] * drive_config["forward_back"]["multiplier"], 0.05
        )

        # Convert from [0,1] to [0, max_wheel_speed] and apply dampen
        linear *= max_wheel_speed * dampen

        # Deadzones for each axis
        left_right = (
            deadzone(
                msg.axes[mappings["left_right"]] * drive_config["left_right"]["multiplier"], 0.4
            )
            if drive_config["left_right"]["enabled"]
            else 0
        )
        twist = quadratic(
            deadzone(msg.axes[mappings["twist"]] * drive_config["twist"]["multiplier"], 0.1)
        )

        angular = twist + left_right

        # Same as linear but for angular speed
        angular *= max_angular_speed * dampen
        # Clamp if both twist and left_right are used at the same time
        if abs(angular) > max_angular_speed:
            angular = copysign(max_angular_speed, angular)
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        twist_pub.publish(twist_msg)

        self.send(text_data=json.dumps({
            'type': 'joystick',
            'left_right':left_right,
            'forward_back': msg.axes[mappings["forward_back"]],
            'twist': twist,
            'dampen': dampen,
            'pan': msg.axes[mappings["pan"]],
            'tilt': msg.axes[mappings["tilt"]],
            }))

    def pdb_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'pdb',
            'temperatures': msg.temperatures,
            'currents': msg.currents
        }))

    def calibration_checkbox_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'calibration_status',
            'names': msg.names,
            'calibrated': msg.calibrated,
        }))

    def arm_controller_callback(self, msg): 
        self.send(text_data=json.dumps({
            'type': 'arm_controller',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))

    def enable_laser(self, msg):
        try:
            result = self.laser_service(msg['data'])
            self.send(text_data=json.dumps({
                'type': 'laser_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            rospy.logerr(e)
    
    def enable_device(self, msg):
        try:
            result = self.calibrate_service(name=msg['name'], calibrate=msg['calibrate'])
            self.send(text_data=json.dumps({
                'type': 'calibrate_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def limit_switch(self, msg):
        joints = ["joint_a","joint_b","joint_c","joint_de_pitch","joint_de_roll","allen_key","gripper"]
        arm_enable_limit_switch = "arm_enable_limit_switch_"
        fail = []
        for joint in joints:
            name = arm_enable_limit_switch+joint
            self.limit_switch_service = rospy.ServiceProxy(name, SetBool)
            try:
                result = self.limit_switch_service(data = msg['data'])
                if not result.success:
                    fail.append(joint)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
        self.send(text_data=json.dumps({
                    'type': 'enable_device_srv',
                    'result': fail
                }))

    def calibrate_motors(self,msg):
        joints = ["joint_a","joint_b","joint_c","joint_de_pitch","joint_de_roll","allen_key","gripper"]
        arm_calibrate = "arm_calibrate_"
        fail = []
        for joint in joints:
            name = arm_calibrate+joint
            self.calibrate_service = rospy.ServiceProxy(name, Trigger)
            try:
                result = self.calibrate_service()
                if not result.success:
                    fail.append(joint)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
        self.send(text_data=json.dumps({
                    'type': 'calibrate_motors',
                    'result': fail
                }))
    
    def arm_adjust(self,msg):
        try:
            result = self.arm_adjust_service(name=msg['name'], value=msg['value'])
            self.send(text_data=json.dumps({
                'type': 'arm_adjust',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
  
    def disable_auton_led(self, msg):
        led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=100)
        message = String()
        message.data = "off"
        led_pub.publish(message)
        
    def joint_state_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'joint_state',
            'name': msg.name,
            'position': msg.position,
            'velocity': msg.velocity,
            'effort': msg.effort
        }))
    
    def gps_fix_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'nav_sat_fix',
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }))
