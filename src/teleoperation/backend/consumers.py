import json
from channels.generic.websocket import JsonWebsocketConsumer
import rospy
from std_srvs.srv import SetBool, Trigger
from mrover.msg import PDB, ControllerState, AutonCommand, GPSWaypoint, LED, StateMachineStateUpdate
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
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        self.drive_moteus_sub = rospy.Subscriber('/drive_controller_data', ControllerState, self.drive_controller_callback)
        # self.calibration_sub = rospy.Subscriber('/calibration_checkbox', Calibrated, self.calibration_checkbox_callback)
        self.laser_service = rospy.ServiceProxy("enable_mosfet_device",SetBool )
        self.gps_fix = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_fix_callback)
        # self.limit_switch_service = rospy.ServiceProxy("enable_limit_switches", EnableDevice)
        self.joint_state_sub = rospy.Subscriber('/drive_joint_data', JointState, self.joint_state_callback)
        self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)
        self.led_sub = rospy.Subscriber("/led", LED, self.led_callback)
        self.nav_state_sub = rospy.Subscriber("/nav_state", StateMachineStateUpdate, self.nav_state_callback)


    def disconnect(self, close_code):
        self.pdb_sub.unregister()
        self.arm_moteus_sub.unregister()
        self.drive_moteus_sub.unregister()
        # self.calibration_sub.unregister()
        self.joint_state_sub.unregister()
        self.gps_fix.unregister()
        self.led_sub.unregister()
        self.nav_state_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
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


    def handle_joystick_message(self, msg):
        mappings = rospy.get_param("teleop/joystick_mappings")
        drive_config = rospy.get_param("teleop/drive_controls")
        max_wheel_speed = rospy.get_param("rover/max_speed")
        wheel_radius = rospy.get_param("wheel/radius")
        max_angular_speed = max_wheel_speed / wheel_radius

        twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg["axes"][mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(
            msg["axes"][mappings["forward_back"]] * drive_config["forward_back"]["multiplier"], 0.05
        )

        # Convert from [0,1] to [0, max_wheel_speed] and apply dampen
        linear *= max_wheel_speed * dampen

        # Deadzones for each axis
        left_right = (
            deadzone(
                msg["axes"][mappings["left_right"]] * drive_config["left_right"]["multiplier"], 0.4
            )
            if drive_config["left_right"]["enabled"]
            else 0
        )
        twist = quadratic(
            deadzone(msg["axes"][mappings["twist"]] * drive_config["twist"]["multiplier"], 0.1)
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
            'forward_back': msg["axes"][mappings["forward_back"]],
            'twist': twist,
            'dampen': dampen,
            'pan': msg["axes"][mappings["pan"]],
            'tilt': msg["axes"][mappings["tilt"]],
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
            'calibrated': msg.calibrated
        }))

    def arm_controller_callback(self, msg): 
        self.send(text_data=json.dumps({
            'type': 'arm_moteus',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))

    def drive_controller_callback(self, msg): 
        self.send(text_data=json.dumps({
            'type': 'drive_moteus',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))

    def enable_laser_callback(self, msg):
        try:
            result = self.laser_service(data=msg['data'])
            self.send(text_data=json.dumps({
                'type': 'laser_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    
    def enable_device_callback(self, msg):
        try:
            result = self.calibrate_service()
            self.send(text_data=json.dumps({
                'type': 'calibrate_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def calibrate_motors_callback(self,msg):
         self.send(text_data=json.dumps({
            'type': 'calibrate_service',
            'name': msg['name'],
            'state': msg['state'],
            'error': msg['error']
        }))
        
  
    def disable_auton_led(self):
        led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=1)
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

    def send_auton_command(self, msg):
        cmd_pub = rospy.Publisher("/auton/command", AutonCommand, queue_size=100)
        waypoints = []
        for w in msg['waypoints']:
            waypoints.append(GPSWaypoint(w['latitude_degrees'], w['longitude_degrees'], w['tag_id'], w['type']))
        
        message = AutonCommand(msg['is_enabled'], waypoints)
        cmd_pub.publish(message)

    def send_teleop_enabled(self, msg):
        teleop_pub = rospy.Publisher("/teleop_enabled", Bool, queue_size=1)
        teleop_pub.publish(msg['data'])

    def led_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'led',
            'red': msg.red,
            'green': msg.green,
            'blue': msg.blue,
            'is_blinking': msg.is_blinking
        }))

    def nav_state_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'nav_state',
            'state': msg.state
        }))

    def auton_bearing(self, msg):
        listener = tf.TransformListener()
        trans, rot = listener.lookupTransform('map', 'base_link', rospy.Time(O))
        self.send(text_data=json.dumps({
            'type': 'auton_tfclient',
            'rotation': rot,
        }))

    def mast_gimbal(self, msg):
        pub = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)
        pub.publish(Throttle(["mast_gimbal_x", "mast_gimbal_y"], msg["throttles"]))