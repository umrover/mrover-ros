import json
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
from std_srvs.srv import SetBool, Trigger
from mrover.msg import PDB, ControllerState, Calibrated
from mrover.srv import EnableDevice
from std_msgs.msg import String
from sensor_msgs.msg import JointState, NavSatFix

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.accept()
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        # self.arm_moteus_sub = rospy.Subscriber('/calibration_checkbox', Calibrated, self.calibration_checkbox_callback_callback)
        self.laser_service = rospy.ServiceProxy("enable_mosfet_device",SetBool )
        self.gps_fix = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_fix_callback)
        # rospy.wait_for_service("enable_limit_switches")
        self.limit_switch_service = rospy.ServiceProxy("enable_limit_switches", EnableDevice)
        self.joint_state_sub = rospy.Subscriber('/drive_joint_data', JointState, self.joint_state_callback)
        self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)

    def disconnect(self, close_code):
        self.pdb_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
     
        if message["type"] == "enable_decive_srv":
            self.enable_device_callback(message)
        elif message["type"] == "disable_auton_led":
            self.disable_auton_led(message)
        elif message["type"] == "laser_service":
            self.enable_laser_callback(message)
        elif message["type"] == "calibrate_service":
            self.calibrate_motors_callback_callback(message)



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
            'type': 'arm_controller',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))

    def enable_laser_callback(self, msg):
        try:
            result = self.laser_service(data=msg["data"])
            self.send(text_data=json.dumps({
                'type': 'laser_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    
    def enable_device_callback(self, msg):
        try:
            result = self.calibrate_service(name=msg["name"], calibrate=msg["calibrate"])
            self.send(text_data=json.dumps({
                'type': 'calibrate_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def calibrate_motors_callback(self,msg):
         self.send(text_data=json.dumps({
            'type': 'calibrate_service',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))
        

  
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
