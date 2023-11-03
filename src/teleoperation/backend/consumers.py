import json
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
from mrover.msg import PDB, ControllerState
from mrover.srv import EnableDevice
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.accept()
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        # rospy.wait_for_service("enable_limit_switches")
        self.limit_switch_service = rospy.ServiceProxy("enable_limit_switches", EnableDevice)
        self.joint_state_sub = rospy.Subscriber('/drive_joint_data', JointState, self.joint_state_callback)

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


    def pdb_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'pdb',
            'temperatures': msg.temperatures,
            'currents': msg.currents
        }))

    def arm_controller_callback(self, msg): 
        self.send(text_data=json.dumps({
            'type': 'arm_controller',
            'name': msg.name,
            'state': msg.state,
            'error': msg.error
        }))

    def enable_device_callback(self, msg):
        try:
            result = self.limit_switch_service(name=msg["name"], enable=msg["enable"])
            self.send(text_data=json.dumps({
                'type': 'enable_device_srv',
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
        # self.send(text_data=json.dumps({
        #     'type': 'joint_state',
        #     'name': msg.name,
        #     'position': msg.position,
        #     'velocity': msg.velocity,
        #     'effort': msg.effort
        # }))
        self.send(text_data=json.dumps({
            'type': 'joint_state'
        }))
