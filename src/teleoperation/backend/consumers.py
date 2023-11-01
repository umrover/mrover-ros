import json
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
from mrover.msg import PDB, ControllerState
from sensor_msgs.msg import JointState

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        self.joint_state_sub = rospy.Subscriber('/arm_joint_data', JointState, self.joint_state_callback)
        self.accept()

    def disconnect(self, close_code):
        self.pdb_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)

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

    def joint_state_callback(self, msg):
        self.send(text_data=json.dumps({
            'type': 'joint_state',
            'name': msg.name,
            'position': msg.position,
            'velocity': msg.velocity,
            'effort': msg.effort
        }))