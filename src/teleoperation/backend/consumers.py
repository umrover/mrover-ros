import json
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
from mrover.msg import PDB, ControllerState
from mrover.srv import EnableLaser

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.pdb_sub = rospy.Subscriber('/pdb_data', PDB, self.pdb_callback)
        self.arm_moteus_sub = rospy.Subscriber('/arm_controller_data', ControllerState, self.arm_controller_callback)
        self.laser_service = rospy.ServiceProxy("enable_mosfet_device", EnableLaser)
        self.accept()

    def disconnect(self, close_code):
        self.pdb_sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
        if message["type"] == "laser_service":
            self.enable_laser_callback(message)


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

    def enable_laser_callback(self, msg):
        try:
            result = self.laser_service(data=msg["data"])
            self.send(text_data=json.dumps({
                'type': 'laser_service',
                'result': result.success
            }))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        
    # def foo():
    #     s = rospy.Service('/joint_name', CalibrationCheckbox, self.calibration_checkbox_callback)

    # def calibration_checkbox_callback(self,msg):
    #      self.send(text_data=json.dumps({
    #         'type': 'calibration_checkbox',
    #         'name': msg.name,
    #         'state': msg.state,
    #         'error': msg.error
    #     }))
        

  