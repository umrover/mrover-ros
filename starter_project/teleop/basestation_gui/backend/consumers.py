import json
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
from mrover.msg import Joystick

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.accept()

    def disconnect(self, close_code):
        self.sub.unregister()

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        message = json.loads(text_data)
        if message['type'] == "joystick":
            self.handle_joystick_message(message)