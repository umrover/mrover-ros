import json
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
from mrover.msg import Joystick

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.sub = rospy.Subscriber('/joystick', Joystick, self.joystick_callback)
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


    def handle_joystick_message(self, msg):
        """ Receive message from room group """
        fb = msg["forward_back"]
        lr = msg["left_right"]

        pub = rospy.Publisher('/joystick', Joystick)
        message = Joystick()
        message.forward_back = float(fb)
        message.left_right = float(lr)
        pub.publish(message)


    def joystick_callback(self, msg):
        forward_back = msg.forward_back
        left_right = msg.left_right

        #Scaling multiplier to adjust values if needed
        K = 1
        
        left = K * (forward_back + left_right)
        right = K * (forward_back - left_right)

        #Ensure values are [-1,1] for each motor
        if abs(left) > 1:
            left = left/abs(left)
        if abs(right) > 1:
            right = right/abs(right)

        # Send message to WebSocket
        self.send(text_data=json.dumps({
            "type": "wheel_cmd",
            "left": left,
            "right": right
        }))