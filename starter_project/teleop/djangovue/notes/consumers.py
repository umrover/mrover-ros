import json
from channels.generic.websocket import AsyncJsonWebsocketConsumer

import rospy
from mrover.msg import Joystick

class GUIConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        self.room_name = 'drive-controls'

        self.sub = rospy.Subscriber('/joystick_pub', Joystick, self.joystick_callback)

        # Join room group
        await self.channel_layer.group_add(
            self.room_name,
            self.channel_name
        )
        await self.accept()

        await self.channel_layer.group_send(self.room_name, {
                'type': 'joystick_update',
                'forward_back': 1.23,
                'left_right': 4.56
            })

    async def disconnect(self, close_code):
        # Leave room group
        await self.channel_layer.group_discard(
            self.room_name,
            self.channel_name
        )

        self.sub.unregister()

    async def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        response = json.loads(text_data)
        fb = response.get("forward_back", None)
        lr = response.get("left_right", None)
        # Send message to room group
        await self.channel_layer.group_send(self.room_name, {
            'type': 'joystick_update',
            'forward_back': fb,
            'left_right': lr
        })


    #Receives message from room group --- follows the 'type' key above
    async def joystick_update(self, event):
        """ Receive message from room group """
        fb = event["forward_back"]
        lr = event["left_right"]

        pub = rospy.Publisher('/joystick_pub', Joystick)
        message = Joystick()
        message.forward_back = float(fb)
        message.left_right = float(lr)
        pub.publish(message)

        # Send message to WebSocket
        await self.send(text_data=json.dumps({
            'forward_back': fb,
            'left_right': lr
        }))

    def joystick_callback(self, msg):
        forward_back = msg.forward_back
        left_right = msg.left_right

        # Send message to WebSocket
        self.send(text_data=json.dumps({
            'forward_back': forward_back,
            'left_right': left_right
        }))