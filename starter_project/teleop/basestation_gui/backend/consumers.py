import json
from channels.generic.websocket import JsonWebsocketConsumer

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.accept()

    def disconnect(self, close_code):
        pass

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """
        pass