from rest_framework import serializers
from notes.models import Joystick

# Serializer tells the REST API what needs to be returned and how to convert to JSON

class JoystickSerializer(serializers.ModelSerializer):
    forward_back = serializers.FloatField()
    left_right = serializers.FloatField()
    class Meta:
        model = Joystick
        fields = ['forward_back', 'left_right']