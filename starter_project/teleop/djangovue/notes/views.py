from django.shortcuts import render

# Create your views here.

from rest_framework import viewsets
from rest_framework import status
from rest_framework.response import Response
from notes.models import Joystick
from notes.serializers import JoystickSerializer

import rospy
import std_msgs.msg as msg
from mrover.msg import Joystick as JoystickMsg

class JoystickViewSet(viewsets.ModelViewSet):
    """
    API endpoint that allows items to be viewed or edited.
    """
    serializer_class = JoystickSerializer
    queryset = Joystick.objects.all()
    
    # def get_queryset(self):
    #     """
    #     Override get_queryset method to return only the last object
    #     """
    #     return [Joystick.objects.last()] if Joystick.objects.last() else []
    
    def create(self, request):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        
        pub = rospy.Publisher('joystick_pub', JoystickMsg)
        message = JoystickMsg()
        message.forward_back = float(request.data['forward_back'])
        message.left_right = float(request.data['left_right'])
        pub.publish(message)

        return Response(serializer.data, status=status.HTTP_201_CREATED)
