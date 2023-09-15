from rest_framework import routers
from django.urls import path, include
from notes import views
from notes.consumers import GUIConsumer

router = routers.DefaultRouter()
router.register(r'joystick', views.JoystickViewSet, basename='joystick')
urlpatterns = [
    path('', include(router.urls)),
]

# websocket_urlpatterns = [
#     path("ws/play/", GUIConsumer.as_asgi())
# ]