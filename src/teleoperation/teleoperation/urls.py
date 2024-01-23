from django.contrib import admin
from django.urls import path

from backend.consumers import GUIConsumer

urlpatterns = [
    path('admin/', admin.site.urls)
]

# lists the websocket routes
websocket_urlpatterns = [
    path('ws/gui', GUIConsumer.as_asgi())
]
