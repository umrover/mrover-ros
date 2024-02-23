"""
ASGI config for basestation_gui project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/4.2/howto/deployment/asgi/
"""
import os
from channels.auth import AuthMiddlewareStack
from channels.routing import ProtocolTypeRouter, URLRouter
from django.core.asgi import get_asgi_application

import teleoperation.urls

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "teleoperation.settings")

application = ProtocolTypeRouter(
    {
        # handle http/https requests
        "http": get_asgi_application(),
        # handle ws/wss requests
        "websocket": AuthMiddlewareStack(URLRouter(teleoperation.urls.websocket_urlpatterns)),
    }
)
