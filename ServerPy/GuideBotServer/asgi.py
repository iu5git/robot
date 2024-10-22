"""
ASGI config for toto project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/3.0/howto/deployment/asgi/
"""

# import os
# import django
# # from django.core.asgi import get_asgi_application
# from channels.routing import get_default_application

# os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'GuideBotServer.settings')
# django.setup()

# # application = get_asgi_application()
# application = get_default_application()


# import os

# from channels.routing import ProtocolTypeRouter
# from django.core.asgi import get_asgi_application

# os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'GuideBotServer.settings')

# application = ProtocolTypeRouter({
#     "http": get_asgi_application(),
#     # Just HTTP for now. (We can add other protocols later.)
# })


import os

import django
# from channels.http import AsgiHandler
from django.core.asgi import get_asgi_application
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.auth import AuthMiddlewareStack
import RobotWS.routing

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'GuideBotServer.settings')
# django.setup()

application = ProtocolTypeRouter({
  # "http": AsgiHandler(),
  "http": get_asgi_application(),
  "websocket": AuthMiddlewareStack(
        URLRouter(
            RobotWS.routing.websocket_urlpatterns
        )
    ),
})
