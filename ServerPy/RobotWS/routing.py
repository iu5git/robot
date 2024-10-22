# Robot/routing.py
from django.urls import re_path

from RobotWS.Robot.consumer import RobotConsumer

websocket_urlpatterns = [
    re_path(r'ws/robot/(?P<room_name>\w+)/$', RobotConsumer.as_asgi()),
]
