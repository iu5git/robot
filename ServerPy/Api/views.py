from urllib import response
from django.http import HttpResponse
from django.shortcuts import render
from Api.models import Robot
# V  DRF  V
from rest_framework import viewsets
from Api.serializers import RobotSerializer

class TestApiView(viewsets.ModelViewSet):
  queryset = Robot.objects.all()
  serializer_class = RobotSerializer
