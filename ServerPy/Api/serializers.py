from Api.models import Robot
from rest_framework import serializers

class RobotSerializer(serializers.ModelSerializer):
  class Meta:
    # Модель, которую мы сериализуем
    model = Robot
    # Поля, которые мы сериализуем
    fields = ["name"]
