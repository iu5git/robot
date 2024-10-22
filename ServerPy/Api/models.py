from django.conf import settings
from django.db import models
from django.utils import timezone
from django.db.models import Count

class Robot(models.Model):
  TEST = "test"
  NAME_CHOICES = [
    (TEST, 'test'),
  ]

  name = models.CharField(
    max_length=4,
    choices=NAME_CHOICES,
    default=TEST,
  )

  def __str__(self):
    return "Robot-" + self.id

  class Meta:
    verbose_name = 'Робот'
    verbose_name_plural = 'Роботы'

