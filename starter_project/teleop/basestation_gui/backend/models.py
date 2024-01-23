from django.db import models

# Create your models here.

class Joystick(models.Model):
    forward_back = models.FloatField()
    left_right = models.FloatField()
