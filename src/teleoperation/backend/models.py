from django.db import models


# Create your models here.
class AutonWaypoint(models.Model):
    id = models.IntegerField(unique=True, primary_key=True)
    tag_id = models.IntegerField()
    # Type based on WaypointType.msg
    type = models.IntegerField()
    created_at = models.DateTimeField(auto_now_add=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)


class BasicWaypoint(models.Model):
    id = models.IntegerField(unique=True, primary_key=True)
    drone = models.BooleanField()
    created_at = models.DateTimeField(auto_now_add=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)
