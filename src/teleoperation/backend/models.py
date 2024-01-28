from django.db import models


# Create your models here.
class Waypoint(models.Model):
    id = models.IntegerField(unique=True, primary_key=True)
    tag_id = models.IntegerField()
    # Type should be in {waypoint, }
    type = models.IntegerField()
    created_at = models.DateTimeField(auto_now_add=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)