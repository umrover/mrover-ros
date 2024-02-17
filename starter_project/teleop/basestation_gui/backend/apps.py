from django.apps import AppConfig

import rospy


class BackendConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "backend"

    def ready(self):
        rospy.init_node("teleop_starter")
