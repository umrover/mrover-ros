from django.apps import AppConfig

import rospy


class BackendConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "backend"

    def ready(self):
        # Disabling signals prevents hanging when you Ctrl-C the server
        rospy.init_node("teleop", disable_signals=True)
