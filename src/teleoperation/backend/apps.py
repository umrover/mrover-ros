from django.apps import AppConfig

import rospy

has_init = False


class BackendConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "backend"

    def ready(self):
        global has_init
        if not has_init:
            rospy.init_node("teleop")
            has_init = True
        else:
            rospy.logwarn("Node already initialized")
