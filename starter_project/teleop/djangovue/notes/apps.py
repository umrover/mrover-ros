from django.apps import AppConfig

import rospy


class NotesConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "notes"

    # def ready(self):
    #     rospy.init_node("teleop_starter")
