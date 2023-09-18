from django.apps import AppConfig

import rospy
# from mrover.msg import Joystick


class NotesConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "notes"

    def ready(self):
        # from notes.consumers import GUIConsumer
        rospy.init_node("teleop_starter")
        # rospy.Subscriber('/joystick_pub', Joystick, GUIConsumer.joystick_callback)
