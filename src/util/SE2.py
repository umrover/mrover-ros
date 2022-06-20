from geometry_msgs.msg import Pose2D


class SE2(Pose2D):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
