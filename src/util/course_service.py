from __future__ import annotations

from typing import List, Tuple

import rospy
import tf2_ros
from mrover.msg import Waypoint, Course
from mrover.srv import PublishCourse
from util.SE3 import SE3


class CourseService(rospy.ServiceProxy):
    SERVICE_NAME = "course_service"

    tf_broadcaster: tf2_ros.StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()

    def __init__(self, **kwargs):
        super().__init__(self.SERVICE_NAME, PublishCourse, **kwargs)

    def call(self, waypoints: List[Tuple[Waypoint, SE3]]):
        all_waypoint_info = []
        for waypoint_info, pose in waypoints:
            all_waypoint_info.append(waypoint_info)
            pose.publish_to_tf_tree(self.tf_broadcaster, "map", waypoint_info.tf_id)
        course = Course(waypoints=all_waypoint_info)
        super().call(course)

    def __call__(self, waypoints: List[Tuple[Waypoint, SE3]]):
        self.call(waypoints)
