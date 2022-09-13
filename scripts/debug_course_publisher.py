#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from typing import List, Tuple

from mrover.msg import Course, Waypoint
import mrover.srv

"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""


def add_waypoint(frame_id: str, fid_id: int) -> Waypoint:
    """
    Generates a waypoint message with the given waypoint name and
    fiducial ID
    """
    w = Waypoint()
    w.fiducial_id = fid_id
    w.tf_id = frame_id
    return w


def get_waypoint_tf(frame_id: str, x: float, y: float):
    """
    Generates a transform for describing the cartesian position
    of the waypoint in the map
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = frame_id
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.w = 1
    return t


def send_waypoints(waypoints: List[Tuple[str, int, int, int]]) -> Waypoint:
    """
    Publishes the array of waypoint tuples over TF
    """
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfs = [get_waypoint_tf(*waypoint[:3]) for waypoint in waypoints]
    tf_broadcaster.sendTransform(tfs)


if __name__ == "__main__":
    rospy.init_node("debug_course_publisher")
    publish_course = rospy.ServiceProxy("course_service", mrover.srv.PublishCourse)

    # These are the waypoints (name, x, y, fiducial)
    waypoints = [
        ("course1", 3, 3, 0),
        # ("course1", -3, -3, -1),
        # ("course2", -5, -5, 0)
    ]

    # This executes an RPC that gives the state machine the course
    c = Course()
    for w in waypoints:
        c.waypoints.append(add_waypoint(w[0], w[3]))
    publish_course(c)

    # Lastly, lets publish their positions over TF to mimic what auton GUI does
    send_waypoints(waypoints)
    rospy.spin()
