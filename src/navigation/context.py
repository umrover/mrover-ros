from __future__ import annotations
from pdb import post_mortem
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import mrover.msg
import mrover.srv
from util.SE3 import SE3
from visualization_msgs.msg import Marker
from typing import ClassVar, Optional, List, Tuple
import numpy as np
from dataclasses import dataclass
from mrover.msg import Waypoint, GPSWaypoint, EnableAuton
import pymap3d


# read required parameters, if they don't exist an error will be thrown
REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")

tf_broadcaster: tf2_ros.StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()


@dataclass
class Gate:
    post1: np.ndarray
    post2: np.ndarray


@dataclass
class Rover:
    ctx: Context

    def get_pose(self) -> SE3:
        return SE3.from_tf_tree(self.ctx.tf_buffer, parent_frame="map", child_frame="base_link")

    def send_drive_command(self, twist: Twist):
        self.ctx.vel_cmd_publisher.publish(twist)

    def send_drive_stop(self):
        self.send_drive_command(Twist())

    def get_pose_with_time(self):
        return SE3.from_tf_time(self.ctx.tf_buffer, parent_frame="map", child_frame="base_link")


@dataclass
class Environment:
    """
    Context class to represent the rover's envrionment
    Information such as locations of fiducials or obstacles
    """

    ctx: Context
    NO_FIDUCIAL: ClassVar[int] = -1

    def get_fid_pos(self, fid_id: int, frame: str = "map") -> Optional[np.ndarray]:
        """
        Retrieves the pose of the given fiducial ID from the TF tree
        if it exists, otherwise returns None
        """
        try:
            fid_pose = SE3.from_tf_tree(self.ctx.tf_buffer, parent_frame="map", child_frame=f"fiducial{fid_id}")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        return fid_pose.position

    def current_fid_pos(self) -> Optional[np.ndarray]:
        """
        Retrieves the position of the current fiducial (and we are looking for it)
        """
        assert self.ctx.course
        current_waypoint = self.ctx.course.current_waypoint()
        if current_waypoint is None or not self.ctx.course.look_for_post():
            return None

        return self.get_fid_pos(current_waypoint.fiducial_id)

    def current_gate(self) -> Optional[Gate]:
        """
        retrieves the position of the gate (if we know where it is, and we are looking for one)
        """
        if self.ctx.course:
            current_waypoint = self.ctx.course.current_waypoint()
            if current_waypoint is None or not self.ctx.course.look_for_gate():
                return None

            post1 = self.get_fid_pos(current_waypoint.fiducial_id)
            post2 = self.get_fid_pos(current_waypoint.fiducial_id + 1)
            if post1 is None or post2 is None:
                return None

            return Gate(post1[0:2], post2[0:2])
        else:
            return None


@dataclass
class Course:
    ctx: Context
    course_data: mrover.msg.Course
    # Currently active waypoint
    waypoint_index: int = 0

    def increment_waypoint(self):
        self.waypoint_index += 1

    def waypoint_pose(self, wp_idx: int) -> SE3:
        """
        Gets the pose of the waypoint with the given index
        """
        waypoint_frame = self.course_data.waypoints[wp_idx].tf_id
        return SE3.from_tf_tree(self.ctx.tf_buffer, parent_frame="map", child_frame=waypoint_frame)

    def current_waypoint_pose(self):
        """
        Gets the pose of the current waypoint
        """
        return self.waypoint_pose(self.waypoint_index)

    def current_waypoint(self) -> Optional[mrover.msg.Waypoint]:
        """
        Returns the currently active waypoint

        :param ud:  State machine user data
        :return:    Next waypoint to reach if we have an active course
        """
        if self.course_data is None or self.waypoint_index >= len(self.course_data.waypoints):
            return None
        return self.course_data.waypoints[self.waypoint_index]

    def look_for_gate(self) -> bool:
        """
        Returns whether the currently active waypoint (if it exists) indicates
        that we should go to a gate
        """
        waypoint = self.current_waypoint()
        if waypoint is not None:
            return waypoint.type.val == mrover.msg.WaypointType.GATE
        else:
            return False

    def look_for_post(self) -> bool:
        """
        Returns whether the currently active waypoint (if it exists) indicates
        that we should go to a post
        """
        waypoint = self.current_waypoint()
        if waypoint is not None:
            return waypoint.type.val == mrover.msg.WaypointType.POST
        else:
            return False

    def is_complete(self):
        return self.waypoint_index == len(self.course_data.waypoints)


def setup_course(ctx: Context, waypoints: List[Tuple[Waypoint, SE3]]) -> Course:
    all_waypoint_info = []
    for waypoint_info, pose in waypoints:
        all_waypoint_info.append(waypoint_info)
        pose.publish_to_tf_tree(tf_broadcaster, "map", waypoint_info.tf_id)
    # make the course out of just the pure waypoint objects which is the 0th elt in the tuple
    return Course(ctx=ctx, course_data=mrover.msg.Course([waypoint[0] for waypoint in waypoints]))


def convert(waypoint: GPSWaypoint) -> Waypoint:
    """
    Converts a GPSWaypoint into a "Waypoint" used for publishing to the CourseService.
    """

    # Create odom position based on GPS latitude and longitude
    odom = np.array(
        pymap3d.geodetic2enu(
            waypoint.latitude_degrees, waypoint.longitude_degrees, 0.0, REF_LAT, REF_LON, 0.0, deg=True
        )
    )
    # zero the z-coordinate of the odom because even though the altitudes are set to zero,
    # two points on a sphere are not going to have the same z coordinate
    # navigation algorithmns currently require all coordinates to have zero as the z coordinate
    odom[2] = 0

    return Waypoint(fiducial_id=waypoint.id, tf_id=f"course{waypoint.id}", type=waypoint.type), SE3(position=odom)


def convert_and_get_course(ctx: Context, data: EnableAuton) -> Course:
    waypoints = [convert(waypoint) for waypoint in data.waypoints]
    return setup_course(ctx, waypoints)


class Context:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    course_listener: rospy.Subscriber

    # Use these as the primary interfaces in states
    course: Optional[Course]
    rover: Rover
    env: Environment
    disable_requested: bool

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher("nav_vis", Marker, queue_size=1)
        self.enable_auton_service = rospy.Service("enable_auton", mrover.srv.PublishEnableAuton, self.recv_enable_auton)
        self.course = None
        self.rover = Rover(self)
        self.env = Environment(self)
        self.disable_requested = False

    def recv_enable_auton(self, req: mrover.srv.PublishEnableAutonRequest) -> mrover.srv.PublishEnableAutonResponse:
        enable_msg = req.enableMsg
        if enable_msg.enable:
            self.course = convert_and_get_course(self, enable_msg)
        else:
            self.disable_requested = True
        return mrover.srv.PublishEnableAutonResponse(True)
