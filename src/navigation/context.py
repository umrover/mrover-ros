from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Optional, List, Tuple

import numpy as np
import pymap3d

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, MapMetaData
from mrover.msg import Waypoint, GPSWaypoint, EnableAuton, WaypointType, GPSPointList
from std_msgs.msg import Time, Bool
from visualization_msgs.msg import Marker

from util.SE3 import SE3

from mrover.msg import Waypoint, GPSWaypoint, WaypointType, GPSPointList, Course as CourseMsg
from mrover.srv import EnableAuton, EnableAutonRequest, EnableAutonResponse
from navigation.drive import DriveController
from std_msgs.msg import Time, Bool
from util.SE3 import SE3
from visualization_msgs.msg import Marker

TAG_EXPIRATION_TIME_SECONDS = 60

REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")

tf_broadcaster: tf2_ros.StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()


@dataclass
class Rover:
    ctx: Context
    stuck: bool
    previous_state: str
    driver: DriveController = DriveController()

    def get_pose(self, in_odom_frame: bool = False) -> SE3:
        if in_odom_frame and self.ctx.use_odom:
            return SE3.from_tf_tree(
                self.ctx.tf_buffer, parent_frame=self.ctx.odom_frame, child_frame=self.ctx.rover_frame
            )
        else:
            return SE3.from_tf_tree(
                self.ctx.tf_buffer, parent_frame=self.ctx.world_frame, child_frame=self.ctx.rover_frame
            )

    def send_drive_command(self, twist: Twist):
        self.ctx.vel_cmd_publisher.publish(twist)

    def send_drive_stop(self):
        self.send_drive_command(Twist())

    def get_pose_with_time(self) -> Tuple[SE3, Time]:
        return SE3.from_tf_time(self.ctx.tf_buffer, parent_frame="map", child_frame="base_link")


@dataclass
class Environment:
    """
    Context class to represent the rover's environment
    Information such as locations of fiducials or obstacles
    """

    ctx: Context
    NO_FIDUCIAL: ClassVar[int] = -1
    arrived_at_post: bool = False
    last_post_location: Optional[np.ndarray] = None

    def get_fid_pos(self, fid_id: int, in_odom_frame: bool = True) -> Optional[np.ndarray]:
        """
        Retrieves the pose of the given fiducial ID from the TF tree in the odom frame
        if in_odom_frame is True otherwise in the world frame
        if it exists and is more recent than TAG_EXPIRATION_TIME_SECONDS, otherwise returns None
        """
        try:
            parent_frame = self.ctx.odom_frame if in_odom_frame else self.ctx.world_frame
            fid_pose, time = SE3.from_tf_time(
                self.ctx.tf_buffer, parent_frame=parent_frame, child_frame=f"fiducial{fid_id}"
            )
            now = rospy.Time.now()
            if now.to_sec() - time.to_sec() >= TAG_EXPIRATION_TIME_SECONDS:
                print(f"TAG EXPIRED {fid_id}!")
                return None
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            return None
        return fid_pose.position

    def current_fid_pos(self, odom_override: bool = True) -> Optional[np.ndarray]:
        """
        Retrieves the position of the current fiducial (and we are looking for it)
        :param: odom_override if false will force it to be in the map frame
        """
        assert self.ctx.course
        in_odom = self.ctx.use_odom and odom_override
        current_waypoint = self.ctx.course.current_waypoint()
        if current_waypoint is None:
            print("CURRENT WAYPOINT IS NONE")
            return None

        return self.get_fid_pos(current_waypoint.tag_id, in_odom)


@dataclass
class Course:
    ctx: Context
    course_data: CourseMsg
    # Currently active waypoint
    waypoint_index: int = 0

    def increment_waypoint(self):
        self.waypoint_index += 1

    def waypoint_pose(self, wp_idx: int) -> SE3:
        """
        Gets the pose of the waypoint with the given index
        """
        waypoint_frame = f"course{wp_idx}"
        return SE3.from_tf_tree(self.ctx.tf_buffer, parent_frame="map", child_frame=waypoint_frame)

    def current_waypoint_pose(self) -> SE3:
        """
        Gets the pose of the current waypoint
        """
        return self.waypoint_pose(self.waypoint_index)

    def current_waypoint(self) -> Optional[Waypoint]:
        """
        Returns the currently active waypoint

        :param ud:  State machine user data
        :return:    Next waypoint to reach if we have an active course
        """
        if self.course_data is None or self.waypoint_index >= len(self.course_data.waypoints):
            return None
        return self.course_data.waypoints[self.waypoint_index]

    def look_for_post(self) -> bool:
        """
        Returns whether the currently active waypoint (if it exists) indicates
        that we should go to a post
        """
        waypoint = self.current_waypoint()
        if waypoint is not None:
            return waypoint.type.val == WaypointType.POST
        else:
            return False

    def is_complete(self) -> bool:
        return self.waypoint_index == len(self.course_data.waypoints)


def setup_course(ctx: Context, waypoints: List[Tuple[Waypoint, SE3]]) -> Course:
    all_waypoint_info = []
    for wp_idx, (waypoint_info, pose) in enumerate(waypoints):
        all_waypoint_info.append(waypoint_info)
        pose.publish_to_tf_tree(tf_broadcaster, "map", f"course{wp_idx}")
    # make the course out of just the pure waypoint objects which is the 0th elt in the tuple
    return Course(ctx=ctx, course_data=CourseMsg([waypoint[0] for waypoint in waypoints]))


def convert_gps_to_cartesian(waypoint: GPSWaypoint) -> Tuple[Waypoint, SE3]:
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
    # navigation algorithms currently require all coordinates to have zero as the z coordinate
    odom[2] = 0

    return Waypoint(tag_id=waypoint.tag_id, type=waypoint.type), SE3(position=odom)


def convert_cartesian_to_gps(coordinate: np.ndarray) -> GPSWaypoint:
    """
    Converts a coordinate to a GPSWaypoint (used for sending data back to basestation)
    """
    lat, lon, _ = pymap3d.enu2geodetic(
        e=coordinate[0], n=coordinate[1], u=0.0, lat0=REF_LAT, lon0=REF_LON, h0=0.0, deg=True
    )
    return GPSWaypoint(lat, lon, WaypointType(val=WaypointType.NO_SEARCH), 0)


def convert_and_get_course(ctx: Context, data: EnableAutonRequest) -> Course:
    waypoints = [convert_gps_to_cartesian(waypoint) for waypoint in data.waypoints]
    return setup_course(ctx, waypoints)


class Context:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rospy.Publisher
    search_point_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    course_listener: rospy.Subscriber
    stuck_listener: rospy.Subscriber
    costmap_listener: rospy.Subscriber

    # Use these as the primary interfaces in states
    course: Optional[Course]
    rover: Rover
    env: Environment
    disable_requested: bool

    # ROS Params from localization.yaml
    use_odom: bool
    world_frame: str
    odom_frame: str
    rover_frame: str

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher("nav_vis", Marker, queue_size=1)
        self.search_point_publisher = rospy.Publisher("search_path", GPSPointList, queue_size=1)
        self.enable_auton_service = rospy.Service("enable_auton", EnableAuton, self.recv_enable_auton)
        self.stuck_listener = rospy.Subscriber("nav_stuck", Bool, self.stuck_callback)
        self.costmap_listener = rospy.Subscriber("costmap", OccupancyGrid, self.costmap_callback)
        self.course = None
        self.rover = Rover(self, False, "")
        self.env = Environment(self)
        self.disable_requested = False
        self.use_odom = rospy.get_param("use_odom_frame")
        self.world_frame = rospy.get_param("world_frame")
        self.odom_frame = rospy.get_param("odom_frame")
        self.rover_frame = rospy.get_param("rover_frame")

    def recv_enable_auton(self, req: EnableAutonRequest) -> EnableAutonResponse:
        if req.enable:
            self.course = convert_and_get_course(self, req)
        else:
            self.disable_requested = True
        return EnableAutonResponse(True)

    def stuck_callback(self, msg: Bool):
        self.rover.stuck = msg.data

    def costmap_callback(self, msg: OccupancyGrid):
        # update data structure
        height = msg.info.height
        width = msg.info.width
        rospy.logerr(f"height: {height}, width: {width}")
        costmap1D = np.array(msg.data)
        costmap2D = costmap1D.reshape((height, width))
        rospy.logerr(f"2D costmap: {costmap2D.shape}")
        rospy.logerr(f"2D costmap: {costmap2D}")
