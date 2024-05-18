from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np
import pymap3d
from scipy.signal import convolve2d

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import (
    Waypoint,
    GPSWaypoint,
    WaypointType,
    GPSPointList,
    Course as CourseMsg,
    ImageTarget,
    ImageTargets,
)
from mrover.srv import EnableAuton, EnableAutonRequest, EnableAutonResponse
from nav_msgs.msg import OccupancyGrid
from navigation.drive import DriveController
from std_msgs.msg import Bool
from util.SE3 import SE3
from util.state_lib.state import State

TARGET_EXPIRATION_DURATION = rospy.Duration(60)

LONG_RANGE_TAG_EXPIRATION_DURATION = rospy.Duration(rospy.get_param("long_range/time_threshold"))
INCREMENT_WEIGHT = rospy.get_param("long_range/increment_weight")
DECREMENT_WEIGHT = rospy.get_param("long_range/decrement_weight")
MIN_HITS = rospy.get_param("long_range/min_hits")
MAX_HITS = rospy.get_param("long_range/max_hits")

REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")

tf_broadcaster: tf2_ros.StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()


@dataclass
class Rover:
    ctx: Context
    stuck: bool
    previous_state: State
    driver: DriveController = DriveController()

    def get_pose(self) -> SE3:
        try:
            return SE3.from_tf_tree(
                self.ctx.tf_buffer, parent_frame=self.ctx.world_frame, child_frame=self.ctx.rover_frame
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise Exception("Navigation failed to get rover pose. Is localization running?")

    def send_drive_command(self, twist: Twist) -> None:
        self.ctx.vel_cmd_publisher.publish(twist)

    def send_drive_stop(self) -> None:
        self.send_drive_command(Twist())


@dataclass
class Environment:
    """
    Context class to represent the rover's environment
    Information such as locations of tags or obstacles
    """

    ctx: Context
    long_range_tags: LongRangeTagStore
    cost_map: CostMap

    NO_TAG: int = -1

    arrived_at_target: bool = False
    arrived_at_waypoint: bool = False
    last_target_location: Optional[np.ndarray] = None

    def get_target_position(self, frame: str) -> Optional[np.ndarray]:
        """
        Retrieves the pose of the target in the world frame if:
        1) It exists
        2) It is not too old (older than TARGET_EXPIRATION_TIME_SECONDS)
        :param frame:   Target frame name. Could be for a tag, the hammer, or the water bottle.
        :return:        Pose of the target or None.
        """
        try:
            target_pose, time = SE3.from_tf_tree_with_time(
                self.ctx.tf_buffer, parent_frame=self.ctx.world_frame, child_frame=frame
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        if rospy.Time.now() - time > TARGET_EXPIRATION_DURATION:
            return None
        return target_pose.position

    def current_target_pos(self) -> Optional[np.ndarray]:
        assert self.ctx.course

        current_waypoint = self.ctx.course.current_waypoint()
        if current_waypoint is None:
            rospy.logwarn("Current waypoint is empty!")
            return None

        match current_waypoint.type.val:
            case WaypointType.POST:
                return self.get_target_position(f"tag{current_waypoint.tag_id}")
            case WaypointType.MALLET:
                return self.get_target_position("hammer")
            case WaypointType.WATER_BOTTLE:
                return self.get_target_position("bottle")
            case _:
                return None


class LongRangeTagStore:
    """
    Context class to represent the tags seen in the long range camera
    """

    @dataclass
    class TagData:
        hit_count: int
        tag_id: int
        bearing: float
        time: rospy.Time

    _data: dict[int, TagData]
    _context: Context
    _min_hits: int
    _max_hits: int

    def __init__(self, context: Context, min_hits: int = MIN_HITS, max_hits: int = MAX_HITS) -> None:
        self._data = {}
        self._context = context
        self._min_hits = min_hits
        self._max_hits = max_hits

    def push_frame(self, tags: List[ImageTarget]) -> None:
        """
        Loops through our current list of our stored tags and checks if the new message includes each tag or doesn't.
        If it does include it, we will increment our hit count for that tag id, store the new tag information, and reset the time we saw it.
        If it does not include it, we will decrement our hit count for that tag id, and if the hit count becomes zero, then we remove it from our stored list.
        If there are tag ids in the new message that we don't have stored, we will add it to our stored list.
        :param tags: A list of LongRangeTags sent by perception, which includes an id and bearing for each tag in the list
        """
        # Update our current tags
        tag_ids = [int(tag.name[3:]) for tag in tags]
        for _, stored_tag in list(self._data.items()):
            # If we do see one of our tags in the new message, increment its hit count
            if stored_tag.tag_id in tag_ids:
                stored_tag.hit_count += INCREMENT_WEIGHT
                if stored_tag.hit_count > self._max_hits:
                    stored_tag.hit_count = self._max_hits
            # If we don't see one of our tags in the new message, decrement its hit count
            else:
                stored_tag.hit_count -= DECREMENT_WEIGHT
                if stored_tag.hit_count <= 0:
                    stored_tag.hit_count = 0
                    # If we haven't seen the tag in a while, remove it from our list
                    time_difference = rospy.Time.now() - stored_tag.time
                    if time_difference > LONG_RANGE_TAG_EXPIRATION_DURATION:
                        del self._data[stored_tag.tag_id]

        # Add newly seen tags to our data structure or update current tag's information
        for tag_id, tag in zip(tag_ids, tags):
            hit_count = self._data[tag_id].hit_count if tag_id in self._data else INCREMENT_WEIGHT
            self._data[tag_id] = self.TagData(
                hit_count=hit_count, tag_id=tag_id, bearing=tag.bearing, time=rospy.Time.now()
            )

    def query(self, tag_id: int) -> Optional[TagData]:
        """
        Returns the corresponding tag if the tag has been seen by the long range camera enough times recently
        :param tag_id:  ID corresponding to the tag we want to return
        :return:        LongRangeTag if we have seen the tag enough times recently in the long range camera, otherwise return None
        """
        if not self._data:
            return None
        if tag_id not in self._data:
            return None
        if rospy.Time.now() - self._data[tag_id].time >= LONG_RANGE_TAG_EXPIRATION_DURATION:
            return None
        return self._data[tag_id]


class CostMap:
    """
    Context class to represent the costmap generated around the water bottle waypoint
    """

    data: np.ndarray
    resolution: int
    height: int
    width: int


@dataclass
class Course:
    ctx: Context
    course_data: CourseMsg
    # Currently active waypoint
    waypoint_index: int = 0

    def increment_waypoint(self) -> None:
        self.waypoint_index += 1

    def waypoint_pose(self, index: int) -> SE3:
        waypoint_frame = f"course{index}"
        return SE3.from_tf_tree(self.ctx.tf_buffer, parent_frame="map", child_frame=waypoint_frame)

    def current_waypoint_pose(self) -> SE3:
        return self.waypoint_pose(self.waypoint_index)

    def current_waypoint(self) -> Optional[Waypoint]:
        """
        :return: The currently active waypoint if we have an active course
        """
        if self.course_data is None:
            return None
        if self.waypoint_index >= len(self.course_data.waypoints):
            return None
        return self.course_data.waypoints[self.waypoint_index]

    def look_for_post(self) -> bool:
        """
        :return: Whether the currently active waypoint is a post (if it exists)
        """
        current_waypoint = self.current_waypoint()
        return current_waypoint is not None and current_waypoint.type.val == WaypointType.POST

    def look_for_object(self) -> bool:
        """
        :return: Whether the currently active waypoint is an object (if it exists).
                 Either the mallet or the water bottle.
        """
        current_waypoint = self.current_waypoint()
        return current_waypoint is not None and current_waypoint.type.val in {
            WaypointType.MALLET,
            WaypointType.WATER_BOTTLE,
        }

    def is_complete(self) -> bool:
        return self.waypoint_index == len(self.course_data.waypoints)

    def get_approach_target_state(self) -> Optional[State]:
        """
        :return: One of the approach states (ApproachPostState, LongRangeState, or ApproachObjectState)
                 if we are looking for a post or object, and we see it in one of the cameras (ZED or long range)
        """
        from navigation import approach_post, long_range, approach_object

        current_waypoint = self.current_waypoint()
        if self.look_for_post():
            # If we see the tag in the ZED, go to ApproachPostState
            if self.ctx.env.current_target_pos() is not None:
                return approach_post.ApproachPostState()
            # If we see the tag in the long range camera, go to LongRangeState
            assert current_waypoint is not None
            if self.ctx.env.long_range_tags.query(current_waypoint.tag_id) is not None:
                return long_range.LongRangeState()
        elif self.look_for_object():
            if self.ctx.env.current_target_pos() is not None:
                return approach_object.ApproachObjectState()  # if we see the object
        return None


def setup_course(ctx: Context, waypoints: List[Tuple[Waypoint, SE3]]) -> Course:
    all_waypoint_info = []
    for index, (waypoint_info, pose) in enumerate(waypoints):
        all_waypoint_info.append(waypoint_info)
        pose.publish_to_tf_tree(tf_broadcaster, "map", f"course{index}")
    # Make the course out of just the pure waypoint objects which is the 0th element in the tuple
    return Course(ctx=ctx, course_data=CourseMsg([waypoint for waypoint, _ in waypoints]))


def convert_gps_to_cartesian(waypoint: GPSWaypoint) -> Tuple[Waypoint, SE3]:
    """
    Converts a GPSWaypoint into a "Waypoint" used for publishing to the CourseService.
    """
    # Create a cartesian position based on GPS latitude and longitude
    position = np.array(
        pymap3d.geodetic2enu(
            waypoint.latitude_degrees, waypoint.longitude_degrees, 0.0, REF_LAT, REF_LON, 0.0, deg=True
        )
    )
    # Zero the z-coordinate because even though the altitudes are set to zero,
    # Two points on a sphere are not going to have the same z-coordinate.
    # Navigation algorithms currently require all coordinates to have zero as the z-coordinate.
    position[2] = 0
    return Waypoint(tag_id=waypoint.tag_id, type=waypoint.type), SE3(position=position)


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
    course_listener: rospy.Subscriber
    stuck_listener: rospy.Subscriber
    tag_data_listener: rospy.Subscriber
    costmap_listener: rospy.Subscriber

    # Use these as the primary interfaces in states
    course: Optional[Course]
    rover: Rover
    env: Environment
    disable_requested: bool

    # ROS Params from localization.yaml
    world_frame: str
    rover_frame: str

    def __init__(self) -> None:
        from navigation.state import OffState

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher("navigation_cmd_vel", Twist, queue_size=1)
        self.search_point_publisher = rospy.Publisher("search_path", GPSPointList, queue_size=1)
        self.enable_auton_service = rospy.Service("enable_auton", EnableAuton, self.recv_enable_auton)
        self.stuck_listener = rospy.Subscriber("nav_stuck", Bool, self.stuck_callback)
        self.course = None
        self.rover = Rover(self, False, OffState())
        self.env = Environment(self, long_range_tags=LongRangeTagStore(self), cost_map=CostMap())
        self.disable_requested = False
        self.world_frame = rospy.get_param("world_frame")
        self.rover_frame = rospy.get_param("rover_frame")
        self.tag_data_listener = rospy.Subscriber("tags", ImageTargets, self.tag_data_callback)
        self.costmap_listener = rospy.Subscriber("costmap", OccupancyGrid, self.costmap_callback)

    def recv_enable_auton(self, req: EnableAutonRequest) -> EnableAutonResponse:
        if req.enable:
            self.course = convert_and_get_course(self, req)
        else:
            self.disable_requested = True
        return EnableAutonResponse(True)

    def stuck_callback(self, msg: Bool) -> None:
        self.rover.stuck = msg.data

    def tag_data_callback(self, tags: ImageTargets) -> None:
        self.env.long_range_tags.push_frame(tags.targets)

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback function for the occupancy grid perception sends
        :param msg: Occupancy Grid representative of a 30 x 30m square area with origin at GNSS waypoint. Values are 0, 1, -1
        """
        self.env.cost_map.resolution = msg.info.resolution  # meters/cell
        self.env.cost_map.height = msg.info.height  # cells
        self.env.cost_map.width = msg.info.width  # cells
        self.env.cost_map.data = (
            np.array(msg.data).reshape((int(self.env.cost_map.height), int(self.env.cost_map.width))).astype(np.float32)
        )

        # change all unidentified points to have a slight cost
        self.env.cost_map.data[self.env.cost_map.data == -1.0] = 0.1  # TODO: find optimal value
        self.env.cost_map.data[self.env.cost_map.data >= 1] = 1
        self.env.cost_map.data = np.rot90(self.env.cost_map.data, k=3, axes=(0, 1))  # rotate 90 degrees clockwise

        # apply kernel to average the map with zero padding
        kernel_shape = (7, 7)  # TODO: find optimal kernel size
        kernel = np.ones(kernel_shape, dtype=np.float32) / (kernel_shape[0] * kernel_shape[1])
        self.env.cost_map.data = convolve2d(self.env.cost_map.data, kernel, mode="same")
