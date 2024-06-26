from typing import Optional

import numpy as np

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from mrover.msg import GPSPointList
from nav_msgs.msg import Path
from navigation import approach_target, recovery, waypoint
from navigation.astar import AStar, SpiralEnd, NoPath
from navigation.context import convert_cartesian_to_gps, Context
from navigation.trajectory import Trajectory, SearchTrajectory
from std_msgs.msg import Header
from util.state_lib.state import State


# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class WaterBottleSearchState(State):
    """
    State when searching for the water bottle
    Follows a search spiral but uses A* to avoid obstacles
    """

    trajectory: Optional[SearchTrajectory] = None  # spiral
    star_traj: Trajectory  # returned by astar
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: rospy.Time
    path_pub: rospy.Publisher
    astar: AStar

    STOP_THRESH = rospy.get_param("water_bottle_search/stop_threshold")
    DRIVE_FWD_THRESH = rospy.get_param("water_bottle_search/drive_forward_threshold")
    SPIRAL_COVERAGE_RADIUS = rospy.get_param("water_bottle_search/coverage_radius")
    SEGMENTS_PER_ROTATION = rospy.get_param(
        "water_bottle_search/segments_per_rotation",
    )  # TODO: after testing, might need to change
    DISTANCE_BETWEEN_SPIRALS = rospy.get_param(
        "water_bottle_search/distance_between_spirals"
    )  # TODO: after testing, might need to change
    TRAVERSABLE_COST = rospy.get_param("water_bottle_search/traversable_cost")
    UPDATE_DELAY = rospy.get_param("water_bottle_search/update_delay")
    SAFE_APPROACH_DISTANCE = rospy.get_param("water_bottle_search/safe_approach_distance")

    def find_endpoint(self, context: Context, end: np.ndarray) -> np.ndarray:
        """
        A-STAR Algorithm: f(n) = g(n) + h(n) to find a path from the given start to the given end in the given costmap
        :param context: Context
        :param end:     Next point in the spiral from traj in cartesian coordinates
        :return:        The end point in cartesian coordinates
        """
        costmap_2d = context.env.cost_map.data
        # convert end to occupancy grid coordinates then node
        end_ij = self.astar.cartesian_to_ij(end)
        end_node = self.astar.Node(None, (end_ij[0], end_ij[1]))

        # check if end node is within range, if it is, check if it has a high cost
        if (costmap_2d.shape[0] - 1) >= end_node.position[0] >= 0 and (costmap_2d.shape[1] - 1) >= end_node.position[
            1
        ] >= 0:
            while (
                costmap_2d[end_node.position[0], end_node.position[1]] >= self.TRAVERSABLE_COST
            ):  # TODO: find optimal value
                # True if the trajectory is finished
                if WaterBottleSearchState.trajectory.increment_point():
                    raise SpiralEnd()
                # update end point to be the next point in the search spiral
                end_ij = self.astar.cartesian_to_ij(WaterBottleSearchState.trajectory.get_current_point())
                end_node = self.astar.Node(None, (end_ij[0], end_ij[1]))
                rospy.loginfo(f"End has high cost! new end: {end_ij}")
        return WaterBottleSearchState.trajectory.get_current_point()

    def avg_cell_cost(self, context: Context, start: np.ndarray, end: np.ndarray) -> float:
        start_ij = self.astar.cartesian_to_ij(start)
        end_ij = self.astar.cartesian_to_ij(end)
        if np.array_equal(start_ij, end_ij):
            return 0.0

        vector = end_ij - start_ij
        magnitude = np.linalg.norm(vector)
        unit_vector = vector / magnitude
        end_ij += np.round(unit_vector * 2).astype(np.int8)

        points = []
        x0 = start_ij[0]
        x1 = end_ij[0]
        y0 = start_ij[1]
        y1 = end_ij[1]
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        i = 0
        max_iter = 1000
        while i < max_iter and ((x0 != x1) or (y0 != y1)):
            points.append((x0, y0))
            err2 = 2 * err
            if err2 > -dy:
                err -= dy
                x0 += sx
            if err2 < dx:
                err += dx
                y0 += sy
            i += 1
        points.append((x1, y1))
        total_cost = 0
        count = 0
        for i, j in points:
            total_cost += context.env.cost_map.data[i][j]
            count += 1
        return total_cost / count

    def on_enter(self, context: Context) -> None:
        if WaterBottleSearchState.trajectory is None:
            self.new_trajectory(context)

        if not self.is_recovering:
            self.prev_target_pos_in_map = None

        origin_in_map = context.course.current_waypoint_pose_in_map().position[0:2]
        self.astar = AStar(origin_in_map, context)
        rospy.loginfo(f"Origin: {origin_in_map}")
        self.star_traj = Trajectory(np.array([]))
        self.time_last_updated = rospy.Time.now()
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)

    def on_exit(self, context: Context) -> None:
        context.costmap_listener.unregister()

    def on_loop(self, context: Context) -> State:
        assert context.course is not None

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        bottle_in_map = context.env.current_target_pos()

        # Only update our costmap every so often
        if rospy.Time.now() - self.time_last_updated > rospy.Duration(self.UPDATE_DELAY):
            rover_position_in_map = rover_in_map.position[0:2]

            if bottle_in_map is None:
                end_point_in_map = self.find_endpoint(
                    context, WaterBottleSearchState.trajectory.get_current_point()[0:2]
                )
            else:
                end_point_in_map = bottle_in_map

            # If path to next spiral point has minimal cost per cell, continue normally to next spiral point
            if self.avg_cell_cost(context, rover_position_in_map, end_point_in_map[0:2]) < self.TRAVERSABLE_COST / 2:
                self.star_traj = Trajectory(np.array([]))
                rospy.loginfo_throttle(1, "NOT running A*, path ahead is clear!")
            # Otherwise, create a path planned through the cost
            else:
                rospy.loginfo("Running A*...")
                context.rover.send_drive_command(Twist())  # stop while planning
                try:
                    occupancy_list = self.astar.a_star(rover_position_in_map, end_point_in_map[0:2])
                except SpiralEnd:
                    # TODO: what to do in this case
                    WaterBottleSearchState.trajectory.reset()
                    occupancy_list = None
                except NoPath:
                    # increment end point
                    if WaterBottleSearchState.trajectory.increment_point():
                        # TODO: what to do in this case
                        WaterBottleSearchState.trajectory.reset()
                    occupancy_list = None
                if occupancy_list is None:
                    self.star_traj = Trajectory(np.array([]))
                else:
                    cartesian_coords = self.astar.ij_to_cartesian(np.array(occupancy_list))
                    self.star_traj = Trajectory(
                        np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1))))
                    )  # current point gets set back to 0

                    # create path type to publish planned path segments to see in rviz
                    path = Path()
                    poses = []
                    path.header = Header()
                    path.header.frame_id = "map"
                    for coord in cartesian_coords:
                        pose_stamped = PoseStamped()
                        pose_stamped.header = Header()
                        pose_stamped.header.frame_id = "map"
                        point = Point(coord[0], coord[1], 0)
                        quat = Quaternion(0, 0, 0, 1)
                        pose_stamped.pose = Pose(point, quat)
                        poses.append(pose_stamped)
                    path.poses = poses
                    self.path_pub.publish(path)

                self.time_last_updated = rospy.Time.now()

        # Continue executing the path from wherever it left off
        target_position_in_map = WaterBottleSearchState.trajectory.get_current_point()
        traj_target = True
        # If there is an alternate path we need to take to avoid the obstacle, use that trajectory
        if len(self.star_traj.coordinates) != 0:
            target_position_in_map = self.star_traj.get_current_point()
            traj_target = False
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_position_in_map,
            rover_in_map,
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target_pos_in_map,
        )
        if arrived:
            self.prev_target_pos_in_map = target_position_in_map
            # If our target was the search spiral point, only increment the spiral path
            if traj_target:
                rospy.loginfo("Arrived at spiral point")
                # If we finish the spiral without seeing the object, move on with course
                if WaterBottleSearchState.trajectory.increment_point():
                    return waypoint.WaypointState()
            else:  # Otherwise, increment the astar path
                # If we finish the astar path, then reset astar and increment the spiral path
                if self.star_traj.increment_point():
                    rospy.loginfo("Arrived at end of astar")
                    self.star_traj = Trajectory(np.array([]))
                    if WaterBottleSearchState.trajectory.increment_point():
                        return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False
        context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in WaterBottleSearchState.trajectory.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)

        if (
            bottle_in_map is not None
            and np.linalg.norm(rover_in_map.position[0:2] - bottle_in_map[0:2]) < self.SAFE_APPROACH_DISTANCE
        ):
            return approach_target.ApproachTargetState()

        return self

    def new_trajectory(self, context) -> None:
        assert context.course is not None
        search_center = context.course.current_waypoint()
        assert search_center is not None

        if not self.is_recovering:
            WaterBottleSearchState.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.tag_id,
                True,
            )
