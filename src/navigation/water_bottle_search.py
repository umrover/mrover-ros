from typing import Optional

import numpy as np

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from mrover.msg import GPSPointList
from nav_msgs.msg import Path
from navigation import approach_object, recovery, waypoint
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

    traj: SearchTrajectory  # spiral
    star_traj: Trajectory  # returned by astar
    prev_target: Optional[np.ndarray] = None
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

    def find_endpoint(self, context: Context, end: np.ndarray) -> np.ndarray:
        """
        A-STAR Algorithm: f(n) = g(n) + h(n) to find a path from the given start to the given end in the given costmap
        :param end: next point in the spiral from traj in cartesian coordinates
        :return: the end point in cartesian coordinates
        """
        costmap_2d = context.env.cost_map.data
        # convert end to occupancy grid coordinates then node
        end_ij = self.astar.cartesian_to_ij(end)
        end_node = self.astar.Node(None, (end_ij[0], end_ij[1]))

        # check if end node is within range, if it is, check if it has a high cost
        if (costmap_2d.shape[0] - 1) >= end_node.position[0] >= 0 and (costmap_2d.shape[1] - 1) >= end_node.position[
            1
        ] >= 0:
            while costmap_2d[end_node.position[0], end_node.position[1]] >= 0.2:  # TODO: find optimal value
                # True if the trajectory is finished
                if self.traj.increment_point():
                    raise SpiralEnd()
                # update end point to be the next point in the search spiral
                end_ij = self.astar.cartesian_to_ij(self.traj.get_cur_pt())
                end_node = self.astar.Node(None, (end_ij[0], end_ij[1]))
                print(f"End has high cost! new end: {end_ij}")
        return self.traj.get_cur_pt()

    def on_enter(self, context: Context) -> None:
        assert context.course is not None

        search_center = context.course.current_waypoint()
        assert search_center is not None

        if not self.is_recovering:
            self.traj = SearchTrajectory.spiral_traj(
                context.rover.get_pose().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.tag_id,
                True,
            )
            origin = context.rover.get_pose().position[0:2]
            self.astar = AStar(origin, context)
            rospy.loginfo(f"Origin: {origin}")
            self.prev_target = None
        self.star_traj = Trajectory(np.array([]))
        self.time_last_updated = rospy.Time.now()
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)

    def on_exit(self, context: Context) -> None:
        context.costmap_listener.unregister()

    def on_loop(self, context: Context) -> State:
        assert context.course is not None

        # Only update our costmap every 1 second
        if rospy.Time.now() - self.time_last_updated > rospy.Duration(1):
            cur_rover_pose = context.rover.get_pose().position[0:2]
            end_point = self.find_endpoint(context, self.traj.get_cur_pt()[0:2])

            rospy.loginfo("Running A*...")
            try:
                occupancy_list = self.astar.a_star(cur_rover_pose, end_point[0:2])
            except SpiralEnd:
                # TODO: what to do in this case
                self.traj.reset()
                occupancy_list = None
            except NoPath:
                # increment end point
                if self.traj.increment_point():
                    # TODO: what to do in this case
                    self.traj.reset()
                occupancy_list = None
            if occupancy_list is None:
                self.star_traj = Trajectory(np.array([]))
            else:
                cartesian_coords = self.astar.ij_to_cartesian(np.array(occupancy_list))
                rospy.loginfo(f"{cartesian_coords}, shape: {cartesian_coords.shape}")
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

        # continue executing the path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        traj_target = True
        # if there is an alternate path we need to take to avoid the obstacle, use that trajectory
        if len(self.star_traj.coordinates) != 0:
            target_pos = self.star_traj.get_cur_pt()
            traj_target = False
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target,
        )
        if arrived:
            self.prev_target = target_pos
            # if our target was the search spiral point, only increment the spiral path
            if traj_target:
                rospy.loginfo("Arrived at spiral point")
                # if we finish the spiral without seeing the object, move on with course
                if self.traj.increment_point():
                    return waypoint.WaypointState()
            else:  # otherwise, increment the astar path
                # if we finish the astar path, then reset astar and increment the spiral path
                if self.star_traj.increment_point():
                    rospy.loginfo("Arrived at end of astar")
                    self.star_traj = Trajectory(np.array([]))
                    if self.traj.increment_point():
                        return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False
        context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in self.traj.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)

        if context.env.current_target_pos() is not None and context.course.look_for_object():
            return approach_object.ApproachObjectState()

        return self
