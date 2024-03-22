from __future__ import annotations

import heapq
import random
from threading import Lock
from typing import Optional
from scipy.signal import convolve2d


import numpy as np

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from mrover.msg import GPSPointList
from nav_msgs.msg import OccupancyGrid, Path
from navigation import approach_object, recovery, waypoint
from navigation.context import convert_cartesian_to_gps, Context
from navigation.trajectory import Trajectory, SearchTrajectory
from util.SE3 import SE3
from util.ros_utils import get_rosparam
from util.state_lib.state import State


class Node:
    """
    Node class for astar pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f

    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f
    
class SpiralEnd(Exception):
    """
    Raise when there are no more points left in the search spiral
    """
    pass

class NoPath(Exception):
    """
    Raise when an A* path could not be found
    """
    pass


# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class WaterBottleSearchState(State):
    traj: SearchTrajectory  # spiral
    star_traj: Trajectory  # returned by astar
    # when we are moving along the spiral
    prev_target: Optional[np.ndarray] = None
    is_recovering: bool = False
    height: int = 0  # height of occupancy grid
    width: int = 0  # width of occupancy grid
    resolution: int = 0  # resolution of occupancy grid
    origin: np.ndarray = np.array([])  # hold the inital rover pose (waypoint of water bottle)
    context: Context
    listener: rospy.Subscriber
    time_last_updated: rospy.Time
    costmap_lock: Lock
    path_pub: rospy.Publisher

    STOP_THRESH = get_rosparam("water_bottle_search/stop_thresh", 0.5)
    DRIVE_FWD_THRESH = get_rosparam("water_bottle_search/drive_fwd_thresh", 0.34)
    SPIRAL_COVERAGE_RADIUS = get_rosparam("water_bottle_search/coverage_radius", 10)
    SEGMENTS_PER_ROTATION = get_rosparam(
        "water_bottle_search/segments_per_rotation", 8
    )  # TODO: after testing, might need to change
    DISTANCE_BETWEEN_SPIRALS = get_rosparam(
        "water_bottle_search/distance_between_spirals", 3
    )  # TODO: after testing, might need to change

    def cartesian_to_ij(self, cart_coord: np.ndarray) -> np.ndarray:
        """
        Convert real world cartesian coordinates (x, y) to coordinates in the occupancy grid (i, j)
        using formula floor(v - (WP + [-W/2, H/2]) / r) * [1, -1]
        v: (x,y) coordinate
        WP: origin
        W, H: grid width, height (meters)
        r: resolution (meters/cell)
        :param cart_coord: array of x and y cartesian coordinates
        :return: array of i and j coordinates for the occupancy grid
        """
        width = self.width * self.resolution
        height = self.height * self.resolution
        width_height = np.array([-1 * width / 2, height / 2])  # [-W/2, H/2]
        converted_coord = np.floor((cart_coord[0:2] - (self.origin + width_height)) / self.resolution)
        return converted_coord.astype(np.int8) * np.array([1, -1])

    def ij_to_cartesian(self, ij_coords: np.ndarray) -> np.ndarray:
        """
        Convert coordinates in the occupancy grid (i, j) to real world cartesian coordinates (x, y)
        using formula (WP - [W/2, H/2]) + [j * r, i * r] + [r/2, -r/2] * [1, -1]
        WP: origin
        W, H: grid width, height (meters)
        r: resolution (meters/cell)
        :param ij_coords: array of i and j occupancy grid coordinates
        :return: array of x and y coordinates in the real world
        """
        width = self.width * self.resolution
        height = self.height * self.resolution
        width_height = np.array([width / 2, height / 2])  # [W/2, H/2]
        resolution_conversion = ij_coords * [self.resolution, self.resolution]  # [j * r, i * r]
        half_res = np.array([self.resolution / 2, -1 * self.resolution / 2])  # [r/2, -r/2]
        return ((self.origin - width_height) + resolution_conversion + half_res) * np.array([1, -1])

    def return_path(self, current_node, costmap2D):
        """
        Return the path given from A-Star in reverse through current node's parents
        :param current_node: end point of path which contains parents to retrieve path
        :param costmap2D: current costmap
        :return: reversed path except the starting point (we are already there)
        """
        costmap2D = np.copy(costmap2D)
        path = []
        current = current_node
        while current is not None:
            path.append(current.position)
            current = current.parent
        reversed_path = path[::-1]
        print("ij:", reversed_path[1:])

        # Print visual of costmap with path and start (S) and end (E) points
        # The lighter the block, the more costly it is
        for step in reversed_path:
            costmap2D[step[0]][step[1]] = 2 # path (.)
        costmap2D[reversed_path[0][0]][reversed_path[0][1]] = 3  # start
        costmap2D[reversed_path[-1][0]][reversed_path[-1][1]] = 4  # end

        for row in costmap2D:
            line = []
            for col in row:
                if col == 1.0:
                    line.append("\u2588")
                elif col < 1.0 and col >= 0.8:
                    line.append("\u2593")
                elif col < 0.8 and col >= 0.5:
                    line.append("\u2592")
                elif col < 0.5 and col >= 0.2:
                    line.append("\u2591")
                elif col < 0.2:
                    line.append(" ")
                elif col == 2:
                    line.append(".")
                elif col == 3:
                    line.append("S")
                elif col == 4:
                    line.append("E")
            print("".join(line))

        return reversed_path[1:]

    def a_star(self, costmap2d: np.ndarray, start: np.ndarray, end: np.ndarray) -> list | None:
        """
        A-STAR Algorithm: f(n) = g(n) + h(n) to find a path from the given start to the given end in the given costmap
        :param costmap2d: occupancy grid in the form of an 2D array of floats
        :param start: rover pose in cartesian coordinates
        :param end: next point in the spiral from traj in cartesian coordinates
        :return: list of A-STAR coordinates in the occupancy grid coordinates (i,j)
        """
        # convert start and end to occupancy grid coordinates
        startij = self.cartesian_to_ij(start)
        endij = self.cartesian_to_ij(end)

        # initialize start and end nodes
        start_node = Node(None, (startij[0], startij[1]))
        end_node = Node(None, (endij[0], endij[1]))

        if start_node == end_node:
            return None

        print(f"start: {start}, end: {end}")
        print(f"startij: {startij}, endij: {endij}")

        # check if end node is within range, if it is, check if it has a high cost
        if (
            end_node.position[0] <= (costmap2d.shape[0] - 1)
            and end_node.position[0] >= 0
            and end_node.position[1] <= (costmap2d.shape[1] - 1)
            and end_node.position[1] >= 0
        ):
            while(costmap2d[end_node.position[0], end_node.position[1]] >= 0.2): # TODO: find optimal value
                # True if the trajectory is finished
                if self.traj.increment_point():
                    raise SpiralEnd()
                # update end point to be the next point in the search spiral
                endij = self.cartesian_to_ij(self.traj.get_cur_pt())
                end_node = Node(None, (endij[0], endij[1]))
                print(f"End has high cost! new end: {endij}")

        # initialize both open and closed list
        open_list = []
        closed_list = []

        # heapify the open_list and add the start node
        heapq.heapify(open_list)
        heapq.heappush(open_list, start_node)

        # add a stop condition
        outer_iterations = 0
        max_iterations = costmap2d.shape[0] * costmap2d.shape[1] // 2

        # movements/squares we can search
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1))
        adjacent_square_pick_index = [0, 1, 2, 3, 4, 5, 6, 7]

        # loop until you find the end
        while len(open_list) > 0:
            # randomize the order of the adjacent_squares_pick_index to avoid a decision making bias
            random.shuffle(adjacent_square_pick_index)

            outer_iterations += 1

            if outer_iterations > max_iterations:
                # if we hit this point return the path such as it is. It will not contain the destination
                print("giving up on pathfinding too many iterations")
                return self.return_path(current_node, costmap2d)

            # get the current node
            current_node = heapq.heappop(open_list)
            closed_list.append(current_node)

            # found the goal
            if current_node == end_node:
                return self.return_path(current_node, costmap2d)

            # generate children
            children = []
            for pick_index in adjacent_square_pick_index:
                new_position = adjacent_squares[pick_index]
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                # make sure within range
                if (
                    node_position[0] > (costmap2d.shape[0] - 1)
                    or node_position[0] < 0
                    or node_position[1] > (costmap2d.shape[1] - 1)
                    or node_position[1] < 0
                ):
                    continue

                # make sure it is traversable terrian (not too high of a cost)
                if costmap2d[node_position[0]][node_position[1]] >= 0.2: # TODO: find optimal value
                    continue

                # create new node and append it
                new_node = Node(current_node, node_position)
                children.append(new_node)

            # loop through children
            for child in children:
                # child is on the closed list
                if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                    continue
                # create the f (total), g (cost in map), and h (euclidean distance) values
                child.g = current_node.g + costmap2d[child.position[0], child.position[1]]
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                    (child.position[1] - end_node.position[1]) ** 2
                )
                child.f = child.g + child.h
                # child is already in the open list
                if (
                    len(
                        [
                            open_node
                            for open_node in open_list
                            if child.position == open_node.position and child.g > open_node.g
                        ]
                    )
                    > 0
                ):
                    continue
                # add the child to the open list
                heapq.heappush(open_list, child)

        print("Couldn't find a path to destination")
        raise NoPath()

    def costmap_callback(self, msg: OccupancyGrid):
        """
        Callback function for the occupancy grid perception sends
        :param msg: Occupancy Grid representative of a 30 x 30m square area with origin at GNSS waypoint. Values are 0, 1, -1
        """
        # only update our costmap every 0.5 seconds
        if rospy.get_time() - self.time_last_updated > 1:
            print("RUN ASTAR")

            self.resolution = msg.info.resolution # meters/cell
            self.height = msg.info.height  # cells
            self.width = msg.info.width  # cells
            with self.costmap_lock:
                costmap2D = np.array(msg.data).reshape((int(self.height), int(self.width))).astype(np.float32)
                
                # change all unidentified points to have a slight cost
                costmap2D[costmap2D == -1.0] = 0.1 # TODO: find optimal value
                costmap2D = np.rot90(costmap2D, k=3, axes=(0,1)) # rotate 90 degress clockwise

                # apply kernel to average the map with zero padding
                kernel_shape = (7,7) # TODO: find optimal kernel size
                kernel = np.ones(kernel_shape, dtype = np.float32) / (kernel_shape[0]*kernel_shape[1])
                costmap2D = convolve2d(costmap2D, kernel, mode = "same")

                cur_rover_pose = self.context.rover.get_pose().position[0:2]
                end_point = self.traj.get_cur_pt()

                # call A-STAR
                try:
                    occupancy_list = self.a_star(costmap2D, cur_rover_pose, end_point[0:2])
                except SpiralEnd as spiral_error:
                    # TODO: what to do in this case
                    self.traj.reset()
                    occupancy_list = None
                except NoPath as path_error:
                    # increment end point
                    if self.traj.increment_point():
                        # TODO: what to do in this case
                        self.traj.reset()
                    occupancy_list = None
                if occupancy_list is None:
                    self.star_traj = Trajectory(np.array([]))
                else:
                    cartesian_coords = self.ij_to_cartesian(np.array(occupancy_list))
                    print(f"{cartesian_coords}, shape: {cartesian_coords.shape}")
                    self.star_traj = Trajectory(
                        np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1))))
                    )  # current point gets set back to 0
                    
                    path = Path()
                    poses = []
                    path.header = Header()
                    path.header.frame_id = "map"
                    for coord in cartesian_coords:
                        pose_stamped = PoseStamped()
                        pose_stamped.header = Header()
                        pose_stamped.header.frame_id = "map"
                        point = Point(coord[0], coord[1], 0)
                        quat = Quaternion(0,0,0,1)
                        pose_stamped.pose = Pose(point, quat)
                        poses.append(pose_stamped)
                    path.poses = poses
                    self.path_pub.publish(path)

                self.time_last_updated = rospy.get_time()

    def on_enter(self, context) -> None:
        self.context = context
        self.costmap_lock = Lock()
        search_center = context.course.current_waypoint()
        if not self.is_recovering:
            self.traj = SearchTrajectory.spiral_traj(
                context.rover.get_pose().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.tag_id,
                True
            )
            self.origin = context.rover.get_pose().position[0:2]
            print(f"ORIGIN: {self.origin}")
            self.prev_target = None
        self.star_traj = Trajectory(np.array([]))
        self.time_last_updated = rospy.get_time()
        self.costmap_listener = rospy.Subscriber("costmap", OccupancyGrid, self.costmap_callback)
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)

    def on_exit(self, context) -> None:
        self.costmap_listener.unregister()

    def on_loop(self, context) -> State:
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
                print("arrived at sprial point")
                # if we finish the spiral without seeing the object, move on with course
                if self.traj.increment_point():
                    return waypoint.WaypointState()
            else:  # otherwise, increment the astar path
                # if we finish the astar path, then reset astar and increment the spiral path
                if self.star_traj.increment_point():
                    print("arrived at end of astar")
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
