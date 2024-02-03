from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import heapq
import numpy as np
import random
import math
import rospy
from mrover.msg import GPSPointList
from util.ros_utils import get_rosparam
from util.state_lib.state import State
from nav_msgs.msg import OccupancyGrid, MapMetaData
from navigation import approach_post, recovery, waypoint
from navigation.context import convert_cartesian_to_gps
from navigation.trajectory import Trajectory
from navigation.search import SearchTrajectory


# TODO: Right now this is a copy of search state, we will need to change this
# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class WaterBottleSearchState(State):
    class Node:
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
    
    traj: SearchTrajectory # spiral
    star_traj: Trajectory # returned by astar
    # when we are moving along the spiral
    prev_target: Optional[np.ndarray] = None
    is_recovering: bool = False
    height: int = 0 # height of occupancy grid 
    width: int = 0 # width of occupancy grid
    resolution: int = 0 # resolution of occupancy 
    origin: np.ndarray = np.ndarray([]) # hold the inital rover pose
    
    STOP_THRESH = get_rosparam("water_bottle_search/stop_thresh", 0.5)
    DRIVE_FWD_THRESH = get_rosparam("water_bottle_search/drive_fwd_thresh", 0.34)
    SPIRAL_COVERAGE_RADIUS = get_rosparam("water_bottle_search/coverage_radius", 10)
    SEGMENTS_PER_ROTATION = get_rosparam("water_bottle_search/segments_per_rotation", 8)  # TODO: after testing, might need to change
    DISTANCE_BETWEEN_SPIRALS = get_rosparam(
        "water_bottle_search/distance_between_spirals", 1.25
    )  # TODO: after testing, might need to change

    def costmap_callback(self, msg: OccupancyGrid):
        """
        Callback function 
        :param msg: Occupancy Grid representative of a 30 x 30m square area with origin at GNSS waypoint. Values are 0, 1, -1
        """
        self.height = msg.info.height * msg.info.resolution
        self.width = msg.info.width * msg.info.resolution
        self.resolution = msg.info.resolution
        rospy.loginfo(f"height: {self.height}, width: {self.width}")
        costmap2D = np.array(msg.data)
        costmap2D.reshape(self.height, self.width)
        rospy.loginfo(f"2D costmap: {costmap2D}")
        
        rover_pose = self.context.rover.get_pose().position[0:2]
        self.traj.increment_point()
        end_point = self.traj.get_cur_pt()
        
        # call A-STAR
        occupancy_list = self.a_star(costmap2d = costmap2D, start = rover_pose, end = end_point)
        if occupancy_list is not None:
            self.star_traj = self.ij_to_cartesian(np.array(occupancy_list))
            self.star_traj = (map(self.ij_to_cartesian, occupancy_list))
        else:
            self.star_traj = []
        # reset current point to start of list
        self.star_traj.reset_cur_pt()
        
    
    def cartesian_to_ij(self, cart_coord: np.ndarray) -> np.ndarray:
        """
        Convert real world cartesian coordinates (x, y) to coordinates in the occupancy grid (i, j)
        :param cart_coord: array of x and y cartesian coordinates
        :return: array of i and j coordinates for the occupancy grid
        """
        width_height  = [-self.width / 2, self.height / 2] # these are the width and height list (m/cell)
        converted_coord = (cart_coord - (self.origin + (width_height) / self.resolution))* [1,-1] 
        return math.floor(converted_coord)
    
    def ij_to_cartesian(self, ij_coords : np.ndarray) -> np.ndarray:
        """
        Convert coordinates in the occupancy grid (i, j) to real world cartesian coordinates (x, y)
        using formula (WP - [W/2, H/2]) + [j x r, -i x r] + [r/2, -r/2] where WP is the origin.
        :param ij_coords: array of i and j occupancy grid coordinates
        :return: array of x and y coordinates in the real world
        """
        width_height = [self.width/2, self.height/2] # [W/2, H/2] width and height list (m/cell)
        resolution_conversion = ij_coords * [self.resolution, -1 * self.resolution] # [j x r, -i x r] 
        half_res = [self.resolution/2, -self.resolution/2] # [r/2, -r/2]
        return self.origin - width_height + resolution_conversion + half_res
    
    
    def return_path(current_node):
        """
        It returns the path given from A-Star in reverse through current node's parents 
        :param current_node: end point of path and contains parents to retrieve path
        """
        path = []
        current = current_node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1]  # Return reversed path
    
    def a_star(self, costmap2d, start: np.ndarray, end: np.ndarray) -> list | None:
        """
        A-STAR Algorithm: f(n) = g(n) + h(n)
        :param start: rover pose (cartesian)
        :param end: next point in the spiral from traj (cartesian)
        :return: list of A-STAR coordinates in the occupancy grid coordinates (i,j)
        """
        #Convert start and end to local cartesian coordinates
        startij  = self.cartesian_to_ij(start)
        endij = self.cartesian_to_ij(end)
        #Do the check for high cost for the end point
        if costmap2d[endij[0]][endij[1]] >= 100:
            if not self.traj.increment_point():
                end = self.traj.get_cur_pt
            #TODO What do we do in this case where we don't have another point in da spiral
            else:
                return None
        #Intialize nodes:
        start_node = self.Node(None, startij)
        start_node.g = start_node.h = start_node.f = 0
        end_node = self.Node(None, endij)
        end_node.g = end_node.h = end_node.f = 0
        # Initialize both open and closed list
        open_list = []
        closed_list = []
        # Heapify the open_list and Add the start node
        heapq.heapify(open_list)
        heapq.heappush(open_list, start_node)
        # Adding a stop condition
        outer_iterations = 0
        max_iterations = (len(costmap2d[0]) * len(costmap2d) // 2)
        #Movements:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1))
        adjacent_square_pick_index = [0, 1, 2, 3, 4, 5, 6, 7]
        
        # Loop until you find the end
        while len(open_list) > 0:
            #Randomize the order of the adjacent_squares_pick_index to avoid a decision making bias
            random.shuffle(adjacent_square_pick_index)
            outer_iterations += 1
            if outer_iterations > max_iterations:
                # if we hit this point return the path such as it is
                # it will not contain the destination
                #warn("giving up on pathfinding too many iterations")
                return self.return_path(current_node)
            # Get the current node
            current_node = heapq.heappop(open_list)
            closed_list.append(current_node)
            # Found the goal
            if current_node == end_node:
                return self.return_path(current_node)
            # Generate children
            children = []
            for pick_index in adjacent_square_pick_index:
                new_position = adjacent_squares[pick_index]
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                # Make sure within range
                if node_position[0] > (len(costmap2d) - 1) or node_position[0] < 0 or node_position[1] > (len(costmap2d[len(costmap2d)-1]) -1) or node_position[1] < 0:
                    continue
                # Make sure walkable terrain
                # if costmap2d[node_position[0]][node_position[1]] != 0:
                #     continue
                # Create new node
                new_node = self.Node(current_node, node_position)
                # Append
                children.append(new_node)
            # Loop through children
            for child in children:
                # Child is on the closed list
                if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                    continue
                # Create the f, g, and h values
                child.g = current_node.g + costmap2d[child.position[0]][child.position[1]]
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h
                # Child is already in the open list
                if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                    continue
                # Add the child to the open list
                heapq.heappush(open_list, child)
                
        return self.return_path(current_node)
                
                
    def on_enter(self, context) -> None:
        self.listener = rospy.Subscriber("costmap", OccupancyGrid, self.costmap_callback)
        search_center = context.course.current_waypoint()
        if not self.is_recovering:
            self.traj = SearchTrajectory.spiral_traj(
                context.rover.get_pose().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.fiducial_id,
            )
            self.origin = context.rover.get_pose().position[0:2]
            self.prev_target = None
    def on_exit(self, context) -> None:
        pass
    def on_loop(self, context) -> State:
        # continue executing the path from wherever it left off
        #if self.star_traj is None:
            
        target_pos = self.star_traj.get_cur_pt()
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target,
        )
        if arrived:
            self.prev_target = target_pos
            # if we finish the spiral without seeing the fiducial, move on with course
            if self.star_traj.increment_point():
                return waypoint.WaypointState()
        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False
        context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in self.star_traj.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)
        if context.env.current_fid_pos() is not None and context.course.look_for_post():
            return approach_post.ApproachPostState()
        return self
