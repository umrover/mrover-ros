from __future__ import annotations

import heapq
import random
from threading import Lock

import numpy as np

from navigation.context import Context

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

class AStar:
    origin: np.ndarray = np.array([])  # holds the inital rover pose (waypoint of water bottle)
    context: Context
    costmap_lock: Lock

    def __init__(self, origin: np.ndarray, context: Context)-> None: 
        self.origin = origin
        self.context = context
        self.costmap_lock = Lock()

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
        width = self.context.env.cost_map.width * self.context.env.cost_map.resolution
        height = self.context.env.cost_map.height * self.context.env.cost_map.resolution
        width_height = np.array([-1 * width / 2, height / 2])  # [-W/2, H/2]
        converted_coord = np.floor((cart_coord[0:2] - (self.origin + width_height)) / self.context.env.cost_map.resolution)
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
        width = self.context.env.cost_map.width * self.context.env.cost_map.resolution
        height = self.context.env.cost_map.height * self.context.env.cost_map.resolution
        width_height = np.array([width / 2, height / 2])  # [W/2, H/2]
        resolution_conversion = ij_coords * [self.context.env.cost_map.resolution, self.context.env.cost_map.resolution]  # [j * r, i * r]
        half_res = np.array([self.context.env.cost_map.resolution / 2, -1 * self.context.env.cost_map.resolution / 2])  # [r/2, -r/2]
        return ((self.origin - width_height) + resolution_conversion + half_res) * np.array([1, -1])

    def return_path(self, current_node: Node) -> list:
        """
        Return the path given from A-Star in reverse through current node's parents
        :param current_node: end point of path which contains parents to retrieve path
        :param costmap2D: current costmap
        :return: reversed path except the starting point (we are already there)
        """
        costmap2D = np.copy(self.context.env.cost_map.data)
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

    def a_star(self, start: np.ndarray, end: np.ndarray) -> list | None:
        """
        A-STAR Algorithm: f(n) = g(n) + h(n) to find a path from the given start to the given end in the given costmap
        :param costmap2d: occupancy grid in the form of an 2D array of floats
        :param start: rover pose in cartesian coordinates
        :param end: next point in the spiral from traj in cartesian coordinates
        :return: list of A-STAR coordinates in the occupancy grid coordinates (i,j)
        """
        with self.costmap_lock:
            costmap2d = self.context.env.cost_map.data
            # convert start and end to occupancy grid coordinates
            startij = self.cartesian_to_ij(start)
            endij = self.cartesian_to_ij(end)

            # initialize start and end nodes
            start_node = self.Node(None, (startij[0], startij[1]))
            end_node = self.Node(None, (endij[0], endij[1]))

            if start_node == end_node:
                return None

            print(f"start: {start}, end: {end}")
            print(f"startij: {startij}, endij: {endij}")

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
                    return self.return_path(current_node)

                # get the current node
                current_node = heapq.heappop(open_list)
                closed_list.append(current_node)

                # found the goal
                if current_node == end_node:
                    return self.return_path(current_node)

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
                    new_node = self.Node(current_node, node_position)
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
            