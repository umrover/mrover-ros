from typing import Optional, List, Tuple, ClassVar
from dataclasses import dataclass
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from dijkstar import Graph, find_path, NoPathError  # https://pypi.org/project/Dijkstar

from geometry_msgs.msg import Twist
from util.SE3 import SE3

from context import Context
from util.np_utils import angle_to_rotate

MAX_DRIVING_EFFORT = 1
MIN_DRIVING_EFFORT = -1
TURNING_P = 10.0

@dataclass
class FailureZone:
    """
    FailureZones are represented as rectangular bounding boxes 
    aligned with the x and y-axes. 
    """
    vertices: ClassVar[Polygon]           

    def get_vertices(self) -> List[Point]:
        """
        Return a list of 4 vertices as Point objects that form the 
        bounding box of this FailureZone 

        Vertices returned in order [lower_left, upper_left, upper_right, lower_right]
        """
        (min_x, min_y, max_x, max_y) = self.vertices.bounds
        lower_left = Point(min_x, min_y)
        upper_left = Point(min_x, max_y)
        upper_right = Point(max_x, max_y)
        lower_right = Point(max_x, min_y)
        return [lower_left, upper_left, upper_right, lower_right]

    def intersects(self, line: LineString) -> bool:
        """
        Returns whether this FailureZone properly intersects the given line. 

        Intersection is defined as passing through or overlapping with an edge
        of the FailureZone, rather than simply touching a corner. 
        """
        return self.vertices.intersects(line) and not self.vertices.touches(line)


class PathPlanner:
    """
    Implements failure zone avoidance algorithm. 
    """
    failure_zones: List[FailureZone] = []

    visibility_graph: ClassVar[Graph] = Graph(undirected=True)  
    source_pos: ClassVar[Point] = None  
    target_pos: ClassVar[Point] = None 

    path: ClassVar[List[Point]] = None
    target_vertex_idx: int = 0

    def get_intermediate_target(self, source_pos: Point, target_pos: Point) -> Point:
        """
        Given a source position (the current position of the rover) and a 
        target position (the final destination), generates an intermediate target
        position after calculating a clear path between source and destination. 

        This path is recalculated only when the target position changes. Once the
        rover reaches this intermediate target, complete_intermediate_target() should
        be called so that the Route can update and return the next target. 

       :param source_pos:   Point object representing source position
       :param target_pos:   Point object representing target position
        """
        if self.path and target_pos == self.target_pos:
            return self.path[self.target_vertex_idx]
        else:
            self.generate_path(source_pos, target_pos)


    def complete_intermediate_target(self) -> None:
        """
        Marks the previously-returned intermediate target as complete. 
        """  
        assert(self.path) 

        if self.target_vertex_idx < len(self.path):
            self.target_vertex_idx += 1


    def is_path_complete(self) -> bool:
        """
        Returns whether the path has been fully traversed, which happens
        if and only if the target_pos has been marked as completed. 
        """
        return (self.path != None) and (self.target_vertex_idx == len(self.path))


    def is_edge_safe(self, edge: LineString) -> bool:
        """
        Returns whether a given proposed edge is safe. An edge is deemed safe
        if and only if it does not intersect with any failure zones. 

        :param edge:    the edge to be checked for safety 
        """
        for fz in self.failure_zones:
            if fz.intersects(edge):
                return False
        
        return True


    def add_failure_zone(self, failure_zone: FailureZone) -> None:
        """
        Adds a failure zone to the visibility graph 

        :param failure_zone:    the failure zone to be added (as a FailureZone object)
        """
        # Step 1: add to list of failure zones
        self.failure_zones.append(failure_zone)

        # Step 2: remove all edges from current graph that cross this zone
        graph = self.visibility_graph.get_data()
        deleted_edges = []

        for u, neighbors in graph.items():
            for v, edge in neighbors.items():
                if failure_zone.intersects(edge):
                    deleted_edges.append((u, v))
        
        for (u, v) in deleted_edges:
            self.visibility_graph.remove_edge(u, v)

        # Step 3: Add corners to graph
        vertices = failure_zone.get_vertices()
        for vertex in vertices:
            self.add_vertex(vertex)

        # Step 4: Reset path, target, etc. to None
        self.path = None
        self.target_vertex_idx = 0


    def add_vertex(self, new_vertex: Point):
        """
        Adds the given vertex to the visibility graph, along with all 
        associated safe edges, and resets the path. 

        :param new_vertex:  shapely Point object representing the vertex to be added
        """
        # Add vertex to graph
        self.visibility_graph.add_node(new_vertex)

        # Iterate through all vertices; if edge doesn't intersect any FZ, add to graph
        for vertex in self.visibility_graph:
            if(vertex != new_vertex):
                edge = LineString(vertex, new_vertex)
                if self.is_edge_safe(edge):
                    self.visibility_graph.add_edge(vertex, new_vertex, edge)
                
        self.path = None
        self.target_vertex_idx = 0


    def generate_path(self, source_pos: Point, target_pos: Point):
        """
        Given a source_pos and a target pos, generates a shortest path between them
        based on the visibility graph using Dijkstra's algorithm. 

        :param source_pos:  The source position as a shapely 2D Point object
        :param target_pos:  The destination position as a shapely 2D Point object
        """
        # Remove old source, target from the visibility graph
        if self.source_pos:
            self.visibility_graph.remove_node(self.source_pos)
        if self.target_pos:
            self.visibility_graph.remove_node(self.target_pos)

        # Add new source, target to visibility graph 
        self.source_pos = source_pos
        self.add_vertex(source_pos)
        self.target_pos = target_pos
        self.add_vertex(target_pos)

        try:
            self.path = find_path(self.visibility_graph, 
                            source_pos, 
                            target_pos, 
                            cost_func = lambda u, v, e, prev_e : e.length).nodes
        except NoPathError: # how do we handle this case?
            print("No path found.")

        self.target_vertex_idx = 0

@dataclass
class Driver:
    ctx: Context
    planner: PathPlanner = PathPlanner()
    
    def add_failure_zone(self, failure_zone: Polygon) -> None:
        """
        Add a newly-detected failure zone to the planner. 
        """
        self.planner.add_failure_zone(FailureZone(failure_zone))

    def get_drive_command(
        self,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        Gets the drive command for the rover currently at rover_pose and intending to
        reach target_pos while avoiding failure zones. 

        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than
                                        this, stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target 
                                        and current heading in order to drive forward. When below, turn in place.

        :return:                        A tuple containing the rover drive effort command 
                                        to an intermediate target, and a bool indicating 
                                        whether the final destination has been reached. 
        """        
        cmd_vel, reached = self.get_intermediate_target_drive_command(target_pos,
                                                                      rover_pose, completion_thresh, turn_in_place_thresh)
        # if intermediate target already reached, then mark intermediate target
        # as complete and get command to next intermediate target (unless path complete)
        while(reached and not self.planner.is_path_complete()):
            self.planner.complete_intermediate_target()
            cmd_vel, reached = self.get_intermediate_target_drive_command(target_pos,
                                                                    rover_pose, completion_thresh, turn_in_place_thresh)
        return cmd_vel, reached

    def get_intermediate_target_drive_command(
            self, 
            target_pos: np.ndarray, 
            rover_pose: SE3, 
            completion_thresh: float, 
            turn_in_place_thresh: float
    ) -> Tuple[Twist, bool]:
        """
        Given a final target_pos and the current rover_pose, uses the planner to return a drive command to an intermediate target that is on the way to the ultimate goal point.

        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than
                                        this, stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target 
                                        and current heading in order to drive forward. When below, turn in place.

        :return:                        A tuple containing the rover drive effort 
                                        command to the intermediate target, and a bool indicating whether this target has been reached. 
        """
        source_point = Point(rover_pose.position[0:2])  
        target_point = Point(target_pos[0:2])

        curr_target = self.planner.get_intermediate_target(source_point, target_point)
        curr_target_pos = np.ndarray([curr_target.x, curr_target.y, 0])
        return self.get_clear_path_drive_command(curr_target_pos, 
                                                 rover_pose, completion_thresh, turn_in_place_thresh)
        

    def get_clear_path_drive_command(
        self,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        Gets the drive command to the given target_pos. Assumes that the path to the target_pos is clear and free of failure zones. 

        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than
                                        this, stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target 
                                        and current heading in order to drive forward. When below, turn in place.

        :return:                        A tuple containing the rover drive effort 
                                        command, and a bool indicating whether the target 
                                        has been reached 
        """
        if not (0.0 < turn_in_place_thresh < 1.0):
            raise ValueError(f"Argument {turn_in_place_thresh} should be between 0 and 1")
        rover_pos = rover_pose.position
        rover_dir = rover_pose.rotation.direction_vector()
        rover_dir[2] = 0

        # Get vector from rover to target
        target_dir = target_pos - rover_pos
        print(
            f"rover direction: {rover_dir}, target direction: {target_dir}, rover position: {rover_pos} , goal: {target_pos}"
        )

        target_dist = np.linalg.norm(target_dir)
        if target_dist == 0:
            target_dist = np.finfo(float).eps

        alignment = angle_to_rotate(rover_dir, target_dir)

        if target_dist < completion_thresh:
            return Twist(), True

        cmd_vel = Twist()
        full_turn_override = True
        if abs(alignment) < turn_in_place_thresh:
            # We are pretty aligned so we can drive straight
            error = target_dist
            cmd_vel.linear.x = np.clip(error, 0.0, MAX_DRIVING_EFFORT)
            full_turn_override = False

        # we want to drive the angular offset to zero so the error is just 0 - alignment
        error = alignment
        cmd_vel.angular.z = (
            np.sign(error) if full_turn_override else np.clip(error * TURNING_P, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
        )
        print(cmd_vel.linear.x, cmd_vel.angular.z)
        return cmd_vel, False
