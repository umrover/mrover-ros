from typing import List, ClassVar
from dijkstar import Graph, find_path, NoPathError  # https://pypi.org/project/Dijkstar
from shapely.geometry import LineString, Point, Polygon
from failure_zone import FailureZone

from util.gps_utils import convert_cartesian_to_gps
from mrover.msg import GPSPointList

import numpy as np

"""
Overload < operator for point objects

This is done so that points can have a total ordering, which is required 
to ensure that undirected edges follow a consistent format (which is necessary
to avoid duplicate edges in the visibility graph). 
"""


def point_less(self: Point, other: Point):
    if self.x < other.x:
        return True
    elif self.x > other.x:
        return False
    else:
        return self.y < other.y


Point.__lt__ = point_less


class PathPlanner:
    """
    Implements failure zone avoidance algorithm. A PathPlanner can be used to keep
    track of FailureZone objects and return intermediate target waypoints on the path to
    an overall goal target that avoid FailureZone objects. The path is only updated if a
    FailureZone is added or the overall target changes. The following functions exist
    in this interface:
        - add_failure_zone(FailureZone fz): Adds a failure zone to be avoided
        - get_intermediate_target(Point src, Point target): returns the next intermediate
            target on the current path (as a Point).
        - complete_intermediate_target(): Mark the current intermeidate target as
            completed, such that get_intermediate_target() returns the next target in the
            path the next time it is called.
        - is_path_complete(): returns True/False depending on whether the overall target
            has been marked as completed.
        - generate_path(Point src, Point target): This should not typically be called.
            This overrides the caching protocol and forcefully generates a new path, even
            if the target_pos has not changed. The path can then be accessed normally,
            using get_intermediate_target() and complete_intermediate_target()
    """

    failure_zones: List[FailureZone]
    visibility_graph: Graph

    source_pos: Point
    target_pos: Point

    # these are necessary to ensure that the source and target
    # are not deleted from the graph if they are vertices of a failure zone
    keep_source_in_graph: bool
    keep_target_in_graph: bool

    path: List[Point]
    cur_path_idx: int

    def __init__(self):
        self.failure_zones: List[FailureZone] = []
        self.visibility_graph: Graph = Graph(undirected=True)

        self.source_pos: Point = None
        self.target_pos: Point = None
        self.keep_source_in_graph = False
        self.keep_target_in_graph = False

        self.path: List[Point] = None
        self.cur_path_idx: int = 0
        self.add_failure_zone(FailureZone(Polygon([(0, -1), (0, 1), (1, 1), (1, -1)])))

    def get_intermediate_target(self, source_pos: Point, target_pos: Point) -> Point:
        """
         Given a source position (the current position of the rover) and a target position
         (the final destination), generates an intermediate target position after
         calculating a clear path between source and destination.

         This path is recalculated only when the target position changes, or a FailureZone
         is added to the graph. Once the rover reaches this intermediate target,
         complete_intermediate_target() should be called so that the path can update and
         return the next target on the next call to get_intermediate_target().

         Note: As the path is only recalculated when the target position changes, it is
               assumed that the source_pos of the rover remains along this path. If you
               you want to override this functionality and generate a path, call
               generate_path() before calling this funciton.

        :param source_pos:   Point object representing source position
        :param target_pos:   Point object representing target position

        :return:             A Point object representing the intermediate target
        """
        if (not self.path) or (target_pos != self.target_pos):
            # generate new path if one doesn't exist or target_pos has changed
            self.generate_path(source_pos, target_pos)

        if self.cur_path_idx == len(self.path):
            # path complete, just return last target
            return self.path[-1]

        return self.path[self.cur_path_idx]

    def complete_intermediate_target(self) -> None:
        """
        Marks the previously-returned intermediate target as complete.
        """
        assert self.path

        if self.cur_path_idx < len(self.path):
            self.cur_path_idx += 1

    def is_path_complete(self) -> bool:
        """
        Returns whether the path has been fully traversed, which happens
        if and only if the target_pos has been marked as completed.

        :return: bool
        """
        return (self.path != None) and (self.cur_path_idx == len(self.path))

    def add_failure_zone(self, failure_zone: FailureZone) -> None:
        """
        Adds a failure zone to the visibility graph

        :param failure_zone:    the failure zone to be added (as a FailureZone object)
        """
        # Step 1: add to list of failure zones
        self.failure_zones.append(failure_zone)

        # Step 2: remove all edges from current graph that cross this zone

        # dict of vertex --> neighbors, where each neighbors
        # object is a dict of vertex --> edge
        graph = self.visibility_graph.get_data()
        deleted_edges = []

        for u, neighbors in graph.items():  # (vertex, [(vertex, edge)])
            for v, edge in neighbors.items():
                if u < v:  # avoid duplicate edges being deleted
                    if failure_zone.intersects(edge):
                        deleted_edges.append((u, v))

        for (u, v) in deleted_edges:
            self.visibility_graph.remove_edge(u, v)

        # Step 3: Add corners of this failure zone to graph
        vertices = failure_zone.get_vertices()
        for vertex in vertices:
            self.__add_vertex(vertex)

        # Step 4: Reset path, target, etc.
        self.path = None
        self.cur_path_idx = 0

    def generate_path(self, source_pos: Point, target_pos: Point) -> None:
        """
        Given a source_pos and a target pos, generates a shortest path between them
        based on the visibility graph using Dijkstra's algorithm. Stores it in internal path variable.

        :param source_pos:  The source position as a shapely 2D Point object
        :param target_pos:  The destination position as a shapely 2D Point object
        """

        # Remove old source_pos, target_pos from the visibility graph
        # If old source, target are the same, they would not have been added
        # to the graph in the first place.
        if self.source_pos != self.target_pos:
            if self.source_pos and not self.keep_source_in_graph:
                self.visibility_graph.remove_node(self.source_pos)
            if self.target_pos and not self.keep_target_in_graph:
                self.visibility_graph.remove_node(self.target_pos)

        # if source_pos = target_pos, don't bother generating a path
        if source_pos == target_pos:
            self.source_pos = source_pos
            self.target_pos = target_pos
            self.path = [source_pos]
            self.cur_path_idx = 0
            return

        # Add new source, target to visibility graph
        self.source_pos = source_pos
        self.keep_source_in_graph = True
        if source_pos not in self.visibility_graph:
            self.__add_vertex(source_pos)
            self.keep_source_in_graph = False

        self.target_pos = target_pos
        self.keep_target_in_graph = True
        if target_pos not in self.visibility_graph:
            self.__add_vertex(target_pos)
            self.keep_target_in_graph = False

        try:
            self.path = find_path(
                self.visibility_graph, source_pos, target_pos, cost_func=lambda u, v, e, prev_e: e.length
            ).nodes
        except NoPathError:
            source_in_fz = False
            target_in_fz = False

            source_fz = None

            for fz in self.failure_zones:
                if fz.intersects(source_pos):
                    source_in_fz = True
                    source_fz = fz
                if fz.intersects(target_pos):
                    target_in_fz = True

            if source_in_fz and not target_in_fz:
                # if the source is in a failure zone, get out of the failure zone
                # as fast as possible, and then plan the path to the target
                closest_vertex = source_fz.get_closest_vertex(source_pos)
                try:
                    path = find_path(
                        self.visibility_graph, closest_vertex, target_pos, cost_func=lambda u, v, e, prev_e: e.length
                    ).nodes
                except NoPathError:
                    self.path = [source_pos, target_pos]

                self.path = [source_pos] + path
            else:
                # if no clear path found, just construct a straight-line to the target
                self.path = [source_pos, target_pos]

        self.cur_path_idx = 0

        # publish to drive_path topic
        gps_point_list = []
        for pt in self.path:
            gps_point_list.append(convert_cartesian_to_gps(np.array([pt.x, pt.y])))

        # self.context.drive_path_publisher.publish(GPSPointList(gps_point_list))

    def __add_vertex(self, new_vertex: Point) -> None:
        """
        Adds the given vertex to the visibility graph, along with all
        associated safe edges, and resets the path.

        :param new_vertex:  shapely Point object representing the vertex to be added
        """
        if new_vertex in self.visibility_graph:
            return

        # Add vertex to graph
        self.visibility_graph.add_node(new_vertex)

        # Iterate through all vertices; if edge doesn't intersect any FZ, add to graph
        for vertex in self.visibility_graph:
            if vertex != new_vertex:
                edge = LineString([vertex, new_vertex])
                if self.__is_edge_safe(edge):
                    self.visibility_graph.add_edge(vertex, new_vertex, edge)

        self.path = None
        self.cur_path_idx = 0

    def __is_edge_safe(self, edge: LineString) -> bool:
        """
        Returns whether a given proposed edge is safe. An edge is deemed safe
        if and only if it does not intersect with any failure zones.

        :param edge:    the edge to be checked for safety

        :return:        bool -- True if no FailureZone objects intersected by given edge,
                        False otherwise.
        """
        for fz in self.failure_zones:
            if fz.intersects(edge):
                return False

        return True
