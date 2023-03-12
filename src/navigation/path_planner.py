from typing import List, ClassVar
from dijkstar import Graph, find_path, NoPathError  # https://pypi.org/project/Dijkstar
from shapely.geometry import LineString, Point
from failure_zone import FailureZone

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