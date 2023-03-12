from typing import List, ClassVar
from dijkstar import Graph, find_path, NoPathError  # https://pypi.org/project/Dijkstar
from shapely.geometry import LineString, Point
from failure_zone import FailureZone

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
    failure_zones: ClassVar[List[FailureZone]] = []
    visibility_graph: ClassVar[Graph] = Graph(undirected=True)  

    source_pos: ClassVar[Point] = None  
    target_pos: ClassVar[Point] = None 
    path: ClassVar[List[Point]] = None
    target_vertex_idx: ClassVar[int] = 0


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
        if (not self.path) or (not target_pos == self.target_pos):
            self.generate_path(source_pos, target_pos)
        
        return self.path[self.target_vertex_idx]


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

        :return: bool 
        """
        return (self.path != None) and (self.target_vertex_idx == len(self.path))


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

        # Step 3: Add corners of this failure zone to graph
        vertices = failure_zone.get_vertices()
        for vertex in vertices:
            self.__add_vertex(vertex)

        # Step 4: Reset path, target, etc.
        self.path = None
        self.target_vertex_idx = 0
    

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
        if(self.source_pos != self.target_pos):
            if self.source_pos:
                self.visibility_graph.remove_node(self.source_pos)
            if self.target_pos:
                self.visibility_graph.remove_node(self.target_pos)

        # if source_pos = target_pos, don't bother generating a path
        if source_pos == target_pos:
            self.source_pos = source_pos
            self.target_pos = target_pos    
            self.path = [source_pos]
            self.target_vertex_idx = 0
            return

        # Add new source, target to visibility graph 
        self.source_pos = source_pos
        self.__add_vertex(source_pos)
        self.target_pos = target_pos
        self.__add_vertex(target_pos)

        try:
            self.path = find_path(self.visibility_graph, 
                            source_pos, 
                            target_pos, 
                            cost_func = lambda u, v, e, prev_e : e.length).nodes
        except NoPathError: 
        # if no clear path found, just construct a straight-line to the target
            self.path = [source_pos, target_pos] 

        self.target_vertex_idx = 0
    

    def __add_vertex(self, new_vertex: Point) -> None:
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
                if self.__is_edge_safe(edge):
                    self.visibility_graph.add_edge(vertex, new_vertex, edge)
                
        self.path = None
        self.target_vertex_idx = 0
    

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