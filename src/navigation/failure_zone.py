from typing import List, ClassVar
from dataclasses import dataclass
from shapely.geometry import Polygon, LineString, Point


@dataclass
class FailureZone:
    """
    FailureZones are represented as rectangular bounding boxes
    aligned with the x and y-axes.

    FailureZones can be initialized by passing in a Shapely Polygon
    object. This will be normalized to a rectangular, x-y aligned bounding box.

    NOTE: FailureZones should typically be intialized such that the Polygon is
    larger than it strictly needs to be; the rover WILL pass through the vertices
    and edges of the failure zone while planning a path if the zone is in the way.
    """

    vertices: Polygon

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
