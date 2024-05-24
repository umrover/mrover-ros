from dataclasses import dataclass, field

import numpy as np


@dataclass
class Trajectory:
    # Coordinates of the trajectory
    coordinates: np.ndarray
    # Currently tracked coordinate index along trajectory
    cur_pt: int = field(default=0, init=False)

    def get_current_point(self) -> np.ndarray:
        return self.coordinates[self.cur_pt]

    def increment_point(self) -> bool:
        """
        Increments the tracked point in the trajectory, returns true if
        the trajectory is finished
        """
        self.cur_pt += 1
        return self.cur_pt >= len(self.coordinates)

    def reset(self) -> None:
        """
        Resets the trajectory back to its start
        """
        self.cur_pt = 0


@dataclass
class SearchTrajectory(Trajectory):
    # Associated tag for this trajectory
    tag_id: int

    @classmethod
    def gen_spiral_coordinates(
        cls, coverage_radius: float, distance_between_spirals: float, num_segments_per_rotation: int, insert_extra: bool
    ) -> np.ndarray:
        """
        Generates a set of coordinates for a spiral search pattern centered at the origin
        :param coverage_radius              radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiralradii = angles * (distance_between_spirals / (2*np.pi)) (float)
        :param num_segments_per_rotation:   number of segments that the spiral has per rotation (int)
        :return                             np.ndarray of coordinates
        """
        # the number of spirals should ensure coverage of the entire radius.
        # We add 1 to ensure that the last spiral covers the radius along the entire rotation,
        # as otherwise we will just make the outermost point touch the radius
        num_spirals = np.ceil(coverage_radius / distance_between_spirals).astype("int") + 1
        # the angles are evenly spaced between 0 and 2pi*num_segments_per_rotation (add one to the number of points because N+1 points make N segments)
        angles = np.linspace(0, 2 * np.pi * num_spirals, num_segments_per_rotation * num_spirals + 1)
        # radii are computed via following polar formula.
        # This is correct because you want the radius to increase by 'distance_between_spirals' every 2pi radians (one rotation)
        radii = angles * (distance_between_spirals / (2 * np.pi))
        # convert to cartesian coordinates
        xcoords = np.cos(angles) * radii
        ycoords = np.sin(angles) * radii
        # we want to return as a 2D matrix where each row is a coordinate pair
        # so we reshape x and y coordinates to be (n, 1) matricies then stack horizontally to get (n, 2) matrix
        vertices = np.hstack((xcoords.reshape(-1, 1), ycoords.reshape(-1, 1)))
        all_points = []
        if insert_extra:
            for i in range(len(vertices) - 1):
                all_points.append(vertices[i])
                vector = vertices[i + 1] - vertices[i]
                magnitude = np.linalg.norm(vector)
                unit_vector = vector / magnitude
                count = 0.0
                while count < magnitude - 3.5:
                    all_points.append(all_points[-1] + (unit_vector * 2.5))  # TODO: figure out how far apart to insert
                    count += 2.5
            return np.array(all_points)

        return vertices

    @classmethod
    def spiral_traj(
        cls,
        center: np.ndarray,
        coverage_radius: float,
        distance_between_spirals: float,
        segments_per_rotation: int,
        tag_id: int,
        insert_extra: bool,
    ):
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:                      position to center spiral on (np.ndarray)
        :param coverage_radius:             radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiral (float)
        :param segments_per_rotation:       number of segments per spiral (int), for example, 4 segments per rotation would be a square spiral, 8 segments per rotation would be an octagonal spiral
        :param tag_id:                      tag id to associate with this trajectory (int)
        :param insert_extra:
        :return:    SearchTrajectory object
        """
        zero_centered_spiral_r2 = cls.gen_spiral_coordinates(
            coverage_radius, distance_between_spirals, segments_per_rotation, insert_extra
        )

        # numpy broadcasting magic to add center to each row of the spiral coordinates
        spiral_coordinates_r2 = zero_centered_spiral_r2 + center
        # add a column of zeros to make it 3D
        spiral_coordinates_r3 = np.hstack(
            (spiral_coordinates_r2, np.zeros(spiral_coordinates_r2.shape[0]).reshape(-1, 1))
        )
        return SearchTrajectory(
            spiral_coordinates_r3,
            tag_id,
        )
