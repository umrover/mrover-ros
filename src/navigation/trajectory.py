from dataclasses import dataclass, field
import numpy as np


@dataclass
class Trajectory:
    # Coordinates of the trajectory
    coordinates: np.ndarray
    # Currently tracked coordinate index along trajectory
    cur_pt: int = field(default=0, init=False)

    def get_cur_pt(self) -> np.ndarray:
        return self.coordinates[self.cur_pt]

    def increment_point(self) -> bool:
        """
        Increments the tracked point in the trajectory, returns true if
        the trajectory is finished
        """
        self.cur_pt += 1
        return self.cur_pt >= len(self.coordinates)

    def __repr__(self):
        result = "Coordinates:\n"
        for pt in range(len(self.coordinates)):
            if pt == self.cur_pt:
                result += "Curr -> "
            result += "[" + str(pt) + "]: " + self.coordinates[pt] + "\n"
        return result
