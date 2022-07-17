from typing import List

import numpy as np

from context import Context
from state import BaseState


def gen_square_spiral_search_pattern(center: np.ndarray, spacing: float, coverage_radius: float) -> List[np.ndarray]:
    """
    Generates a square spiral search pattern around a center position, assumes rover is at the center position
    :param center:          position to center spiral on (np.ndarray),
    :param spacing:         distance in between each spiral (float)
    :param coverage_radius: spiral covers at least this far out
    :return:                list of positions for the rover to traverse List(np.ndarray)
    """
    # TODO: refactor
    out = []
    cur = center
    cur_segment_len = spacing / 2
    directions = [
        np.array([1, 0]),
        np.array([0, -1]),
        np.array([-1, 0]),
        np.array([1, 0]),
    ]
    direction_index = 0
    while cur_segment_len <= coverage_radius:
        next_point = cur + (cur_segment_len * directions[direction_index])
        out.append(next_point)
        cur_segment_len += spacing / 2
        direction_index = (direction_index + 1) % len(directions)
    return out


class SearchState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["found_tag", "finished_search"],
            add_input_keys=["search_point_index", "searchPoints"],
            add_output_keys=["search_point_index"],
        )

    def evaluate(self, ud):
        pass
