from dataclasses import dataclass
import numpy as np

from util import np_utils
from typing import Optional
from state import BaseState
from trajectory import Trajectory
from drive import get_drive_command
from aenum import Enum, NoAlias
from context import Context

STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.95
POST_SEPARATION = 2  # meters


@dataclass
class PartialGateTrajectory(Trajectory):
    def partial_gate_traj(post_pos, rover_pos):
        """
        Generates a trajectory to find second of two posts
        :param post_pos:    position of the post (np.ndarray)
        :param rover_pos:   position of the rover (np.ndarray). Assumes that the rover is facing the post
        :return:            list of positions for the rover to traverse List(np.ndarray)
        """
        # If the post is at (a, b) and rover is at (a + x, b + y),
        # we'd calculate rover-to-post as c * (-x, -y).Then,
        #   Point 1 (Left Perp): (a + cy, b - cx)
        #   Point 2 (Opposite): (a - cx, b - cy)
        #   Point 3 (Right Perp): (a - cy, b + cx)
        #   Point 4 (Finish): (a + cx, b + cy)
        # See documentation on the Wiki for visual.

        rover_to_post = post_pos - rover_pos
        rover_to_post = POST_SEPARATION * np_utils.normalized(rover_to_post)
        # scale vector to have magnitude == POST_SEPARATION

        left_perp = np.array([-rover_to_post[1], rover_to_post[0], 0])  # (-y,x)
        right_perp = np.array([rover_to_post[1], -rover_to_post[0], 0])  # (y,-x)

        # This is just making our trajectory points into an array that we can read in
        coords = np.vstack(
            (post_pos + left_perp, post_pos + rover_to_post, post_pos + right_perp, post_pos - rover_to_post)
        )

        return PartialGateTrajectory(coords)


class PartialGateStateTransitions(Enum):
    _settings_ = NoAlias
    # State Transitions
    no_fiducial = "SearchState"
    partial_gate = "PartialGateState"
    found_gate = "GateTraverseState"
    done = "DoneState"


class PartialGateState(BaseState):
    def __init__(
        self,
        context: Context,
    ):
        super().__init__(context, add_outcomes=[transition.name for transition in PartialGateStateTransitions])  # type: ignore
        self.traj: Optional[PartialGateTrajectory] = None

    def evaluate(self, ud):
        post_pos = self.context.env.current_fid_pos()
        if post_pos is None:
            post_pos = self.context.env.other_gate_fid_pos()
        gate = self.context.env.current_gate()

        if gate is not None:  # If we have a gate, we are done
            return PartialGateStateTransitions.found_gate.name  # type: ignore
        elif post_pos is not None:  # Searching for second post
            if self.traj is None:
                self.traj = PartialGateTrajectory.partial_gate_traj(post_pos, self.context.rover.get_pose().position)
        else:
            return PartialGateStateTransitions.no_fiducial.name  # type: ignore

        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = get_drive_command(
            target_pos,
            self.context.rover.get_pose(),
            STOP_THRESH,
            DRIVE_FWD_THRESH,
        )
        if arrived:
            # if we finish the gate path, we're done (or continue search) CHECK THIS***
            if self.traj.increment_point():
                self.traj = None
                self.context.course.increment_waypoint()
                return PartialGateStateTransitions.done.name  # type: ignore

        self.context.rover.send_drive_command(cmd_vel)
        return PartialGateStateTransitions.partial_gate.name  # type: ignore
