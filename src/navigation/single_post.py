from dataclasses import dataclass
from turtle import done
import numpy as np

from typing import Optional
from state import BaseState
from trajectory import Trajectory
from drive import get_drive_command
from aenum import Enum, NoAlias
from context import Context

STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.95
POST_SEPARATION = 2 #meters

@dataclass
class SinglePostTrajectory(Trajectory):
    def single_post_traj(self, post_pos, rover_pos):
        """
        Generates a trajectory to find second of two posts
        :param post_pos:    position of the post (np.ndarray)
        :param rover_pos:   position of the rover (np.ndarray). Assumes that the rover is facing the post
        :return:            list of positions for the rover to traverse List(np.ndarray)
        """
        #The Path will be (assuming the vector <x, y> from rover to post
        # Point one: left perpendicular, <-y,x>/mag * Post_separation
        # Point two: opposite end, <x,y>/mag * Post_separation
        # Point three: right perpendicular, <y,-x>/mag * Post_separation
        # Essentially traveling around the circle made by the first post

        rover_to_post = post_pos - rover_pos
        rover_to_post *= POST_SEPARATION / np.linalg.norm(rover_to_post) 
        #scale vector to have magnitude == POST_SEPARATION
        
        left_perp = np.array([-rover_to_post[1], rover_to_post[0]]) #(-y,x)\
        right_perp = np.array([rover_to_post[1], -rover_to_post[0]]) #(y,-x)
        coords = np.vstack((post_pos + left_perp, post_pos + rover_to_post, post_pos + right_perp))
        coords = np.hstack((coords, np.zeros(coords.shape[0]).reshape(-1, 1)))

        return SinglePostTrajectory(coords)
class SinglePostStateTransitions(Enum):
    _settings_ = NoAlias
    # State Transitions
    no_fiducial = "SearchState"
    single_post = "SinglePostState"
    found_gate = "GateTraverseState"
    done = "DoneState"


class SinglePostState(BaseState):
    def __init__(
        self,
        context: Context,
    ):
        super().__init__(
            context,
            add_outcomes=[transition.name for transition in SinglePostStateTransitions]
        )
        self.traj: Optional[SinglePostTrajectory] = None
    
    def evalutate(self):
        post_pos = self.context.env.current_fid_pos();
        gate = self.context.env.current_gate();

        if gate is not None: #If we have a gate, we are done
            return SinglePostStateTransitions.found_gate.name
        elif post_pos is not None: #Searching for second post
            if self.traj is None:
                self.traj = SinglePostTrajectory.single_post_traj(post_pos, self.context.rover.get_pose().position)
        else: 
            return SinglePostStateTransitions.no_fiducial.name

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
                return SinglePostStateTransitions.done.name
        
        self.context.rover.send_drive_command(cmd_vel)

        return SinglePostStateTransitions.single_post.name
