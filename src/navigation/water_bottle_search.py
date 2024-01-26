from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rospy
from mrover.msg import GPSPointList
from util.ros_utils import get_rosparam
from util.state_lib.state import State

from nav_msgs.msg import OccupancyGrid, MapMetaData
from navigation import approach_post, recovery, waypoint
from navigation.context import convert_cartesian_to_gps
from navigation.trajectory import Trajectory
from navigation.search import SearchTrajectory

# TODO: Right now this is a copy of search state, we will need to change this
# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit


class WaterBottleSearchState(State):
    # Spiral
    traj: SearchTrajectory
    # when we are moving along the spiral
    prev_target: Optional[np.ndarray] = None
    is_recovering: bool = False

    # TODO: add rosparam names to water_bottle_search/... in the navigation.yaml file
    STOP_THRESH = get_rosparam("search/stop_thresh", 0.5)
    DRIVE_FWD_THRESH = get_rosparam("search/drive_fwd_thresh", 0.34)
    SPIRAL_COVERAGE_RADIUS = get_rosparam("search/coverage_radius", 10)
    SEGMENTS_PER_ROTATION = get_rosparam("search/segments_per_rotation", 8)  # TODO: after testing, might need to change
    DISTANCE_BETWEEN_SPIRALS = get_rosparam(
        "search/distance_between_spirals", 1.25
    )  # TODO: after testing, might need to change

    # TODO: Data Structure to store the information given from the map (Look at navigation)

    # 2D list and map parts
    costMap = []
    height = 0
    width = 0
    resolution = 0

    #Shfited Center
    origin = []
    center = []

    # TODO: Make call_back function to push into data structure

    def costmap_callback(self, msg: OccupancyGrid):
        # update data structure
        self.height = msg.info.height * msg.info.resolution
        self.width = msg.info.width * msg.info.resolution
        self.resolution = msg.info.resolution
        rospy.loginfo(f"height: {self.height}, width: {self.width}")
        costmap2D = np.array(msg.data)
        costmap2D.reshape(self.height, self.width)
        rospy.loginfo(f"2D costmap: {costmap2D}")
        self.origin = self.center - [self.width/2, self.height/2]
        # Call A-STAR

    
    """
    # TODO: A-STAR Algorithm: f(n) = g(n) + h(n)  
    # def a_star():j
    # start = rover pose (Cartesian)
    # end = next point in the spiral
    """
    def a_star(self, costmap2d, start, end) -> list | None:
        #Convert start and end to local cartesian coordinates 


        #Do the check for high cost for the end point
        
        pass
    
    """
    #TODO: Convert global to i and j (Occupancy grid)

    """
    #def cartesian_convert(self, cart_coord: np.ndarray) -> np.ndarray:
        


    def on_enter(self, context) -> None:
        self.listener = rospy.Subscriber("costmap", OccupancyGrid, self.costmap_callback)
        search_center = context.course.current_waypoint()
        if not self.is_recovering:
            self.center = context.rover.get_pose().position[0:2]
            self.traj = SearchTrajectory.spiral_traj(
                context.rover.get_pose().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.fiducial_id,
            )
            self.prev_target = None

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        # continue executing the path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target,
        )
        if arrived:
            self.prev_target = target_pos
            # if we finish the spiral without seeing the fiducial, move on with course
            if self.traj.increment_point():
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False

        context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in self.traj.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)

        if context.env.current_fid_pos() is not None and context.course.look_for_post():
            return approach_post.ApproachPostState()
        return self
