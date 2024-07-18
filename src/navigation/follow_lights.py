from typing import Optional

import numpy as np

import rospy
from mrover.msg import GPSPointList, WaypointType
from navigation import recovery, waypoint
from navigation.context import convert_cartesian_to_gps, Context
from util.state_lib.state import State
from navigation.state import DoneState
from navigation.approach_target import ApproachTargetState
from util.SE3 import SE3


class FollowLightsState(State):
    """
        This state aims to follow trails of lights for CIRC's traversal mission

        I will be using an approach similar to object approach rather than trajectories, because
        trajectories are used for well defined paths while this is still heavily reliant on the 
        perception currently avaiable
    """
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False

    STOP_THRESH = rospy.get_param("search/stop_threshold")
    DRIVE_FORWARD_THRESHOLD = rospy.get_param("search/drive_forward_threshold")
    SPIRAL_COVERAGE_RADIUS = rospy.get_param("search/coverage_radius")
    SEGMENTS_PER_ROTATION = rospy.get_param("search/segments_per_rotation")
    DISTANCE_BETWEEN_SPIRALS = rospy.get_param("search/distance_between_spirals")

    OBJECT_SPIRAL_COVERAGE_RADIUS = rospy.get_param("object_search/coverage_radius")
    OBJECT_DISTANCE_BETWEEN_SPIRALS = rospy.get_param("object_search/distance_between_spirals")

    def on_enter(self, context: Context) -> None:
        self.light_points: dict[tuple[int, int], tuple[SE3, bool]] = dict()
        self.current_closest_light: SE3 = None
        self.current_closest_distance = np.inf
        pass

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        rover_in_map = context.rover.get_pose_in_map()

        print(self.light_points)

        if rover_in_map is not None:

            # Find all of the lights that exist inside of the TF tree
            numLightsSeen = 1
            hasSeenMax = False
            while not hasSeenMax:
                print("LOOP")
                try:
                    light_frame = f'light{numLightsSeen}'
                    light_in_map = SE3.from_tf_tree(context.tf_buffer, parent_frame="map", child_frame=light_frame)

                    # Create the point tuple key
                    light_tuple = (int(light_in_map.position[0]), int(light_in_map.position[1]))

                    # If the point is not in the map then add it as unvisited
                    if(not self.light_points.__contains__(light_tuple)):
                        self.light_points[light_tuple] = (light_in_map, False)
                    
                    numLightsSeen += 1

                except:
                    # If the TF lookup fails, then we know that there are no more available light points to look at in the TF tree
                    hasSeenMax = True

            # If the rover does not have a current target, find the closest point. Otherwise use the current target
            if(self.current_closest_light is None):
                for _, (light_location_value, seen) in self.light_points.items():
                    current_distance: float = np.linalg.norm(np.subtract(light_location_value.position, rover_in_map.position))
                    if self.current_closest_distance > current_distance and not seen:
                        self.current_closest_light = light_location_value
                        self.current_closest_distance = current_distance
            
            # Drive towards the closest point
            if self.current_closest_light is not None:
                cmd_vel, arrived = context.rover.driver.get_drive_command(
                    self.current_closest_light.position,
                    rover_in_map,
                    self.STOP_THRESH,
                    self.DRIVE_FORWARD_THRESHOLD,
                    path_start=self.prev_target_pos_in_map,
                )

                if arrived:
                    # Upon arrival mark the current point as traveled to in the tf tree
                    light_tuple = (int(self.current_closest_light.position[0]), int(self.current_closest_light.position[1]))
                    self.light_points[light_tuple] = (self.light_points[light_tuple][0], True)

                    # Remove the current target from being a target
                    # These values are ok to modify because they are not used in later in this loop, only in the next cycle
                    self.current_closest_light = None
                    self.current_closest_distance = np.inf

                # TODO: Fix (john) This is written quite poorly, but
                # it doesnt fit into existing infrastructure because 
                # we don't necessarily know which post we are searching for
                # This searches to see if we have seen any of the 100 Aruco tags
                for i in range(100):
                    if context.env.get_target_position(f'tag{i}') is not None:
                        # After seeing the tag, we change the current type of 
                        # waypoint that we are at to be a post with the id we just saw.
                        # Then we transition into approaching that post
                        context.course.current_waypoint().tag_id = i
                        context.course.current_waypoint().type.val = WaypointType.POST
                        return ApproachTargetState()

                context.rover.send_drive_command(cmd_vel)

        # This is important to be self and not FollowLightsState() because we need the dictionary to persist through iterations
        return self