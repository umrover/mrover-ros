import rospy
from mrover.msg import WaypointType
from mrover.srv import MoveCostMap
from navigation import (
    search,
    recovery,
    post_backup,
    state,
    water_bottle_search,
)
from navigation.context import Context
from util.state_lib.state import State


class WaypointState(State):
    STOP_THRESHOLD: float = rospy.get_param("waypoint/stop_threshold")
    DRIVE_FORWARD_THRESHOLD: float = rospy.get_param("waypoint/drive_forward_threshold")
    USE_COSTMAP: bool = rospy.get_param("water_bottle_search/use_costmap")
    NO_TAG: int = -1

    def on_enter(self, context: Context) -> None:
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        assert current_waypoint is not None

        if current_waypoint.type.val == WaypointType.WATER_BOTTLE:
            rospy.wait_for_service("move_cost_map")
            move_cost_map = rospy.ServiceProxy("move_cost_map", MoveCostMap)
            try:
                move_cost_map(f"course{context.course.waypoint_index}")
            except rospy.ServiceException as exc:
                rospy.logerr(f"Service call failed: {exc}")

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a tag id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param ud:  State machine user data
        :return:    Next state
        """
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return state.DoneState()

        # If we are at a post currently (from a previous leg), backup to avoid collision
        if context.env.arrived_at_target:
            context.env.arrived_at_target = False
            return post_backup.PostBackupState()

        # Returns either ApproachTargetState, LongRangeState, or None
        approach_state = context.course.get_approach_state()
        if approach_state is not None:
            return approach_state

        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        # Attempt to find the waypoint in the TF tree and drive to it
        waypoint_position_in_map = context.course.current_waypoint_pose_in_map().position
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            waypoint_position_in_map,
            rover_in_map,
            self.STOP_THRESHOLD,
            self.DRIVE_FORWARD_THRESHOLD,
        )
        if arrived:
            context.env.arrived_at_waypoint = True
            if not context.course.look_for_post() and not context.course.look_for_object():
                # We finished a regular waypoint, go onto the next one
                context.course.increment_waypoint()
            elif current_waypoint.type.val == WaypointType.WATER_BOTTLE and self.USE_COSTMAP:
                # We finished a waypoint associated with the water bottle, but we have not seen it yet and are using the costmap to search
                return water_bottle_search.WaterBottleSearchState()
            else:
                # We finished a waypoint associated with a post, mallet, or water bottle, but we have not seen it yet (no costmap for search).
                search_state = search.SearchState()
                search_state.new_trajectory(context)
                return search_state

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        context.rover.send_drive_command(cmd_vel)

        return self
