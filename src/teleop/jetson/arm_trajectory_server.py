#!/usr/bin/env python3
# Adapted from https://github.com/ros-industrial/robot_movement_interface/blob/master/ur_driver/scripts/joint_trajectory_action.py
import rospy
import time
import actionlib
import numpy as np
from threading import Lock
from collections import deque
from typing import Deque, List
from util.ros_utils import get_rosparam

from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

conf_joint_names = get_rosparam("teleop/ra_joints/", ["joint_a, joint_b, joint_c, joint_d, joint_e"])
lock = Lock()
joint_states = JointState()


# ------------------------------------------------------------------------
# Callback function executed after the publication of the current robot position
# ------------------------------------------------------------------------
def joint_states_callback(msg: JointState):
    with lock:
        global joint_states
        joint_states = msg


def euclidean_error(threshold: float, feedback: FollowJointTrajectoryFeedback) -> str:
    """
    Computes the norm of the measured joint position errors and compares it against the given threshold.
    Returns an error message if the threshold is exceeded, and an empty string if it is not.
    """
    position_errors = np.array(feedback.error.positions)
    error = np.linalg.norm(position_errors)

    if error > threshold:
        return f"Euclidean error of {error} exceeded threshold of {threshold}"
    return ""


def joint_error(thresholds: List[float], feedback: FollowJointTrajectoryFeedback) -> str:
    """
    Compares the position errors of each joint against the given list of thresholds.
    Returns an error message if a joint's threshold is exceeded, and an empty string if no threshold is exceeded.
    """
    position_errors = feedback.error.positions

    for i in range(len(thresholds)):
        if position_errors[i] > thresholds[i]:
            return f"""
                Joint {chr(i+65)} exceeded error threshold of {thresholds[i]}
                Expected: {feedback.desired.positions[i]} rad")
                Actual: {feedback.actual.positions[i]} rad")
            """
    return ""


# Return an error message if arm has exceeded the error thresholds and an empty string otherwise
def error_threshold_exceeded(feedback: FollowJointTrajectoryFeedback) -> str:
    euclidean_error_threshold = get_rosparam("teleop/euclidean_error_threshold", 3.14)
    joint_error_thresholds = [x for _, x in sorted(get_rosparam("teleop/joint_error_thresholds", 1.57).items())]

    error = euclidean_error(euclidean_error_threshold, feedback)
    if error:
        return error
    return joint_error(joint_error_thresholds, feedback)


class MoveItAction(object):
    # Rearranges the point path following the name convention joint_0, ... joint_6
    def rearrange(self, joint_trajectory: JointTrajectory) -> None:
        mapping = [joint_trajectory.joint_names.index(j) for j in conf_joint_names]

        # Return early if already arranged properly
        if mapping == sorted(mapping):
            return
        for point in joint_trajectory.points:
            temp_positions: List[float] = []
            temp_velocities: List[float] = []
            temp_accelerations: List[float] = []
            temp_effort: List[float] = []

            # Motion Plan will publish 5 (every joint besides f)
            # postions velocities and accelerations for each point of the plan
            for i in range(len(point.positions)):
                temp_positions.append(point.positions[mapping[i]])
                temp_velocities.append(point.velocities[mapping[i]])
                temp_accelerations.append(point.accelerations[mapping[i]])

            point.positions = temp_positions
            point.velocities = temp_velocities
            point.accelerations = temp_accelerations
            point.effort = temp_effort

        joint_trajectory.joint_names = conf_joint_names

    # Action initialisation
    def __init__(self, name: str) -> None:
        self._feedback = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
        self.publisher = rospy.Publisher("ra_cmd", JointState, queue_size=100)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.trajectory_point_queue: Deque[JointTrajectoryPoint] = deque()
        self.desired_joint_state = JointState(name=conf_joint_names)
        # Publisher for gazebo position controller
        self.gazebo_pub = rospy.Publisher("gazebo_arm_controller/command", Float64MultiArray, queue_size=100)
        self._as.start()

    # Action callback
    def execute_cb(self, goal: FollowJointTrajectoryGoal) -> None:
        rospy.loginfo("Executing FollowJointTrajectory Action")
        # It is required to rearrange the arrays because MoveIt doesn't guarantee order preservation
        self.rearrange(goal.trajectory)

        # A trajectory needs at least 2 points
        if len(goal.trajectory.points) < 2:
            return

        time_start = rospy.Time.from_sec(time.time())

        # ------------- Set up command queue

        self.trajectory_point_queue = deque(goal.trajectory.points)

        # ------------- Wait until the termination while providing feedback

        last_point = self.trajectory_point_queue[0]
        self.trajectory_point_queue.popleft()
        # Initialize gazebo Float array
        gazebo_positions = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        while len(self.trajectory_point_queue) > 0:
            point = self.trajectory_point_queue[0]
            self.trajectory_point_queue.popleft()
            self.desired_joint_state.position = point.positions
            self.publisher.publish(self.desired_joint_state)
            gazebo_positions.data = point.positions
            self.gazebo_pub.publish(gazebo_positions)
            # Wait
            rospy.sleep(point.time_from_start - last_point.time_from_start)
            # Trajectory abort!
            # To abort the current movement, it is possible to send an empty trajectory
            if self._as.is_preempt_requested():
                self.publisher.publish(joint_states)
                self._as.set_preempted()
                return
            # ---------------------------------------
            # Feedback
            self._feedback.joint_names = goal.trajectory.joint_names
            self._feedback.desired = point
            with lock:
                self._feedback.actual.positions = joint_states.position
                self._feedback.actual.velocities = joint_states.velocity
                self._feedback.actual.time_from_start = rospy.Time.from_sec(time.time()) - time_start
            self._feedback.error.positions = np.subtract(
                self._feedback.desired.positions, self._feedback.actual.positions
            ).tolist()
            self._feedback.error.velocities = np.subtract(
                self._feedback.desired.velocities, self._feedback.actual.velocities
            ).tolist()
            self._as.publish_feedback(self._feedback)
            # ---------------------------------------
            # Abort upon exceeding error threshold
            error = error_threshold_exceeded(self._feedback)
            if error:
                self._result.error_code = -4  # PATH_TOLERANCE_VIOLATED
                self._result.error_string = error
                self._as.set_succeeded(self._result)
                self.publisher.publish(joint_states)
                return

            last_point = point

        # ---------------------------------------
        # Result
        self._result.error_code = 0
        self._as.set_succeeded(self._result)
        # ---------------------------------------


if __name__ == "__main__":
    rospy.init_node("mrover_arm_follow_joint_trajectory")
    rospy.Subscriber("joint_states", JointState, joint_states_callback)
    MoveItAction("arm_controller/follow_joint_trajectory")
    rospy.spin()
