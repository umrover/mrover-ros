#!/usr/bin/env python3
# Adapted from https://github.com/ros-industrial/robot_movement_interface/blob/master/ur_driver/scripts/joint_trajectory_action.py
import rospy
import time
import actionlib
import _thread
from collections import deque

from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

conf_joint_names = rospy.get_param("teleop/ra_joints/")
lock = _thread.allocate_lock()
joint_states = JointState()


# ------------------------------------------------------------------------
# Callback function executed after the publication of the current robot position
# ------------------------------------------------------------------------
def joint_states_callback(msg: JointState):
    with lock:
        global joint_states
        joint_states = msg


class MoveItAction(object):

    # Rearranges the point path following the name convention joint_0, ... joint_6
    def rearrange(self, joint_trajectory: JointTrajectory) -> None:

        mapping = [joint_trajectory.joint_names.index(j) for j in conf_joint_names]

        # Return early if already arranged properly
        if mapping == sorted(mapping):
            return
        for point in joint_trajectory.points:

            temp_positions = []
            temp_velocities = []
            temp_accelerations = []
            temp_effort = []

            for i in range(len(point.positions)):
                temp_positions.append(point.positions[mapping[i]])
            for i in range(len(point.velocities)):
                temp_velocities.append(point.velocities[mapping[i]])
            for i in range(len(point.accelerations)):
                temp_accelerations.append(point.accelerations[mapping[i]])
            for i in range(len(point.effort)):
                temp_effort.append(point.effort[mapping[i]])

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
        self.trajectory_point_queue = deque()
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
            self._as.publish_feedback(self._feedback)
            # ---------------------------------------
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
