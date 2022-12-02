#!/usr/bin/env python3
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

class FakeJointStates:
	def __init__(self) -> None:
		self.publisher = rospy.Publisher("joint_states", JointState, queue_size=100)
		self.joint_state_to_publish = JointState()

	def trajectory_callback(self, goal: FollowJointTrajectoryGoal):
		self.joint_state_to_publish.position = goal.trajectory.points[0].positions
		self.publisher.publish(self.joint_state_to_publish)
		last_point = self.joint_state_to_publish.position = goal.trajectory.points[0]
		for point, i in enumerate(goal.trajectory.points[1::]):
			rospy.sleep(point.time_from_start - last_point.time_from_start)
			self.joint_state_to_publish.position = point.positions
			self.publisher.publish(self.joint_state_to_publish)
			last_point = point


if __name__ == "__main__":
	rospy.init_node("fake_joint_state_publisher")
	fake_joint_states = FakeJointStates()
	rospy.Subscriber("arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryGoal, fake_joint_states.trajectory_callback)
	rospy.spin()