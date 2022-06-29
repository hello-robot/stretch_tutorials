#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import time

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot.
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories.
from trajectory_msgs.msg import JointTrajectoryPoint

# Import hello_misc script for handling trajecotry goals with an action client.
import hello_helpers.hello_misc as hm

class SingleJointActuator(hm.HelloNode):
	"""
	A class that sends multiple joint trajectory goals to a single joint.
	"""
	# Initialize the inhereted hm.Hellonode class.
	def __init__(self):
		hm.HelloNode.__init__(self)

	def issue_command(self):
		"""
		Function that makes an action call and sends joint trajectory goals
		to a single joint
		:param self: The self reference.
		"""
		# Here is a list of joints and their recommended position limits:
		############################# Joint limits #############################
		# joint_lift:      lower_limit =  0.15,  upper_limit =  1.10  # in meters
		# wrist_extension: lower_limit =  0.00,  upper_limit =  0.50  # in meters
		# joint_wrist_yaw: lower_limit = -1.75,  upper_limit =  4.00  # in radians
		# joint_head_pan:  lower_limit = -2.80,  upper_limit =  2.90  # in radians
		# joint_head_tilt: lower_limit = -1.60,  upper_limit =  0.40  # in radians
		# joint_gripper_finger_left:  lower_limit = -0.35,  upper_limit =  0.165  # in radians
		########################################################################

		# Set trajectory_goal as a FollowJointTrajectoryGoal and define
		# the joint name.
		trajectory_goal = FollowJointTrajectoryGoal()
		trajectory_goal.trajectory.joint_names = ['joint_head_pan']

		# Provide desired positions for joint name.
		# Set positions for the following 5 trajectory points.
		point0 = JointTrajectoryPoint()
		point0.positions = [0.65]

		# point1 = JointTrajectoryPoint()
		# point1.positions = [-1.50]

		# Then trajectory_goal.trajectory.points is set as a list of the joint
		# trajectory points
		trajectory_goal.trajectory.points = [point0]#, point1]

		# Specify the coordinate frame that we want (base_link) and set the time to be now.
		trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
		trajectory_goal.trajectory.header.frame_id = 'base_link'

		# Make the action call and send the goal. The last line of code waits
		# for the result before it exits the python script.
		self.trajectory_client.send_goal(trajectory_goal)
		rospy.loginfo('Sent goal = {0}'.format(trajectory_goal))
		self.trajectory_client.wait_for_result()

	def main(self):
		"""
		Function that initiates the issue_command function.
		:param self: The self reference.
		"""
		# The arguments of the main function of the hm.HelloNode class are the
		# node_name, node topic namespace, and boolean (default value is true).
		hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
		rospy.loginfo('issuing command...')
		self.issue_command()
		time.sleep(2)


if __name__ == '__main__':
	try:
	# Initialize the SingleJointActuator() class and set it to node and run the
	# main() function.
		node = SingleJointActuator()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
