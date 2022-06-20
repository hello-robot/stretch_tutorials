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

class MultiPointCommand(hm.HelloNode):
	"""
	A class that sends multiple joint trajectory goals to the stretch robot.
	"""

	# Initialize the inhereted hm.Hellonode class.
	def __init__(self):
		hm.HelloNode.__init__(self)

	def issue_multipoint_command(self):
		"""
		Function that makes an action call and sends multiple joint trajectory goals.
		:param self: The self reference.
		"""
		# Set point0 as a JointTrajectoryPoint().
		point0 = JointTrajectoryPoint()

		# Provide desired positions of lift, wrist extension, and yaw of
		# the wrist (in meters).
		point0.positions = [0.2, 0.0, 3.4]

		# Provide desired velocity of the lift (m/s), wrist extension (m/s),
		# and wrist yaw (rad/s).
		# IMPORTANT NOTE: The lift and wrist extension can only go up to 0.2 m/s!
		point0.velocities = [0.2, 0.2, 2.5]

		# Provide desired velocity of the lift (m/s^2), wrist extension (m/s^2),
		# and wrist yaw (rad/s^2).
		point0.accelerations = [1.0, 1.0, 3.5]

		# Set positions for the following 5 trajectory points.
		# IMPORTANT NOTE: If you do not provide any velocities or accelerations for the lift
		# or wrist extension, then they go to their default values. However, the
		# Velocity and Acceleration of the wrist yaw will stay the same from the
		# previous value unless updated.
		point1 = JointTrajectoryPoint()
		point1.positions = [0.3, 0.1, 2.0]

		point2 = JointTrajectoryPoint()
		point2.positions = [0.5, 0.2, -1.0]

		point3 = JointTrajectoryPoint()
		point3.positions = [0.6, 0.3, 0.0]

		point4 = JointTrajectoryPoint()
		point4.positions = [0.8, 0.2, 1.0]

		point5 = JointTrajectoryPoint()
		point5.positions = [0.5, 0.1, 0.0]

		# Set trajectory_goal as a FollowJointTrajectoryGoal and define
		# the joint names as a list.
		trajectory_goal = FollowJointTrajectoryGoal()
		trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']

		# Then trajectory_goal.trajectory.points is defined by a list of the joint
		# trajectory points
		trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]

		# Specify the coordinate frame that we want (base_link) and set the time to be now.
		trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
		trajectory_goal.trajectory.header.frame_id = 'base_link'

		# Make the action call and send the goal. The last line of code waits
		# for the result before it exits the python script.
		self.trajectory_client.send_goal(trajectory_goal)
		rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
		self.trajectory_client.wait_for_result()

	# Create a funcion, main(), to do all of the setup the hm.HelloNode class
	# and issue the stow command.
	def main(self):
		"""
		Function that initiates the multipoint_command function.
		:param self: The self reference.
		"""
		# The arguments of the main function of the hm.HelloNode class are the
		# node_name, node topic namespace, and boolean (default value is true).
		hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
		rospy.loginfo('issuing multipoint command...')
		self.issue_multipoint_command()
		time.sleep(2)


if __name__ == '__main__':
    try:
        # Initialize the MultiPointCommand() class and set it to node and run the
        # main() function.
        node = MultiPointCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
