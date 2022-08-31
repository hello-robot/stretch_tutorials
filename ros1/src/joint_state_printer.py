#!/usr/bin/env python3

# Import ROS Python basic API and sys
import rospy
import sys

# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
from sensor_msgs.msg import JointState

class JointStatePublisher():
	"""
	A class that prints the positions of desired joints in Stretch.
	"""

	def __init__(self):
		"""
		Function that initializes the subscriber.
		:param self: The self reference
		"""
		# Set up a subscriber. We're going to subscribe to the topic "joint_states"
		self.sub = rospy.Subscriber('joint_states', JointState, self.callback)


	def callback(self, msg):
		"""
		Callback function to deal with the incoming JointState messages.
		:param self: The self reference.
		:param msg: The JointState message.
		"""
		# Store the joint messages for later use
		self.joint_states = msg


	def print_states(self, joints):
		"""
		print_states function to deal with the incoming JointState messages.
		:param self: The self reference.
		:param joints: A list of string values of joint names.
		"""
		# Create an empty list that will store the positions of the requested joints
		joint_positions = []

		# Use of forloop to parse the names of the requested joints list.
		# The index() function returns the index at the first occurrence of
		# the name of the requested joint in the self.joint_states.name list
		for joint in joints:
			if joint == "wrist_extension":
				index = self.joint_states.name.index('joint_arm_l0')
				joint_positions.append(4*self.joint_states.position[index])
				continue

			index = self.joint_states.name.index(joint)
			joint_positions.append(self.joint_states.position[index])

		# Print the joint position values to the terminal
		print("name: " + str(joints))
		print("position: " + str(joint_positions))

		# Sends a signal to rospy to shutdown the ROS interfaces
		rospy.signal_shutdown("done")

		# Exit the Python interpreter
		sys.exit(0)



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('joint_state_printer', anonymous=True)

	# Instantiate a `JointStatePublisher()` object
	JSP = JointStatePublisher()

	# Use the rospy.sleep() function to allow the class to initialize before
	# requesting to publish joint_positions of desired joints (running the
	# print_states() function)
	rospy.sleep(.1)

	# Create a list of the joints and name them joints. These will be an argument
	# for the print_states() function
	joints = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]
	# joints = ["joint_head_pan","joint_head_tilt", "joint_gripper_finger_left", "joint_gripper_finger_right"]
	JSP.print_states(joints)

	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages
	rospy.spin()
