#!/usr/bin/env python3

# Import modules
import rospy

# Import the Marker message type from the visualization_msgs package
from visualization_msgs.msg import Marker

class Balloon():
	"""
	A class that attaches a Sphere marker directly above the Stretch robot.
	"""
	def __init__(self):
		"""
		Function that initializes the marker's features.
		:param self: The self reference.
		"""
		# Set up a publisher.  We're going to publish on a topic called balloon
		self.pub = rospy.Publisher('balloon', Marker, queue_size=10)

		# Create a marker.  Markers of all shapes share a common type
		self.marker = Marker()

		# Set the frame ID and type.  The frame ID is the frame in which the position of the marker
		# is specified.  The type is the shape of the marker, detailed on the wiki page
		self.marker.header.frame_id = 'base_link'
		self.marker.header.stamp = rospy.Time()
		self.marker.type = self.marker.SPHERE

		# Each marker has a unique ID number. If you have more than one marker that
		# you want displayed at a given time, then each needs to have a unique ID
		# number.If you publish a new marker with the same ID number and an existing
		# marker, it will replace the existing marker with that ID number
		self.marker.id = 0

		# Set the action.  We can add, delete, or modify markers
		self.marker.action = self.marker.ADD

		# These are the size parameters for the marker. These will vary by marker type
		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.scale.z = 0.5

		# Color, as an RGB triple, from 0 to 1
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0

		# Alpha value, from 0 (invisible) to 1 (opaque). If you don't set this and
		# it defaults to zero, then your marker will be invisible
		self.marker.color.a = 1.0

		# Specify the pose of the marker. Since spheres are rotationally invarient,
		# we're only going to specify the positional elements. As usual, these are
		# in the coordinate frame named in frame_id. In this case, the position
		# will always be directly above the robot, and will move with it
		self.marker.pose.position.x = 0.0
		self.marker.pose.position.y = 0.0
		self.marker.pose.position.z = 2.0

		# Create log message
		rospy.loginfo("Publishing the balloon topic. Use RViz to visualize.")

	def publish_marker(self):
		"""
		Function that publishes the sphere marker.
		:param self: The self reference.

		:publishes self.marker: Marker message.
		"""
		# publisher the marker
		self.pub.publish(self.marker)

if __name__ == '__main__':
	# Initialize the node, as usual
	rospy.init_node('marker')

	# Instanstiate a `Balloon()` object
	balloon = Balloon()

	# Set a rate
	rate = rospy.Rate(10)

	# Publish the marker at 10Hz
	while not rospy.is_shutdown():
		balloon.publish_marker()
		rate.sleep()
