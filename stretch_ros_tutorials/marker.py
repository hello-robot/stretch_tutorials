#!/usr/bin/env python3

# This node will publish a spherical marker above the current robot position, as
# it drives around the world

import rclpy
from rclpy.node import Node

# Import the Marker message type from the visualization_msgs package
from visualization_msgs.msg import Marker

class Balloon(Node):
	def __init__(self):
		# Initialize the node named marker
		super().__init__('stretch_marker')

		# Set up a publisher that will publish on a topic called balloon
		self.publisher_ = self.create_publisher(Marker, 'balloon', 10)	
		
		# Markers of all shapes share a common type
		self.marker = Marker()

		# The frame ID is the frame in which the position of the marker is specified
		# Refer to the wiki page for more marker types
		self.marker.header.frame_id = '/base_link'
		self.marker.header.stamp = self.get_clock().now().to_msg()
		self.marker.type = self.marker.SPHERE

		# If you have more than one marker that you want displayed at a given time,
		# then each needs to have a unique ID number
		self.marker.id = 0

		# You can Add, Delete, or Modify markers by defining the action
		self.marker.action = self.marker.ADD

		# Define the size parameters for the marker
		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.scale.z = 0.5

		# Define the color as an RGB triple from 0 to 1
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0

		# Define the Alpha value, from 0 (invisible) to 1 (opaque)
		self.marker.color.a = 1.0

		# Specify the pose of the marker. Since spheres are rotationally invariant,
		# we're only going to specify the positional elements. These are
		# in the coordinate frame named in frame_id. In this case, the position
		# will always be directly above the robot, and will move with it
		self.marker.pose.position.x = 0.0
		self.marker.pose.position.y = 0.0
		self.marker.pose.position.z = 2.0

		self.get_logger().info("Publishing the balloon topic. Use RViz to visualize.")

	def publish_marker(self):
		# publisher the marker
		self.publisher_.publish(self.marker)

def main(args=None):
	rclpy.init(args=args)

	balloon = Balloon()

	# This will loop until ROS shuts down the node.  This can be done on the
	# command line with a ctrl-C, or automatically from roslaunch.
	while rclpy.ok():
		balloon.publish_marker()

	balloon.destroy_node()	
	rclpy.shutdown()


if __name__ == '__main__':
	main()
