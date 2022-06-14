#!/usr/bin/env python

# This node will publish a spherical marker above the current robot position, as
# it drives around the world.

# Import rospy and sys
from time import sleep
import rclpy
from rclpy.node import Node

# Import the Marker message type from the visualization_msgs package.
from visualization_msgs.msg import Marker

class Balloon(Node):
	def __init__(self):
		# Create a marker.  Markers of all shapes share a common type.
		self.marker = Marker()

		# Set the frame ID and type.  The frame ID is the frame in which the position of the marker
		# is specified.  The type is the shape of the marker, detailed on the wiki page.
		self.marker.header.frame_id = '/base_link'
		self.marker.header.stamp = self.get_clock().now().to_msg()
		self.marker.type = self.marker.SPHERE

		# Each marker has a unique ID number. If you have more than one marker that
		# you want displayed at a given time, then each needs to have a unique ID
		# number.If you publish a new marker with the same ID number and an existing
		# marker, it will replace the existing marker with that ID number.
		self.marker.id = 0

		# Set the action.  We can add, delete, or modify markers.
		self.marker.action = self.marker.ADD

		# These are the size parameters for the marker. These will vary by marker type
		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.scale.z = 0.5

		# Color, as an RGB triple, from 0 to 1.
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0

		# Alpha value, from 0 (invisible) to 1 (opaque). If you don't set this and
		# it defaults to zero, then your marker will be invisible.
		self.marker.color.a = 1.0

		# Specify the pose of the marker. Since spheres are rotationally invarient,
		# we're only going to specify the positional elements. As usual, these are
		# in the coordinate frame named in frame_id. In this case, the position
		# will always be directly above the robot, and will move with it.
		self.marker.pose.position.x = 0.0
		self.marker.pose.position.y = 0.0
		self.marker.pose.position.z = 2.0

	def publish_marker(self):
		# publisher the marker
		self.publisher_.publish(self.marker)

	def main():
		# First the rclpy library is initialized
		rclpy.init(args=args)
		
		# Initialize the node, as usual
		node = rclpy.create_node('marker')

		ballon = Balloon()

		# Set up a publisher.  We're going to publish on a topic called balloon.
		self.publisher_ = rclpy.create_publisher(Marker, 'balloon', 10)

		# This will loop until ROS shuts down the node.  This can be done on the
		# command line with a ctrl-C, or automatically from roslaunch.
		while rclpy.ok():
			ballon.publish_marker()
			sleep(1.0)
	
		self.destroy_node()	
		rclpy.shutdown()


if __name__ == '__main__':
	balloon = Balloon()
	balloon.main()
