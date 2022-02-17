#!/usr/bin/env python

### This node is still WIP, proceed with caution

# Import rclpy so its Node class can be used.
import threading
import rclpy
from rclpy.node import Node

# Imports the built-in Twist message type that the node uses to structure the velocity data that it passes on the topic.
from geometry_msgs.msg import Twist

# The Move class is created, which inherits from (or is a subclass of) Node
class Move(Node):
	def __init__(self):
		# Setup a publisher that will send the velocity commands for the Stretch
		# This will publish on a topic called "/stretch/cmd_vel" with a message type Twist.
		self.publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10) #/stretch_diff_drive_controller/cmd_vel for gazebo

	def move_forward(self):
		# Make a Twist message.  We're going to set all of the elements, since we
		# can't depend on them defaulting to safe values.
		command = Twist()

		# A Twist has three linear velocities (in meters per second), along each of the axes.
		# For Stretch, it will only pay attention to the x velocity, since it can't
		# directly move in the y direction or the z direction.
		command.linear.x = 0.1
		command.linear.y = 0.0
		command.linear.z = 0.0

		# A Twist also has three rotational velocities (in radians per second).
		# The Stretch will only respond to rotations around the z (vertical) axis.
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0

		# Publish the Twist message
		self.publisher.publish(command)

if __name__ == '__main__':
	# First the rclpy library is initialized
	rclpy.init(args=args)
	
	# Initialize the node, and call it "move".
	node = rclpy.create_node('move')
	
	# Setup Move class to base_motion
	base_motion = Move()
	
	# Spin in a separate thread
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()
	
	# create_rate allows us to control the (approximate) rate at which we publish things.
	# For this example, we want to publish at 10Hz.
	rate = node.create_rate(10)

	# This will loop until ROS shuts down the node.  This can be done on the
	# command line with a ctrl-C, or automatically from roslaunch.
	while rclpy.ok():
		# Run the move_foward function in the Move class
		base_motion.move_forward()

		# Do an idle wait to control the publish rate.  If we don't control the
		# rate, the node will publish as fast as it can, and consume all of the
		# available CPU resources.  This will also add a lot of network traffic,
		# possibly slowing down other things.
		rate.sleep()
		
	rclpy.shutdown()
	thread.join()
