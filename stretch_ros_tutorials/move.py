#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Imports the built-in Twist message type that the node uses to structure the velocity data
from geometry_msgs.msg import Twist


class Move(Node):
	def __init__(self):
		super().__init__('stretch_base_move')
		
		# Setup a publisher that will send the velocity commands to the base
		# This will publish on a topic called "/stretch/cmd_vel" with a message type Twist
		self.publisher_ = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
		
		self.get_logger().info("Starting to move in circle...")

		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.move_around)

	def move_around(self):
		# Define a Twist message
		command = Twist()

		# A Twist has three linear velocities (in meters per second), along each of the axes
		# Stretch has a ddifferential drive base and can't move laterally, it will only pay 
		# attention to velocity along the x-axis
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0

		# A Twist also has three rotational velocities (in radians per second)
		# Stretch will only respond to rotations along the z-axis
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.5

		# Publish the Twist message
		self.publisher_.publish(command)

	
def main(args=None):
	rclpy.init(args=args)
	
	base_motion = Move()

	rclpy.spin(base_motion)

	base_motion.destroy_node()	
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	
