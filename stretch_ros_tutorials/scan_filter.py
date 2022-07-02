#!/usr/bin/env python3

# Every python controller needs these lines
import rclpy
from rclpy.node import Node
from numpy import linspace, inf
from math import sin

# We're going to subscribe to a LaserScan message
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
	def __init__(self):
		# Initialize a ROS node named scan_filter
		super().__init__('stretch_scan_filter')

		# Set up a publisher that will publish on a topic called "filtered_scan",
		# with a LaserScan message type
		self.pub = self.create_publisher(LaserScan, '/filtered_scan', 10) #/stretch_diff_drive_controller/cmd_vel for gazebo

		# Set up a subscriber.  We're going to subscribe to the topic "scan",
		# looking for LaserScan messages.  When a message comes in, ROS is going
		# to pass it to the function "callback" automatically
		self.sub = self.create_subscription(LaserScan, '/scan', self.scan_filter_callback, 10)
		
		# We're going to assume that the robot is pointing up the x-axis, so that
		# any points with y coordinates further than half of the defined
		# width (1 meter) from the axis are not considered
		self.width = 1
		self.extent = self.width / 2.0

		self.get_logger().info("Publishing the filtered_scan topic. Use RViz to visualize.")

	def scan_filter_callback(self,msg):
		# Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
		# laser, with different numbers of beams
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# Work out the y coordinates of the ranges
		points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]

		# If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

		# Substitute in the new ranges in the original message, and republish it
		msg.ranges = new_ranges
		self.pub.publish(msg)


def main(args=None):
	# First the rclpy library is initialized
	rclpy.init(args=args)

	# Create object of the ScanFilter class
	scan_filter = ScanFilter()

	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages
	rclpy.spin(scan_filter)

	scan_filter.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
